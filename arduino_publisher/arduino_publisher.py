#!/usr/bin/env python

import rospy
import serial
import tf
import math
from PyCRC.CRC16 import CRC16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


class ArduinoPublisher:
    def __init__(self):
        # initialize port
        port_name = rospy.get_param("~port")
        self.ser = serial.Serial(port=port_name, baudrate=115200, timeout=0)

        # set up node
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        rospy.init_node('arduino_pub', anonymous=True)

        # Set the polling rate for checking serial messages
        if rospy.has_param("~rate"):
            self.RATE = rospy.get_param("~rate")
        else:
            self.RATE = 100

        # Constant distance travelled per pulse of the encoder
        # 6" diameter wheel, 1024 pulses per revolution
        if rospy.has_param("~m_per_pulse"):
            self.M_PER_PULSE = rospy.get_param("~m_per_pulse")
        else:
            self.M_PER_PULSE = 2 * math.pi * (6 / 2) * 0.0254 / 1024

        # Width between the wheels 31.5" - Change if necessary
        if rospy.has_param("~width"):
            self.WIDTH = rospy.get_param("~width")
        else:
            self.WIDTH = 31.5 * 0.0254

        # The difference between the encoder output of the two wheels in m for it to be considered straight
        # Tweak this to get rid of unstable swerving
        if rospy.has_param("~drift_error"):
            self.DRIFT_ERROR = rospy.get_param("~drift_error")
        else:
            self.DRIFT_ERROR = 1e-6

    # start node
    def begin(self):
        x = 0.0
        y = 0.0
        th = 0.0

        # TODO link with localino server to provide initial fix on location

        last_time = rospy.Time.now()

        rate = rospy.Rate(100)  # 100Hz, increase if needed
        # TODO: Adjust rate according to 2x Arduino's sending rate if possible
        while not rospy.is_shutdown():
            data = self.ser.read()
            if data == 0xEE:
                data = self.ser.read()
                if data == 0x01:
                    # Read 3 32bit ints and a 16bit CRC
                    x1 = self.ser.read(4)
                    x2 = self.ser.read(4)
                    d_time = self.ser.read(4)
                    crc = self.ser.read(2)

                    # Calculate the CRC to verify packet integrity
                    buf = bytearray()
                    buf.extend(x1)
                    buf.extend(x2)
                    buf.extend(d_time)
                    calc_crc = CRC16().calculate(bytes(buffer))
                    if calc_crc == crc:
                        # Display the time frame each packet represents vs the node's refresh rate
                        current_time = rospy.Time.now()
                        delta_time = int(d_time) * 1e-6
                        delta_ros_time = (current_time - last_time).to_nsec()
                        rospy.loginfo("Delta ROS Time: %f ns\tDelta Time: %f ns", delta_ros_time, delta_time*1e9)

                        # Convert number of pulses to a distance
                        delta_left = int(x1) * self.M_PER_PULSE
                        delta_right = int(x2) * self.M_PER_PULSE

                        # TODO adjust straight error according to tests
                        # Math from https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
                        if math.abs(delta_left - delta_right) < 1e-6:
                            dx = delta_left * math.cos(th)
                            dy = delta_right * math.sin(th)
                            vth = 0
                        else:
                            r = self.WIDTH * (delta_right + delta_left) / (2 * (delta_right - delta_left))
                            wd = (delta_right - delta_left) / self.WIDTH
                            dx = r * math.sin(wd + th) - r * math.sin(th)
                            dy = -r * math.cos(wd + th) + r * math.cos(th)
                            th = (th + wd + (2 * math.pi)) % (2 * math.pi)
                            vth = wd * delta_time
                        x = x + dx
                        y = y + dy
                        vx = dx * delta_time
                        vy = dy * delta_time
                        # TODO link with localino server to provide periodic fix on location

                        # Convert 1D Euler rotation to quaternion
                        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

                        self.odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_link", "odom")

                        # Construct a message with the position, rotation, and velocity
                        odom = Odometry()
                        odom.header.stamp = current_time
                        odom.header.frame_id = "odom"
                        odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
                        odom.child_frame_id = "base_link"
                        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

                        # TODO: measure covariance with experiment + statistics
                        # This says position estimate is accurate
                        odom.pose.covariance = {99999, 0, 0, 0, 0, 0,  # covariance on gps_x
                                                0, 99999, 0, 0, 0, 0,  # covariance on gps_y
                                                0, 0, 99999, 0, 0, 0,  # covariance on gps_z
                                                0, 0, 0, 99999, 0, 0,  # large covariance on rot x
                                                0, 0, 0, 0, 99999, 0,  # large covariance on rot y
                                                0, 0, 0, 0, 0, 99999}  # large covariance on rot z

                        # This says velocity estimate is accurate
                        odom.twist.covariance = {99999, 0, 0, 0, 0, 0,  # covariance on gps_x
                                                0, 99999, 0, 0, 0, 0,  # covariance on gps_y
                                                0, 0, 99999, 0, 0, 0,  # covariance on gps_z
                                                0, 0, 0, 99999, 0, 0,  # large covariance on rot x
                                                0, 0, 0, 0, 99999, 0,  # large covariance on rot y
                                                0, 0, 0, 0, 0, 99999}  # large covariance on rot z

                        self.pub.publish(odom)

                        last_time = current_time
                    else:
                        rospy.logwarn("Packet didn't pass checksum, something is wrong with Arduino->Pi communication.")

            rate.sleep()


if __name__ == "__main__":
    try:
        ap = ArduinoPublisher()
        ap.begin()
    except rospy.ROSInterruptException:
        pass
