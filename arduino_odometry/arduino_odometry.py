#!/usr/bin/env python
import smbus
import struct
import rospy
import math
import tf
import sys
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry


class ArduinoOdometry:
    """Reads encoder data from the Arduino and publishes it as a transform and on the odom topic
    
    """

    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_odometry', anonymous=True)

        # Setup I2C variables
        self.encoder_cmd = ord('e')
        self.address = int(rospy.get_param("~address", 0x04))

        # The width between the wheels
        self.width = float(rospy.get_param("~width", 31.5 * 0.0254))
        self.radius = self.width / 2.0

        # Constant distance travelled per pulse of the encoder
        # 6" diameter wheel, 4096 pulses per revolution
        self.meters_per_pulse = float(rospy.get_param("~meters_per_pulse", 2 * math.pi * (6 / 2) * 0.0254 / 4096))
        self.rate = rospy.Rate(float(rospy.get_param("~poll_rate", 10)))

        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0

        is_virtual = int(rospy.get_param("~is_virtual", 0))
        # Setup the i2c bus
        if is_virtual:
            self.bus = smbus.SMBus(0)
        else:
            rospy.sleep(4.0)
            self.bus = smbus.SMBus(1)

        self.odom_broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)

    # get data from Arduino
    def read_encoders(self):
        """Reads the encoder count from the Arduino using I2C
        
        Returns: 
            tuple: The left and right encoder counts as ints
        """
        data = self.bus.read_i2c_block_data(self.address, self.encoder_cmd)
        left, right = struct.unpack('=ii', bytearray(data[0:8]))

        return -left, -right

    # start the node: spin forever
    def begin(self):
        """Keeps reading the encoder, determines the new position and velocity, and publishes it to the odom topic

        """
        old_left = 0
        old_right = 0
        previous_time = 0

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # Construct Odom message
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # set the position
            odom.pose.pose.position.z = 0.0
            odom.twist.twist.linear.y = 0.0

            # Set covariances, from turtlebot
            # TODO: find actual covariances
            odom.pose.covariance = [1e-9, 0, 0, 0, 0, 0,
                                    0, 1e-3, 1e-9, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e-9]
            odom.twist.covariance = [1e-9, 0, 0, 0, 0, 0,
                                     0, 1e-3, 1e-9, 0, 0, 0,
                                     0, 0, 1e6, 0, 0, 0,
                                     0, 0, 0, 1e6, 0, 0,
                                     0, 0, 0, 0, 1e6, 0,
                                     0, 0, 0, 0, 0, 1e-9]

            odom_quat = Quaternion()
            odom_quat.x = 0.0
            odom_quat.y = 0.0

            try:
                # read encoders
                new_left, new_right = self.read_encoders()
                rospy.loginfo_throttle(1, "Left count: " + str(new_left) + " | Right count: " + str(new_right))

                current_time = rospy.get_time()
                # adapted from Mastering ROS for Robotics Programming page 303-306
                # https://github.com/qboticslabs/mastering_ros/blob/master/chapter_9_codes/chefbot_navig_cpp/src/diff_tf.cpp
                ##################################################################
                delta_time = current_time - previous_time
                delta_left = (new_left - old_left) * self.meters_per_pulse  # m
                delta_right = (new_right - old_right) * self.meters_per_pulse  # m

                d = (delta_left + delta_right) / self.width
                th = (delta_right - delta_left) / self.width
                self.dx = d / delta_time
                self.dr = th / delta_time
                old_left = new_left
                old_right = new_right
                previous_time = current_time
                if d != 0:
                    x = math.cos(th) * d
                    y = -math.sin(th) * d
                    self.x = self.x + (math.cos(self.th) * x - math.sin(self.th) * y)
                    self.y = self.y + (math.sin(self.th) * x + math.cos(self.th) * y)
                if th != 0:
                    self.th = self.th + th
                rospy.loginfo_throttle(1, "dLeft=%f dRight=%f th=%f x=%f y=%f dx=%f dr=%f" % (
                    delta_left, delta_right, th, self.x, self.y, self.dx, self.dr))
                # update odom quaternion
                odom_quat.w = math.cos(th / 2)
                odom_quat.z = math.sin(th / 2)
            except IOError as e:
                rospy.logwarn(e)
                odom_quat.w = 1.0
                odom_quat.z = 0.0
            # send the transform
            self.odom_broadcaster.sendTransform((self.x, self.y, 0.0), [0.0, 0.0, odom_quat.z, odom_quat.w], now,
                                                "base_link", "odom")
            # Update odom message and send it
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.twist.twist.linear.x = -self.dx / 2
            odom.twist.twist.angular.z = self.dr
            odom.pose.pose.orientation = odom_quat
            self.pub.publish(odom)
            self.rate.sleep()


if __name__ == "__main__":
    controller = None
    try:
        controller = ArduinoOdometry()
        controller.begin()
    except rospy.ROSInterruptException:
        controller.bus.close()
        pass
