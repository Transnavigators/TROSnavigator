#!/usr/bin/env python

import rospy
import serial
import tf
import math
import os
from PyCRC.CRC16 import CRC16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import BatteryState


class ArduinoPublisher:
    def __init__(self):
        # initialize port
        if rospy.has_param("~port"):
            self.port_name = rospy.get_param("~port")
        else:
            self.port_name = '/dev/ttyACM0'

        if rospy.has_param("~baud_rate"):
            self.baud_rate = int(rospy.get_param("~baud_rate"))
        else:
            self.baud_rate = 115200

        # Conversion factor from Arduino's AnalogRead to actual battery voltage
        if rospy.has_param("~analog_to_volts"):
            self.ANALOG_TO_VOLTAGE = rospy.get_param("~analog_to_volts")
        else:
            self.ANALOG_TO_VOLTAGE = 2 * 10 * 5.0 / 1024

        # Set the polling rate for checking serial messages
        # TODO: measure rate and adjust this value
        if rospy.has_param("~poll_rate"):
            self.RATE = rospy.get_param("~poll_rate")
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
        # TODO: Tweak this to get rid of unstable swerving
        if rospy.has_param("~drift_error"):
            self.DRIFT_ERROR = rospy.get_param("~drift_error")
        else:
            self.DRIFT_ERROR = 1e-6

        if rospy.has_param("~battery_capacity"):
            self.BATTERY_CAPACITY = float(rospy.get_param("~battery_capacity"))
        else:
            self.BATTERY_CAPACITY = 35.0

    def close_port(self):
        self.ser.close()

    # start node
    def begin(self):
    
        # set up node
        rospy.init_node('arduino_pub', anonymous=True)
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.battery_pub = rospy.Publisher("battery", BatteryState, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        
        # Change how ports are configured if in a docker container with virtual ports
        if 'INSIDEDOCKER' in os.environ:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0, rtscts=True, dsrdtr=True)
        else:
            self.ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0)
        # make sure the port is closed on exit
        rospy.on_shutdown(self.close_port)

    
    
    
    
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
                    calc_crc = CRC16().calculate(bytes(buf))
                    if calc_crc == crc:
                        # Display the time frame each packet represents vs the node's refresh rate
                        current_time = rospy.Time.now()
                        delta_time = int(d_time) * 1e-6
                        delta_ros_time = (current_time - last_time).to_nsec()
                        rospy.loginfo("Delta ROS Time: %f ns\tDelta Time: %f ns", delta_ros_time, delta_time * 1e9)

                        # Convert number of pulses to a distance
                        delta_left = int(x1) * self.M_PER_PULSE
                        delta_right = int(x2) * self.M_PER_PULSE

                        # TODO adjust straight error according to tests
                        # Math from https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
                        if abs(delta_left - delta_right) < 1e-6:
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

                        # Convert 1D Euler rotation to quaternion
                        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

                        self.odom_broadcaster.sendTransform((x, y, 0.), odom_quat, current_time, "base_link", "odom")

                        # Construct a message with the position, rotation, and velocity
                        msg = Odometry()
                        msg.header.stamp = current_time
                        msg.header.frame_id = "odom"
                        msg.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
                        msg.child_frame_id = "base_link"
                        msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

                        # TODO: measure covariance with experiment + statistics
                        # This says position estimate is accurate
                        msg.pose.covariance = {99999, 0, 0, 0, 0, 0,  # covariance on gps_x
                                               0, 99999, 0, 0, 0, 0,  # covariance on gps_y
                                               0, 0, 99999, 0, 0, 0,  # covariance on gps_z
                                               0, 0, 0, 99999, 0, 0,  # large covariance on rot x
                                               0, 0, 0, 0, 99999, 0,  # large covariance on rot y
                                               0, 0, 0, 0, 0, 99999}  # large covariance on rot z

                        # This says velocity estimate is accurate
                        msg.twist.covariance = {99999, 0, 0, 0, 0, 0,  # covariance on gps_x
                                                0, 99999, 0, 0, 0, 0,  # covariance on gps_y
                                                0, 0, 99999, 0, 0, 0,  # covariance on gps_z
                                                0, 0, 0, 99999, 0, 0,  # large covariance on rot x
                                                0, 0, 0, 0, 99999, 0,  # large covariance on rot y
                                                0, 0, 0, 0, 0, 99999}  # large covariance on rot z

                        self.pub.publish(msg)

                        last_time = current_time
                    else:
                        rospy.logwarn("Packet didn't pass checksum, something is wrong with Arduino->Pi communication.")
                elif data == 0x02:
                    batt = self.ser.read(2)
                    crc = self.ser.read(2)
                    buf = bytearray()
                    buf.extend(batt)
                    buf.extend(crc)
                    calc_crc = CRC16().calculate(bytes(buf))
                    if calc_crc == crc:
                        msg = BatteryState()
                        msg.voltage = int(batt) * self.ANALOG_TO_VOLTAGE

                        msg.capacity = self.BATTERY_CAPACITY
                        msg.design_capacity = self.BATTERY_CAPACITY

                        # Linear trendline generated by Excel of voltages vs SoC
                        msg.percentage = 86.558 * msg.voltage - 1015.4
                        msg.charge = 35 * msg.percentage / 100
                        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

                        # We are using a lead acid battery, but that type is not available in ROS or Linux's enums
                        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
                        msg.present = True

                        # Can't measure current due to ~200A starting current
                        msg.current = float('NaN')

                        # Cell voltage is impossible to measure in Lead Acid Battery
                        msg.cell_voltage = [float('NaN')] * 6

                        # TODO: extend battery reading for both batteries individually vs in series
                        msg.location = ""
                        msg.serial_number = ""

                        self.battery_pub.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        ap = ArduinoPublisher()
        ap.begin()
    except rospy.ROSInterruptException:
        pass
