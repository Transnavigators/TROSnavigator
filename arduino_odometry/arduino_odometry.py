#!/usr/bin/env python
import smbus
import struct
import rospy
import math
import tf
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry


class ArduinoOdometry:

    # Commands for sending and receiving I2C packets
    
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_odometry', anonymous=True)

        self.bus = smbus.SMBus(1)
        self.encoder_cmd = ord('e')
        self.address = 0x04

        # The width between the wheels
        self.width = rospy.get_param("~width", 31.5 * 0.0254)
        self.radius = float(self.width) / 2

        # Constant distance travelled per pulse of the encoder
        # 6" diameter wheel, 4096 pulses per revolution
        self.meters_per_pulse = float(rospy.get_param("~meters_per_pulse", 2 * math.pi * (6 / 2) * 0.0254 / 4096))
        self.rate = float(rospy.get_param("~poll_rate", 10))
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    # get data from Arduino
    def read_encoders(self):
        data = self.bus.read_i2c_block_data(self.address, self.encoder_cmd)
        return data
         
    # start the node: spin forever
    def begin(self):
        old_left = 0
        old_right = 0
        
        previous_time = 0

        th = 0
        x = 0
        y = 0

        rate = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            # read encoders
            try:
                receive_data = self.read_encoders()
                new_left, new_right = struct.unpack('=ii', bytearray(receive_data[0:8]))
            
                rospy.loginfo_throttle(1, "Left count: "+str(new_left)+" | Right count: "+str(new_right))
            
                current_time = rospy.get_time()
                now = rospy.Time.now()

                # adapted from Mastering ROS for Robotics Programming page 303-306
                # https://github.com/qboticslabs/mastering_ros/blob/master/chapter_9_codes/chefbot_navig_cpp/src/diff_tf.cpp
                ##################################################################
                delta_time = current_time - previous_time
                delta_left = (new_left - old_left) * self.meters_per_pulse # m
                delta_right = (new_right - old_right) * self.meters_per_pulse # m

                old_left = new_left
                old_right = new_right
                previous_time = current_time

                if abs(delta_left - delta_right) < 1e-6:
                    dx = delta_left * math.cos(th)
                    dy = delta_right * math.sin(th)
                    vth = 0
                else:
                    r = self.width * (delta_right + delta_left) / (2 * (delta_right - delta_left)) # m
                    wd = (delta_right - delta_left) / self.width  # radians
                    dx = r * math.sin(wd + th) - r * math.sin(th)  # delta x position
                    dy = r * math.cos(wd + th) + r * math.cos(th)  # delta y position
                    th = (th + wd + (2 * math.pi)) % (2 * math.pi)  # delta theta
                    vth = wd / delta_time  # radians/sec
                x = x + dx  # absolute x position starting at 0,0
                y = y + dy  # absolute y position starting at 0,0
                vx = dx / delta_time  # m/s
                vy = dy / delta_time  # m/s

                # send messages
                odom_quat = Quaternion()
                odom_quat.w = math.cos(th / 2)
                odom_quat.x = 0.0
                odom_quat.y = 0.0
                odom_quat.z = math.sin(th/2)

                # send the transform
                self.odom_broadcaster.sendTransform((x, y, 0.0), [odom_quat.w, 0.0, 0.0, odom_quat.z], now, "base_link", "odom")
            
                # next, we'll publish the odometry message over ROS
                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = "odom"
                # set the position
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = odom_quat
                # set the velocity
                odom.child_frame_id = "base_link"
                odom.twist.twist.linear.x = vx
                odom.twist.twist.linear.y = vy
                odom.twist.twist.angular.z = vth
            
                self.pub.publish(odom)
            
                ###########################################################
                rate.sleep()
            except IOError as e:
                # rospy.logwarn(e)
                pass

if __name__ == "__main__":
    try:
        controller = ArduinoOdometry()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
