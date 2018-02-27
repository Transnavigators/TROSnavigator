#!/usr/bin/env python
import smbus
import struct
import rospy
import serial
import math
import os
import sys
import tf
import time
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry


class Arduino:

    # commands for sending and receiving it
    
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino', anonymous=True)

        self.bus = smbus.SMBus(1)

        self.move_cmd = ord('m')
        self.encoder_cmd = ord('e')
        self.address = 0x04

        # The width between the wheels
        if rospy.has_param("~width"):
            self.width = rospy.get_param("~width")
            self.radius = float(self.width) / 2
        else:
            self.width = 31.5 * 0.0254
            self.radius = self.width / 2
            

        # Constant distance travelled per pulse of the encoder
        # 6" diameter wheel, 4096 pulses per revolution
        if rospy.has_param("~meters_per_pulse"):
            self.meters_per_pulse = rospy.get_param("~meters_per_pulse")
        else:
            self.meters_per_pulse = 2 * math.pi * (6 / 2) * 0.0254 / 4096
            
        # Subscribe to the arduino_commands topic
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    # callback for receiving data from the Arduino
    def callback(self, msg):
        # Convert twist to motor commands from https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/

        # vel_r = (msg.angular.z*self.width)/2 + msg.linear.x;
        # vel_l = msg.linear.x*2-vel_r;
        
        # additional Conversion from https://github.com/vcwu/ros-differential-drive/blob/master/nodes/twist_to_motors.py
        
        # from arduino controller
        linear_vel = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2) * 1e6
        angular_vel = msg.angular.z * self.radius * 1e6
        vel_l = int(linear_vel - angular_vel)
        vel_r = int(linear_vel + angular_vel)

        # scale from -127 to 128 (sat currently)
        if (vel_l < -127):
            vel_l = -127
        if (vel_l > 128):
            vel_l = 128
        if (vel_r < -127):
            vel_r = -127
        if (vel_r > 128):
            vel_r = 128
        
        self.sendSpeedToMotor(vel_1,vel_r)
        rospy.loginfo_throttle(1, "Sending vel1=%d vel2=%d" % (vel_l, vel_r))

    # send data to arduino
    def sendSpeedToMotor(self,m1,m2):
        self.bus.write_i2c_block_data(self.address, self.move_cmd, [m1, m2]);
        
    # get data from arduino
    def readEncoders(self):
        data = self.bus.read_i2c_block_data(self.address, self.encoder_cmd)
        return data
         
    # start the node: spin forever
    def begin(self):
        old_left = 0;
        old_right = 0;
        
        previous_time = rospy.get_time()
        
        x_final = 0
        y_final = 0
        theta_final = 0
        
        
        
        while not rospy.is_shutdown():
            time.sleep(1)
            # read encoders
            receive_data = self.readEncoders()
            new_left,new_right = struct.unpack('ii',bytearray(receive_data[0:8]))
            
            current_time = rospy.get_time()
            now = rospy.Time.now()

            # adapted from Mastering ROS for Robotics Programming page 303-306
            # https://github.com/qboticslabs/mastering_ros/blob/master/chapter_9_codes/chefbot_navig_cpp/src/diff_tf.cpp
            ##################################################################
            elapsed = current_time - previous_time
            if old_left == 0: # reset? we might not need this check 
                d_left = 0
                d_right = 0
            else:
                d_left = (new_left - old_left) * self.meters_per_pulse
                d_right = (new_right - old_right) * self.meters_per_pulse

            old_left = new_left
            old_right = new_right
            previous_time = current_time
            
            d = (d_left + d_right ) / 2.0
            th = ( d_right - d_left ) / self.width
            dx = d /elapsed
            dr = th / elapsed
            if d != 0:
                x = math.cos( th ) * d
                y = -math.sin( th ) * d
                # calculate the final position of the robot
                x_final = x_final + ( math.cos( theta_final ) * x - math.sin(theta_final ) * y )
                y_final = y_final + ( math.sin( theta_final ) * x + math.cos(theta_final ) * y )
            if th != 0:
                theta_final = theta_final + th
                
                
                
            # send messages
            odom_quat = Quaternion()
            odom_quat.x = 0.0
            odom_quat.y = 0.0
            odom_quat.z = 0.0
            
            odom_quat.z = math.sin( theta_final / 2 )
            odom_quat.w = math.cos( theta_final / 2 )
            
            # # first, we'll publish the transform over tf
            # odom_trans = TransformStamped()
            # odom_trans.header.stamp = now
            # odom_trans.header.frame_id = "odom"
            # odom_trans.child_frame_id = "base_link"
            
            # odom_trans.transform.translation.x = x_final
            # odom_trans.transform.translation.y = y_final
            # odom_trans.transform.translation.z = 0.0
            # odom_trans.transform.rotation = odom_quat
            #send the transform
            self.odom_broadcaster.sendTransform((x_final,y_final, 0.0),
                                                tf.transformations.quaternion_from_euler(0,0,theta_final),
                                                now,
                                                "base_link",
                                                "odom")
            
            #next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            #set the position
            odom.pose.pose.position.x = x_final
            odom.pose.pose.position.y = y_final
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = odom_quat
            #set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = dr
            
            self.pub.publish(odom)
            
            ###########################################################
            


if __name__ == "__main__":
    try:
        controller = Arduino()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
