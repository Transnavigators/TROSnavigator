#!/usr/bin/env python

# modified from https://answers.ros.org/question/209963/cmd_veltwist-transform-twist-message-into-left-and-right-motor-commands/
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


class ArduinoMotor:

    # commands for sending and receiving it
    
    def __init__(self):
        # Initialize the serial port
        rospy.init_node('arduino_motor', anonymous=True)
        rospy.loginfo("%s started" % rospy.get_name())

        self.width = rospy.get_param("~width", 31.5 * 0.0254)
        # if rospy.has_param("~meters_per_pulse"):
        #     self.meters_per_pulse = rospy.get_param("~meters_per_pulse")
        # else:
        #     self.meters_per_pulse = 2 * math.pi * (6 / 2) * 0.0254 / 4096
        self.constant=52.8
        self.bus = smbus.SMBus(1)

        self.move_cmd = ord('m')
        self.encoder_cmd = ord('e')
        self.address = 0x04

        rospy.Subscriber("cmd_vel", Twist, self.callback)

        self.rate = rospy.get_param("~rate", 50)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        self.ticks_since_target = 0

    def callback(self,msg):
        #rospy.loginfo("twist to motors:: twistCallback raw msg: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y  
    # send data to arduino
    def sendSpeedToMotor(self,m1,m2):
        self.bus.write_i2c_block_data(self.address, self.move_cmd, [m1, m2]);

    # start the node: spin forever
    def begin(self):
    
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(10)
        self.ticks_since_target = self.timeout_ticks
    
        ###### main loop  ######
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()
                
    def spinOnce(self):
    
        # dx = (l + r) / 2
        # dr = (r - l) / w
            
        self.right = 1.0 * self.dx + self.dr * self.width / 2 
        self.left = 1.0 * self.dx - self.dr * self.width / 2
        # rospy.loginfo("twist to motors:: spinOnce (dx:%f, dr: %f)", self.dx,self.dr)
        rospy.loginfo("twist to motors:: spinOnce (self.left:%f,self.right %f)" % (self.left,self.right) )
        rospy.loginfo("LEFT: " +str(int(self.left*self.left*self.constant))+"RIGHT: " +str(int(self.right*self.right*self.constant)))
        while True:
            try:
                self.sendSpeedToMotor(int(self.left*self.left*self.constant),int(self.right*self.right*self.constant))
                break
            except IOError as e:
                rospy.logwarn(e)
                pass
            
        self.ticks_since_target += 1

if __name__ == "__main__":
    try:
        controller = ArduinoMotor()
        controller.begin()
    except rospy.ROSInterruptException:
        pass
