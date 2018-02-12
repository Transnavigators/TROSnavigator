#!/usr/bin/env python
import sys
import unittest

import rospy
import serial
import math
from geometry_msgs.msg import Twist
from PyCRC.CRC16 import CRC16

package_name = 'arduino_control'
test_name = 'arduino_control'
package_name = 'test_arduino_control'


class TestArduinoControl(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestArduinoControl, self).__init__(*args, **kwargs)
        
        # Initialize the node
        rospy.init_node('test_arduino_controller', anonymous=True)
        # Publish to the cmd_vel topic
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=50)
        

    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")
        
    def test_stop(self):
        # create Twist
        vel_msg = Twist()
    
        # set speed
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        
        # publish the message
        self.publisher.publish(vel_msg)
        
        # expected values
        self.STOP_CMD = b'\xEE\x00'
        self.STOP_CRC = CRC16().calculate(bytes(self.STOP_CMD))
        
        # set up serial
        self.baud_rate = 115200
        self.port_name = '/dev/ttyACM0'
        self.ser = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0)
        
        
        while not rospy.is_shutdown():
            data = self.ser.read()
            if data == b'\xEE':
                data = self.ser.read()
                if data == b'\x00':
                    data = ser.read(2)
                    self.assertEquals(data,self.STOP_CRC,"STOP FAILED")
                    break
                    
                    
if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoControl)