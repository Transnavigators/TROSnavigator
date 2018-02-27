#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
import binascii
import os
from struct import pack,unpack
from geometry_msgs.msg import Twist, Vector3
from PyCRC.CRCCCITT import CRCCCITT

package_name = 'arduino_odometry'
test_name = 'arduino_odometry'
package_name = 'test_arduino_odometry'


class TestArduinoOdometry(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_odometry', anonymous=True)
        # Publish to the cmd_vel topic
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=50)


    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoOdometry)
