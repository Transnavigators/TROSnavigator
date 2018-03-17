#!/usr/bin/env python
import unittest
import os
import subprocess
import rospy
from nav_msgs.msg import Odometry


package_name = 'arduino_odometry'
test_name = 'arduino_odometry'
package_name = 'test_arduino_odometry'


class TestArduinoOdometry(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_odometry', anonymous=True)
        # Publish to the cmd_vel topic
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.hasMsg = False

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_odom(self):
        root_dir = os.path.dirname(os.path.abspath(__file__))
        subprocess.call('i2c-stub-from-dump 0x05 %s/arduino.dump' % root_dir, shell=True)
        rospy.sleep(2.0)
        self.assertTrue(self.hasMsg)

    def callback(self, msg):
        self.hasMsg = True


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoOdometry)
