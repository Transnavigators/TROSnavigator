#!/usr/bin/env python
import unittest
import struct
import random
import smbus
import rospy
from geometry_msgs.msg import Twist, Vector3


package_name = 'arduino_odometry'
test_name = 'arduino_odometry'
package_name = 'test_arduino_odometry'


class TestArduinoOdometry(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_odometry', anonymous=True)
        # Publish to the cmd_vel topic
        self.publisher = rospy.Subscriber("cmd_vel", Twist, self.callback)
        self.hasMsg = False

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_odom(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.bus = smbus.SMBus(1)
            except IOError:
                rate.sleep()
        cmd = ord('e')
        encoder1 = random.randint(-1000, 1000)
        encoder2 = random.randint(2000, 3000)
        val = list(bytearray(struct.pack("=ii", encoder1, encoder2)))
        self.bus.write_i2c_block_data(0x05, cmd, list(val))
        rospy.sleep(1.0)
        self.assertTrue(self.hasMsg)

    def callback(self, msg):
        self.hasMsg = True
        print(msg)

if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoOdometry)
