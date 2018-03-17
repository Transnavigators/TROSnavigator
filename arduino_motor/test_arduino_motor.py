#!/usr/bin/env python
import unittest
import rospy
import smbus
import struct
from geometry_msgs.msg import Twist, Vector3

package_name = 'arduino_motor'
test_name = 'arduino_motor'
package_name = 'test_arduino_motor'


class TestArduinoMotor(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_motor', anonymous=True)
        # Publish to the cmd_vel topic
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_motor(self):
        bus = smbus.SMBus(0)
        twist = Twist()
        twist.linear = Vector3(2.2, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        self.pub.publish(twist)
        rospy.sleep(1.0)
        address = 0x04
        encoder_cmd = ord('e')
        data = bus.read_i2c_block_data(address, encoder_cmd)
        left, right = struct.unpack('=ff', bytearray(data[0:8]))
        self.assertEqual(left, 2.2, "Left velocity %f != 2.2")
        self.assertEqual(right, 2.2, "Right velocity %f != 2.2")

if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoMotor)
