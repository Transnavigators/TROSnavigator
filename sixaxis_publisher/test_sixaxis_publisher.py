#!/usr/bin/env python

import unittest
import rospy
import time
from sixaxis_publisher import SixaxisPublisher
from evdev import InputEvent, SynEvent
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3

package_name = 'sixaxis_publisher'
test_name = 'test_sixaxis_publisher'


class TestSixaxisPublisher(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test sixaxis_publisher
    def test_sixaxis(self):
        self.pub = rospy.Publisher('joytest', Vector3, queue_size=10)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback)

        rospy.sleep(4.0)

        # stationary
        self.sendJoystick(True, 128, 128, 0, 0)

        # Ramp up to full reverse
        self.sendJoystick(True, 128, 192, -0.25, 0)
        self.sendJoystick(True, 128, 255, -0.5, 0)

        # Ramp down
        self.sendJoystick(True, 128, 192, -0.25, 0)
        self.sendJoystick(True, 128, 128, 0, 0)

        # Ramp up to full speed
        self.sendJoystick(True, 128, 64, 1.1, 0)
        self.sendJoystick(True, 128, 0, 2.2, 0)

        # Ramp down
        self.sendJoystick(True, 128, 64, 1.1, 0)
        self.sendJoystick(True, 128, 128, 0, 0)

        # Test turn in place right
        self.sendJoystick(True, 192, 128, 0, -0.875)
        self.sendJoystick(True, 255, 128, 0, -1.75)
        self.sendJoystick(True, 192, 128, 0, -0.875)
        self.sendJoystick(True, 128, 128, 0, 0)

        # Test turn in place left
        self.sendJoystick(True, 64, 128, 0, 0.875)
        self.sendJoystick(True, 0, 128, 0, 1.75)
        self.sendJoystick(True, 64, 128, 0, 0.875)
        self.sendJoystick(True, 128, 128, 0, 0)

    def send_event(self, mtype, code, value):
        self.pub.publish(Vector3(float(mtype), float(code), float(value)))

    def sendJoystick(self, recv_msg, x, y, linspeed, angspeed):
        self.num_msg = 0

        self.send_event(3, 4, y)  # Send y axis
        self.send_event(0, 0, 0)  # Syn message
        self.send_event(3, 3, x)  # Send x axis
        self.send_event(0, 0, 0)  # Syn message

        rospy.sleep(0.1)

        # Make sure we received the message and it is moving at correct speed
        if recv_msg:
            self.assertEqual(self.num_msg, 2, "Received %d messages, not 2" % self.num_msg)
        else:
            self.assertEqual(self.num_msg, 0, "Received %d messages, not 0" % self.num_msg)
        if recv_msg:
            self.assertTrue(abs(self.lastMsg.linear.x - linspeed) < 0.01,
                            "Linear speed %f != %f msg=%s" % (self.lastMsg.linear.x, linspeed, str(self.lastMsg)))
            self.assertTrue(abs(self.lastMsg.angular.z - angspeed) < 0.2,
                             "Angular speed %f != %f msg=%s" % (self.lastMsg.angular.z, angspeed, str(self.lastMsg)))

    def callback(self, msg):
        self.num_msg += 1
        self.lastMsg = msg


if __name__ == '__main__':
    import rostest

    rospy.init_node('test_sixaxis_publisher', anonymous=True)
    rostest.rosrun(package_name, test_name, TestSixaxisPublisher)
