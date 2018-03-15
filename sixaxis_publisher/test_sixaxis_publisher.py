#!/usr/bin/env python

import unittest
import rospy
from evdev import UInput, AbsInfo
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3

package_name = 'sixaxis_publisher'
test_name = 'test_sixaxis_publisher'


class TestSixaxisPublisher(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test sixaxis_publisher
    def test_sixaxis(self):
        self.hasMsg = False
        self.pub = rospy.Publisher("cmd_vel", Twist, self.callback)
        # Generated from device.capabilities()
        cap = {0L: [0L, 1L, 3L, 4L, 21L],
               1L: [304L, 305L, 307L, 308L, 310L, 311L, 312L, 313L, 314L, 315L, 316L, 317L, 318L, 544L, 545L, 546L,
                    547L],
               3L: [
                   (0L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                   (1L, AbsInfo(value=126, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                   (2L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                   (3L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                   (4L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                   (5L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0))
               ],
               4L: [4L], 21L: [80L, 81L, 88L, 89L, 90L, 96L]
               }
        self.ui = UInput(cap, name="PLAYSTATION(R)3 Controller", version=0x1)

        # stationary
        self.sendJoystick(128, 128, 0, 0)

        # Check error detection code with linear impulse
        self.sendJoystick(128, 255, 0, 0)

        # Ramp up to full speed
        self.sendJoystick(128, 192, 1.1, 0)
        self.sendJoystick(128, 255, 2.2, 0)

        # Ramp down
        self.sendJoystick(128, 192, 1.1, 0)
        self.sendJoystick(128, 128, 0, 0)

        # Check error detection code with angular impulse
        self.sendJoystick(255, 128, 0, 0)

        # Test turn in place right
        self.sendJoystick(192, 128, 0, 0.875)
        self.sendJoystick(255, 128, 0, 1.75)
        self.sendJoystick(192, 128, 0, 0.875)
        self.sendJoystick(128, 128, 0, 0)

        # Test turn in place left
        self.sendJoystick(64, 128, 0, -0.875)
        self.sendJoystick(0, 128, 0, -1.75)
        self.sendJoystick(64, 128, 0, -0.875)
        self.sendJoystick(128, 128, 0, 0)

        # Close device so the node doesn't get confused later
        self.ui.close()

    def sendJoystick(self, x, y, linspeed, angspeed):
        self.hasMsg = False
        self.ui.write(3, 4, y)  # Move forward at full speed
        self.ui.write(3, 3, x)  # Don't turn
        self.ui.syn()
        rospy.sleep(0.1)

        # Make sure we received the message and it is moving at correct speed
        self.assertTrue(self.hasMsg)
        self.assertEqual(self.lastMsg.linear.x, linspeed)
        self.assertEqual(self.lastMsg.angular.z, angspeed)
        self.hasMsg = False

    def callback(self, msg):
        self.hasMsg = True
        self.lastMsg = msg


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestSixaxisPublisher)
