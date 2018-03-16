#!/usr/bin/env python

import unittest
import rospy
import subprocess
from evdev import UInput, AbsInfo, ecodes as e
from geometry_msgs.msg import Quaternion, Point, Twist, Vector3

package_name = 'sixaxis_publisher'
test_name = 'test_sixaxis_publisher'


class TestSixaxisPublisher(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test sixaxis_publisher
    def test_sixaxis(self):
        subprocess.call('modprobe uinput', shell=True)
        self.hasMsg = False
        self.pub = rospy.Publisher("cmd_vel", Twist, self.callback)

        # Generated from device.capabilities()
        # See http://python-evdev.readthedocs.io/en/latest/tutorial.html#injecting-input
        # cap = {0L: [0L, 1L, 3L, 4L, 21L],
        #        1L: [304L, 305L, 307L, 308L, 310L, 311L, 312L, 313L, 314L, 315L, 316L, 317L, 318L, 544L, 545L, 546L,
        #             547L],
        #        3L: [
        #            (0L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #            (1L, AbsInfo(value=126, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #            (2L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #            (3L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #            (4L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #            (5L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0))
        #        ],
        #        4L: [4L], 21L: [80L, 81L, 88L, 89L, 90L, 96L]
        #        }
        cap = {0L: [0L, 1L, 3L, 4L, 21L],
               1L: [304L, 305L, 307L, 308L, 310L, 311L, 312L, 313L, 314L, 315L, 316L, 317L, 318L, 544L, 545L, 546L,
                    547L],
               3L: [(0L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                    (1L, AbsInfo(value=126, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                    (2L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                    (3L, AbsInfo(value=129, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                    (4L, AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
                    (5L, AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0))],
               4L: [4L],
               21L: [80L, 81L, 88L, 89L, 90L, 96L]}

        # cap = {(e.EV_MSC, 4L): [(e.MSC_SCAN, 4L)],
        #        (e.EV_KEY, 1L): [([e.BTN_A, e.BTN_GAMEPAD, e.BTN_SOUTH], 304L), ([e.BTN_B, e.BTN_EAST], 305L),
        #                         ([e.BTN_NORTH, e.BTN_X], 307L), ([e.BTN_WEST, e.BTN_Y], 308L), (e.BTN_TL, 310L),
        #                         (e.BTN_TR, 311L), (e.BTN_TL2, 312L), (e.BTN_TR2, 313L), (e.BTN_SELECT, 314L),
        #                         (e.BTN_START, 315L), (e.BTN_MODE, 316L), (e.BTN_THUMBL, 317L), (e.BTN_THUMBR, 318L),
        #                         (e.BTN_DPAD_UP, 544L), (e.BTN_DPAD_DOWN, 545L), (e.BTN_DPAD_LEFT, 546L),
        #                         (e.BTN_DPAD_RIGHT, 547L)],
        #        (e.EV_ABS, 3L): [((e.ABS_X, 0L), AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #                         ((e.ABS_Y, 1L), AbsInfo(value=126, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #                         ((e.ABS_Z, 2L), AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #                         ((e.ABS_RX, 3L), AbsInfo(value=129, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #                         ((e.ABS_RY, 4L), AbsInfo(value=128, min=0, max=255, fuzz=0, flat=15, resolution=0)),
        #                         ((e.ABS_RZ, 5L), AbsInfo(value=0, min=0, max=255, fuzz=0, flat=15, resolution=0))],
        #        (e.EV_FF, 21L): [([e.FF_EFFECT_MIN, e.FF_RUMBLE], 80L), (e.FF_PERIODIC, 81L),
        #                         ([e.FF_SQUARE, e.FF_WAVEFORM_MIN], 88L), (e.FF_TRIANGLE, 89L), (e.FF_SINE, 90L),
        #                         ([e.FF_GAIN, e.FF_MAX_EFFECTS], 96L)],
        #        (e.EV_SYN, 0L): [(e.SYN_REPORT, 0L), (e.SYN_CONFIG, 1L), (e.SYN_DROPPED, 3L), (e['?'], 4L), (e['?'], 21L)]}

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
