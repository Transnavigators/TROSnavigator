#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
import os
from PyCRC.CRCCCITT import CRCCCITT
from nav_msgs.msg import Odometry
from struct import pack

package_name = 'arduino_publisher'
test_name = 'arduino_publisher'
package_name = 'test_arduino_publisher'


class TestArduinoPublisher(unittest.TestCase):
    msgList = []

    def setUp(self):
        rospy.init_node('test_arduino_publisher', anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test publisher
    def test_pub(self):
        # serial params
        port_name = '/tmp/ttyTST0'

        # Kill any open instances of socat
        subprocess.call(['pkill', 'socat'])

        # Create virtual port
        proc = subprocess.Popen(['/usr/bin/socat', '-d', '-d', 'pty,link=%s' % port_name, 'pty,link=/tmp/ttyTST1'])
        rospy.sleep(4)

        ser = serial.Serial(port=port_name, baudrate=115200, timeout=1, rtscts=True, dsrdtr=True)

        cmd = pack('2siiI', b'\xEE\x01', 7500, 7500, 3600)
        crc = CRCCCITT().calculate(cmd[0:14])
        packet = pack('14sH', cmd[0:14], crc)

        # Wait for the publisher
        ser.write(packet)

        # Wait for a response
        for i in range(5):
            if len(self.msgList) > 0:
                break
            rospy.sleep(1)
        if len(self.msgList) > 0:
            self.assertTrue(self.msgList[0].pose.pose.position.x > 0, "Didn't move forward")
            self.assertTrue(self.msgList[0].twist.twist.linear.x > 0, "Velocity is incorrect")
            self.assertTrue(self.msgList[0].twist.twist.angular.z < 0.01, "Angle is incorrect")
        else:
            self.fail("Message never sent, exiting.")
        proc.terminate()

    def callback(self, msg):
        self.msgList.append(msg)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoPublisher)