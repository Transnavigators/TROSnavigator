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

        # Create virtual port
        cmd = ['/usr/bin/socat', 'pty,link=/tmp/ttyTST0', 'pty,link=/tmp/ttyTST1']
        self.proc = subprocess.Popen(cmd)

        # serial params
        baud_rate = 115200
        port_name = '/tmp/ttyTST0'

        # Wait 10s for
        for i in range(0, 10):
            if os.path.exists(port_name):
                break
            rospy.sleep(1)

        if os.path.exists(port_name):
            self.ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=0, rtscts=True, dsrdtr=True)
        else:
            self.fail("Port not found, exiting.")

        self.sub = rospy.Subscriber("odom", Odometry, self.callback)

    def tearDown(self):
        self.proc.kill()

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test publisher
    def test_pub(self):
        cmd = pack('2siiI', b'\xEE\x01', 7500, 7500, 3600)
        crc = CRCCCITT().calculate(cmd[0:14])
        packet = pack('14sH', cmd[0:14], crc)
        rospy.sleep(1)
        self.ser.write(packet)
        for i in range(10):
            if len(self.msgList) > 0:
                break
            rospy.sleep(0.1)
        if len(self.msgList) > 0:
            self.assertTrue(self.msgList[0].pose.pose.position.x > 0)
            self.assertTrue(self.msgList[0].twist.twist.linear.x > 0)
            self.assertTrue(self.msgList[0].twist.twist.angular.z < 0.01)
        else:
            self.fail("Message never sent, exiting.")

    def callback(self, msg):
        self.msgList.append(msg)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoPublisher)