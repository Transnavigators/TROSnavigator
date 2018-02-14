#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
from PyCRC.CRC16 import CRC16
from nav_msgs.msg import Odometry
from struct import pack

package_name = 'arduino_publisher'
test_name = 'arduino_publisher'
package_name = 'test_arduino_publisher'


class TestArduinoPublisher(unittest.TestCase):
    noMsg = True

    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    ## test publisher
    def test_pub(self):
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        cmd = ['/usr/bin/socat', '-d', '-d', 'pty,link=/tmp/ttyTST0', 'pty,link=/tmp/ttyTST1']
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        rospy.sleep(1.)
        ser = serial.Serial(port='/tmp/ttyTST0', baudrate=115200, timeout=0, rtscts=True, dsrdtr=True)
        stop_cmd = b'\xEE\x00'
        stop_crc = CRC16().calculate(bytes(stop_cmd))
        packet = pack('2si', stop_cmd, stop_crc)
        ser.write(packet)
        ser.close()
        proc.kill()
        while self.noMsg:
            rospy.sleep(0.1)
        self.assertFalse(self.noMsg)

    def callback(self, msg):
        self.noMsg = False


if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoPublisher)