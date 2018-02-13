#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
from PyCRC.CRC16 import CRC16

package_name = 'arduino_publisher'
test_name = 'arduino_publisher'
package_name = 'test_arduino_publisher'


class TestArduinoPublisher(unittest.TestCase):
    def __init__(self):
        cmd = ['/usr/bin/socat', '-d', '-d', 'pty,link=/dev/ttyTST0', 'pty,link=/dev/ttyTST1']
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    ## test publisher
    def test_pub(self):
        rospy.sleep(1.)
        ser = serial.Serial(port='/dev/ttyTST0', baudrate=self.baud_rate, timeout=0, rtscts=True, dsrdtr=True)
        stop_cmd = b'\xEE\x00'
        stop_crc = CRC16().calculate(bytes(self.STOP_CMD))
        ser.write(stop_cmd)
        ser.write(stop_crc)
        ser.stop()

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoPublisher)