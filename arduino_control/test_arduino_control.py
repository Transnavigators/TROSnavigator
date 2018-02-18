#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
import binascii
import os
from geometry_msgs.msg import Twist, Vector3
from PyCRC.CRCCCITT import CRCCCITT

package_name = 'arduino_control'
test_name = 'arduino_control'
package_name = 'test_arduino_control'


class TestArduinoControl(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_controller', anonymous=True)
        # Publish to the cmd_vel topic
        self.publisher = rospy.Publisher("cmd_vel", Twist, queue_size=50)

        # Create a virtual serial port
        cmd = ['/usr/bin/socat', 'pty,link=/tmp/ttyTST0', 'pty,link=/tmp/ttyTST1']
        self.proc = subprocess.Popen(cmd)

        # serial params
        baud_rate = 115200
        port_name = '/tmp/ttyTST1'

        # Wait 10s for
        for i in range(0, 10):
            if os.path.exists(port_name):
                break
            rospy.sleep(1)
        if not os.path.exists(port_name):
            self.fail("Port not found, exiting.")
        else:
            self.ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=0, rtscts=True, dsrdtr=True)

    def tearDown(self):
        self.proc.kill()
        self.ser.close()

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test stopping
    def test_stop(self):

        # create Twist
        vel_msg = Twist()

        # set speed
        vel_msg.linear = Vector3(0, 0, 0)
        vel_msg.angular = Vector3(0, 0, 0)

        # publish the message
        self.publisher.publish(vel_msg)

        rospy.sleep(1.)

        # expected values
        stop_cmd = b'\xEE\x00'
        stop_crc = bytes(CRCCCITT().calculate(bytes(stop_cmd)))
        stop_crc_str = binascii.hexlify(stop_crc).decode('ascii')

        while not rospy.is_shutdown():
            if self.ser.read() == b'\xEE':
                if self.ser.read() == b'\x00':
                    crc = self.ser.read(2)
                    crc_str = binascii.hexlify(crc).decode('ascii')
                    self.assertEquals(crc, stop_crc, "CRC Failed Packet CRC: %s Calc CRC: %s" % (crc_str, stop_crc_str))
                    break


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoControl)
