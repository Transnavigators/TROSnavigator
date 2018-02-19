#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
import binascii
import os
from struct import pack,unpack
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
        # if os.path.exists(port_name):
        #    os.remove(port_name)
        # if os.path.exists('/tmp/ttyTST0'):
        #    os.remove('/tmp/ttyTST0')

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    # test stopping
    def test_stop(self):
        # serial params
        baud_rate = 115200
        port_name = '/tmp/ttyTST1'

        # Kill any open instances of socat
        subprocess.call(['pkill','socat'])
        proc = subprocess.Popen(['/usr/bin/socat', '-d', '-d', 'pty,link=/tmp/ttyTST0', 'pty,link=/tmp/ttyTST1'])
        # Wait 10s for port to come up

        rospy.sleep(4)
        self.ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=1, rtscts=True, dsrdtr=True)

        # create Twist
        vel_msg = Twist()

        # set speed
        vel_msg.linear = Vector3(0, 0, 0)
        vel_msg.angular = Vector3(0, 0, 0)

        # publish the message
        self.publisher.publish(vel_msg)

        # expected values
        stop_cmd = b'\xEE\x00'
        stop_crc = pack('H', CRCCCITT().calculate(stop_cmd))

        data = self.ser.read()
        if data == b'\xEE':
            data = self.ser.read()
            if data == b'\x00':
                self.received_packet = True
                crc = self.ser.read(2)
                err_str = "CRC Failed Packet CRC: %s Calc CRC: %s" % (binascii.hexlify(crc), binascii.hexlify(stop_crc))
                self.assertEquals(crc, stop_crc, err_str)
            else:
                self.fail("Didn't receive packet instruction")
        else:
            self.fail("Didn't receive packet header")
        proc.terminate()


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoControl)
