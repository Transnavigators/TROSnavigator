#!/usr/bin/env python
import unittest
import subprocess
import rospy
import serial
from geometry_msgs.msg import Twist, Vector3
from PyCRC.CRC16 import CRC16

package_name = 'arduino_control'
test_name = 'arduino_control'
package_name = 'test_arduino_control'


class TestArduinoControl(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    ## test stopping
    def test_stop(self):
        # Initialize the node
        rospy.init_node('test_arduino_controller', anonymous=True)
        # Publish to the cmd_vel topic
        publisher = rospy.Publisher("cmd_vel", Twist, queue_size=50)
        cmd = ['/usr/bin/socat', '-d', '-d', 'pty,link=/dev/ttyTST0', 'pty,link=/dev/ttyTST1']
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        rospy.sleep(1.)

        # set up serial
        baud_rate = 115200
        port_name = '/dev/ttyTST1'
        ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=0, rtscts=True, dsrdtr=True)

        # create Twist
        vel_msg = Twist()

        # set speed
        vel_msg.linear = Vector3(0, 0, 0)
        vel_msg.angular = Vector3(0, 0, 0)

        # publish the message
        publisher.publish(vel_msg)

        rospy.sleep(1.)

        # expected values
        stop_cmd = b'\xEE\x00'
        stop_crc = CRC16().calculate(bytes(stop_cmd))

        while not rospy.is_shutdown():
            data = ser.read()
            if data == b'\xEE':
                data = ser.read()
                if data == b'\x00':
                    data = ser.read(2)
                    self.assertEquals(data, stop_crc, "STOP FAILED")
                    break
        proc.kill()


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoControl)
