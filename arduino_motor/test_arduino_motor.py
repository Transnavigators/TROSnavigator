#!/usr/bin/env python
import unittest
import rospy
from geometry_msgs.msg import Twist, Vector3

package_name = 'arduino_motor'
test_name = 'arduino_motor'
package_name = 'test_arduino_motor'


class TestArduinoMotor(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_motor', anonymous=True)
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



if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoMotor)
