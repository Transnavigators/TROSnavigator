#!/usr/bin/env python
import unittest
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import UInt8MultiArray

package_name = 'arduino_motor'
test_name = 'arduino_motor'
package_name = 'test_arduino_motor'


class TestArduinoMotor(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_arduino_motor', anonymous=True)

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_motor(self):
        self.has_msg = False
        self.sent_msg = False

        # Publish to the cmd_vel topic
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=50)
        self.sub = rospy.Subscriber('motorcmd', UInt8MultiArray, self.callback)
        twist = Twist()
        twist.linear = Vector3(2.2, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        rospy.sleep(2.0)
        self.pub.publish(twist)
        self.sent_msg = True
        rospy.sleep(1.0)
        self.assertTrue(self.has_msg, "Didn't receive a message")

    def callback(self, msg):
        self.has_msg = True
        if self.sent_msg:
            self.assertEqual(msg.data[0:8], [205, 204, 12, 64, 205, 204, 12, 64],
                             "Received [%s]" % ' '.join(str(int(e)) for e in msg.data[0:8]))
        else:
            self.assertEqual(msg.data[0:8], [0, 0, 0, 0, 0, 0, 0, 0],
                             "Didn't receive zeros, received [%s]" % (' '.join(str(int(e)) for e in msg.data[0:8])))


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestArduinoMotor)
