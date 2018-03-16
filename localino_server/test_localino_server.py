#!/usr/bin/env python
import unittest
import socket
import random
import rospy
from nav_msgs.msg import Odometry

test_name = 'localino_server'
package_name = 'test_localino_server'


class TestLocalinoServer(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_localino(self):
        self.sub = rospy.Subscriber('vo', Odometry, self.callback)
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        anchor_names = [9002, 9003, 9005]
        tag_names = [1001, 1002]
        rospy.sleep(2.0)
        for i in range(1, 20):
            self.hasMsg = False
            for anchor in anchor_names:
                for tag in tag_names:
                    dist = random.random()*20
                    packet_data = '%d,%d,%f,%d,3.43V' % (anchor, tag, dist, i)
                    soc.sendto(packet_data, ('127.0.0.1', 10000))
                    rospy.sleep(0.1)
            self.assertTrue(self.hasMsg, "Failed to receive ROS message")

    def callback(self, msg):
        self.hasMsg = True

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestLocalinoServer)
