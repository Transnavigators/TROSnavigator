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

    # Test localino server
    def test_localino(self):
        self.sub = rospy.Subscriber('vo', Odometry, self.callback)
        soc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        anchor_names = [9002, 9003, 9005]
        tag_names = [1001, 1002]
        rospy.sleep(1.0)
        num_msgs = 10
        self.numMsgs = 0
        for i in range(0, num_msgs):
            for tag in tag_names:
                for anchor in anchor_names:
                    dist = random.random()*20
                    soc.sendto('%d,%d,%f,%d,3.43V' % (anchor, tag, dist, i), ('127.0.0.1', 10000))
        rospy.sleep(0.1)
        self.assertEquals(self.numMsgs, num_msgs, "Only received %d messages" % self.numMsgs)

    def callback(self, msg):
        self.numMsgs += 1

if __name__ == '__main__':
    import rostest

    rospy.init_node('test_localino_server', anonymous=True)
    rostest.rosrun(package_name, test_name, TestLocalinoServer)
