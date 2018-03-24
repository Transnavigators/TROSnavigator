#!/usr/bin/env python
import unittest
import rospy

test_name = 'master'
package_name = 'test_master'


class TestMaster(unittest.TestCase):
    def setUp(self):
        # Initialize the node
        rospy.init_node('test_master', anonymous=True)

    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")


if __name__ == '__main__':
    import rostest

    rostest.rosrun(package_name, test_name, TestMaster)
