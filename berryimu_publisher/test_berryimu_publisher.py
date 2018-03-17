#!/usr/bin/env python
import unittest
import subprocess
import os
import rospy
from sensor_msgs.msg import Imu

package_name = 'berryimu_publisher'
test_name = 'berryimu_publisher'
package_name = 'test_berryimu_publisher'


class TestBerryimuPublisher(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

    def test_berryimu(self):
        self.hasMsg = False
        self.sub = rospy.Subscriber("imu_data", Imu, self.callback)
        root_dir = os.path.dirname(os.path.abspath(__file__))

        subprocess.call('i2c-stub-from-dump 0x33,0x44,0x55 %s/berry_mag.dump %s/berry_iu.dump %s/berry_iu.dump' % (root_dir, root_dir, root_dir), shell=True)
        rospy.sleep(2)
        self.assertTrue(self.hasMsg)

    def callback(self, msg):
        self.hasMsg = True

if __name__ == '__main__':
    import rostest

    rospy.init_node('test_berryimu_publisher', anonymous=True)
    rostest.rosrun(package_name, test_name, TestBerryimuPublisher)