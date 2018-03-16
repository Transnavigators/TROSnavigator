#!/usr/bin/env python
import unittest
import subprocess
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
        subprocess.call('i2c-stub-from-dump 0x33 berry_iu.dump', shell=True)
        subprocess.call('i2c-stub-from-dump 0x55 berry_mag.dump', shell=True)
        rospy.sleep(1.0)
        self.assertTrue(self.hasMsg)

    def callback(self, msg):
        self.hasMsg = True

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestBerryimuPublisher)