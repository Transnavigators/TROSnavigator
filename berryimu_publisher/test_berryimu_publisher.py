#!/usr/bin/env python
import unittest


package_name = 'berryimu_publisher'
test_name = 'berryimu_publisher'
package_name = 'test_berryimu_publisher'


class TestBerryimuPublisher(unittest.TestCase):
    # test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestBerryimuPublisher)