#!/usr/bin/env python
import sys
import unittest


package_name = 'arduino_publisher'
test_name = 'arduino_publisher'
package_name = 'test_arduino_publisher'


class TestArduinoPublisher(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoPublisher)