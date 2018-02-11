#!/usr/bin/env python
import sys
import unittest


package_name = 'arduino_control'
test_name = 'arduino_control'
package_name = 'test_arduino_control'


## A sample python unit test
class TestArduinoControl(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestArduinoControl)