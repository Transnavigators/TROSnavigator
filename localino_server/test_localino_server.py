#!/usr/bin/env python
import sys
import unittest


package_name = 'localino_server'
test_name = 'localino_server'
package_name = 'test_localino_server'


class TestLocalinoServer(unittest.TestCase):
    ## test 1 == 1
    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(package_name, test_name, TestLocalinoServer)