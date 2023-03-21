#!/usr/bin/env python

import unittest
import catkin_boost_python_test as cbpt

from nose import SkipTest


class TestTest(unittest.TestCase):
    def test_test(self):
      self.assertEqual(cbpt.test(), 42)


if __name__ == '__main__':
    unittest.main()
