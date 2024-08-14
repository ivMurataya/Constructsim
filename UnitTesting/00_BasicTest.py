#!/usr/bin/env python

import rosunit
import unittest
PKG = 'test_demo'
NAME = 'test_demo'

## A sample python unit test
class TestBareBones(unittest.TestCase):

    def test_one_equals_one(self):
        self.assertEquals(1, 1, "1!=1")

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_bare_bones', TestBareBones)

"""
Moreover, if you work for a company or with open source code, many people will be working and modifying the same code. 
With this scenario in mind, it is very easy to introduce errors or bugs, without even noticing it. 
Often, a small and inoffensive change can lead to errors in other parts of the program. And we don't want that, do we?

Currently, there exist hundreds (if not thousands) of ROS packages, and the number keeps growing year to year. 
And guess what? All these package need to be constantly updated, by developers all around the world. 
So with this in mind, unit testing becomes an essential tool, which all developers should know how to apply.

Not convinced yet? Well then, let's have a look at some of the things unit tests can help you with:
    You can make incremental updates to your code more quickly
    You can refactor your code with greater confidence
    It leads to better designed code
    They prevent recurring bugs (bug regressions)
    Other people can work on your code more easily (an automatic form of documentation)
    It is much easier to become a contributor to ROS if we have automated unit tests
    Automatic tests simplify maintainer-ship
"""
