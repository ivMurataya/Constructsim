#! /usr/bin/env python
from robot_control.rotate_robot import RobotControl
import unittest

class CaseA(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        speed, angle = self.rc.convert_degree_to_rad(60,90)
        self.assertEqual(angle, 1.57 , "1.57!=1.57")
        self.rc.shutdownhook()

class CaseB(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        speed, angle = self.rc.convert_degree_to_rad(60,-90)
        self.assertEqual(angle, 1.57 , "1.57!=1.57")
        self.rc.shutdownhook()

class CaseC(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        speed, angle = self.rc.convert_degree_to_rad(60,360)
        self.assertEqual(angle, 6.28 , "0!=0")
        self.rc.shutdownhook()


class CaseD(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        # Test 1: User enters a non-numeric string
        with self.assertRaises((TypeError, ValueError)):
            self.rc.convert_degree_to_rad(60, "ninety")
        self.rc.shutdownhook()

class CaseE(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):

        # Test 2: User enters an empty string
        with self.assertRaises((TypeError, ValueError)):
            self.rc.convert_degree_to_rad(60, "")

        self.rc.shutdownhook()

class CaseF(unittest.TestCase):
    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        # Test 3: User enters None (missing value)
        with self.assertRaises((TypeError, ValueError)):
            self.rc.convert_degree_to_rad(60, None)

        self.rc.shutdownhook()



class MyTestSuite(unittest.TestSuite):
    def __init__ (self):
        super(MyTestSuite,self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())
        self.addTest(CaseC())
        self.addTest(CaseD())
        self.addTest(CaseE())
        self.addTest(CaseF())
        


