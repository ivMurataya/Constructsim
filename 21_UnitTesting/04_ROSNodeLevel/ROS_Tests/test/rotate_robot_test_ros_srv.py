#! /usr/bin/env python

from robot_control.rotate_robot_srv import RobotControl
from robot_control.srv import RotateRobot, RotateRobotRequest
import rospy
import rosunit
import unittest
import rostest

PKG = 'robot_control'
NAME = 'rotate_robot_test_ros_srv'


class TestRobotControl(unittest.TestCase):
    def test_rotate_robot_service(self):
        rospy.wait_for_service('rotate_robot')
        s = rospy.ServiceProxy('rotate_robot', RotateRobot)
        test = [(60,90,'y')]
        for x,y,z in test:
            print("Requesting %s + %s + %s" % (x,y,z))
            #test both simple and formal call syntax
            resp = s(x,y,z)
            resp2 = s.call(RotateRobotRequest(x,y,z))
            self.assertEquals(resp.rotation_successfull, resp2.rotation_successfull)
            self.assertTrue(resp.rotation_successfull, "integration failure, service responce was not True")

 
    


if __name__ == '__main__':
    rostest.rosrun(PKG,NAME,TestRobotControl)