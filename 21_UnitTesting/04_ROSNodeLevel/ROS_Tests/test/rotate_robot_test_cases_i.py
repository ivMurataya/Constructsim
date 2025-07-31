#! /usr/bin/env python

from robot_control.rotate_robot_complete import RobotControl
from robot_control.srv import RotateRobot, RotateRobotRequest
import rospy
import rosunit
import unittest
import rostest
from geometry_msgs.msg import Twist
import time

PKG = 'robot_control'
NAME = 'rotate_robot_test_ros_srv'


class TestRobotControl(unittest.TestCase):

    # In Python's unittest, the setUp() method is run once per test method.  
    # Use @classmethod with setUpClass() if you want to only instantiate RobotControl() once for the entire test class  

    @classmethod
    def setUpClass(cls):
        cls.rc = RobotControl()
        cls.success = False
        rospy.loginfo("SetUpClass Done")

    # def setUp(self):
    #     self.rc = RobotControl(start_service = False)
    #     rospy.loginfo("SetUp Done")
    #     self.success = False

    def callback(self, msg):
        print(rospy.get_caller_id(), "Angular Speed: %s" % msg.angular.z)
        self.success = msg.angular.z and msg.angular.z == 1

    def test_publish_cmd_vel(self):
        rospy.loginfo("\nRunning Test Publish cmd ")

        test_sub = rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.rc.cmd.angular.z = 1
        self.rc.publish_once_in_cmd_vel()
        timeout_t = time.time() + 10.0 # 10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success)

    def test_rotate_robot_service(self):
        rospy.loginfo("\nRunning Test Rotate Service ")

        rospy.wait_for_service('rotate_robot')
        s = rospy.ServiceProxy('rotate_robot', RotateRobot)
        # test = [(60,90,'y'), (43,'d','y')]
        test = [
                (60, 90, 'y', True),   # valid
                (43, 43, 'f', False),  # invalid
                (30, 30, 'n', True),   # valid
                (10, 10, 'x', False),  # invalid
            ]
        for x,y,z,expected_success in test:
            print("Requesting %s + %s + %s" % (x,y,z))
            #test both simple and formal call syntax
            resp = s(x,y,z)
            resp2 = s.call(RotateRobotRequest(x,y,z))
            # self.assertEquals(resp.rotation_successfull, resp2.rotation_successfull)
            # self.assertTrue(resp.rotation_successfull, "integration failure, service responce was not True")
            self.assertEqual(resp.rotation_successfull, expected_success, "Failed for input {}".format(z))

    def test_stop_robot(self):
        # Set some dummy motion values before calling stop
        rospy.loginfo("\nRunning Test STOP")
        self.rc.cmd.linear.x = 1.5
        self.rc.cmd.angular.z = 2.5


        self.rc.stop_robot()
        rospy.loginfo("Robot Stopped, Vel angular.z = %f", self.rc.cmd.angular.z)

        self.assertEqual(self.rc.cmd.linear.x, 0.0, "Expected linear.x to be 0.0 after stop_robot")
        self.assertEqual(self.rc.cmd.angular.z, 0.0, "Expected angular.z to be 0.0 after stop_robot")




if __name__ == '__main__':
    rostest.rosrun(PKG,NAME,TestRobotControl)



'''

Description

This Python script contains unit and integration tests for the RobotControl class using unittest and rostest. 
It verifies that the robot correctly publishes to /cmd_vel and that the /rotate_robot service works as expected.

## Test Cases

1. **test_publish_cmd_vel**
    Publishes an angular velocity (angular.z = 1) to /cmd_vel.
    Checks if the message is received successfully.

2. **test_rotate_robot_service**
    Calls the /rotate_robot service with test values (speed, angle, direction).
    Verifies that the service responds with rotation_successfull == True.
    Compares both simple and formal service call formats.

## Usage

**Step 1:** Run the service node in a separate terminal:

bash
rosrun robot_control rotate_robot_complete.py
'''
