#! /usr/bin/env python

import rospy
from robot_control.srv import RotateRobot, RotateRobotRequest
import sys

rospy.init_node('rotate_robot_client')
rospy.wait_for_service('/rotate_robot')
rotate_robot_service = rospy.ServiceProxy('/rotate_robot', RotateRobot)
srv_msg = RotateRobotRequest()
srv_msg.speed_d = 60
srv_msg.angle_d = 90
srv_msg.clockwise_yn = 'y'
result = rotate_robot_service(srv_msg)
