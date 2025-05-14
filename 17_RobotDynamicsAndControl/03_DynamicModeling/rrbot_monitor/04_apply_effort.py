#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import ApplyJointEffort

try:
    apply_joint_effort_srv = rospy.ServiceProxy("/gazebo/apply_joint_effort", ApplyJointEffort)
    apply_joint_effort_srv(joint_name='joint1', effort=10.0, duration=rospy.Duration(30))
except rospy.ServiceException as e:
    print ("/gazebo/apply_joint_effort call failed: %s"%e)
