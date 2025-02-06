#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("arm")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()

"""
# You can get the reference frame for a certain group by executing this line:
print ("Reference frame: %s" % group.get_planning_frame())

#You can get the end-effector link for a certain group by executing this line:
print ("End effector: %s" % group.get_end_effector_link())

#You can get a list with all of the groups of the robot, like this:
print ("Robot Groups:")
print (robot.get_group_names())

#You can also get the current Pose of the end-effector of the robot, like this:
print ("Current Pose:")
print (group.get_current_pose())

#Finally, you can check the general status of the robot, like this:
print ("Robot State:")
print (robot.get_current_state())
"""
#You can get the current values of the joints, like this:
print ("Current Joint Values:")
print (group.get_current_joint_values())

group_variable_values[0] = 0.1498
group_variable_values[1] = 0.45029
group_variable_values[2] = 0.4589
group_variable_values[3] = -0.04988
group_variable_values[4] = 0.726
group_variable_values[5] = 0.74997
group_variable_values[6] = 0.5558

group.set_joint_value_target(group_variable_values)
plan2 = group.plan()
rospy.sleep(5)
# It's very simple. In order to execute a trajectory, you just need to call the go() function from the planning group. Like this:
group.go(wait=True)


group_variable_values = [-0.742669, -0.08320, 2.261977, -0.7403532, -1.06993, -0.715316, -0.977430]
group.set_joint_value_target(group_variable_values)
plan2 = group.plan()
rospy.sleep(5)
group.go(wait=True)

moveit_commander.roscpp_shutdown()
