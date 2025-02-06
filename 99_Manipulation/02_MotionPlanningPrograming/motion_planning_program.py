#! /usr/bin/env python
"""
In this section of the code, we are just importing some modules and messages that we'll need for the program. 
The most important one here is the moveit_commander module, which will allow us to communicate with the MoveIt RViz interface.
"""
import sys
import copy
import rospy
import moveit_commander  
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv) # initializing the moveit_commander module
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #ROS node.

robot = moveit_commander.RobotCommander() #RobotCommander object, which is, basically, an interface to our robot.
scene = moveit_commander.PlanningSceneInterface() #PlanningSceneInterface object, which is, basically, an interface to the world that surrounds the robot.   
group = moveit_commander.MoveGroupCommander("arm") #MoveGroupCommander object, which is an interface to the manipulator group of joints that we defined when we created the MoveIt package.This will allow us to interact with this set of joints, which, in this case, is the full arm.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) #Topic Publisher, which will publish into the /move_group/display_planned_path topic. By publishing into this topic, we will be able to visualize the planned motion through the MoveIt RViz interface.

pose_target = geometry_msgs.msg.Pose() #Here, we are creating a Pose object, which is the type of message that we will send as a goal. 
pose_target.orientation.w = 1.0
pose_target.position.x = 0.96
pose_target.position.y = 0
pose_target.position.z = 1.18
group.set_pose_target(pose_target)

plan1 = group.plan()  #Telling the "manipulator" group that we created previously to calculate the plan. If the plan is successfully computed, it will be displayed through MoveIt RViz

rospy.sleep(5)

moveit_commander.roscpp_shutdown()
