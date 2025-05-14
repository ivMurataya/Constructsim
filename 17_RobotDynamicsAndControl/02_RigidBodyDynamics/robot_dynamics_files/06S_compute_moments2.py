#! /usr/bin/env python

import rospy
from gazebo_utils.services import (reset_world, set_pose, get_twist, disable_gravity, apply_body_wrench)
from geometry_msgs.msg import (Pose, Point, Wrench, Vector3)

reset_world()
disable_gravity()

torque = 0.1
set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(torque,0,0)), rospy.Duration(1))
rospy.sleep(1)
twist = get_twist('cardboard_box')
w = twist.angular.x
print('Ix = %.8f' % (torque / w))

set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(0,torque,0)), rospy.Duration(1))
rospy.sleep(1)
twist = get_twist('cardboard_box')
w = twist.angular.y
print('Iy = %.8f' % (torque / w))

set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(0,0,torque)), rospy.Duration(1))
rospy.sleep(1)
twist = get_twist('cardboard_box')
w = twist.angular.z
print('Iz = %.8f' % (torque / w))
