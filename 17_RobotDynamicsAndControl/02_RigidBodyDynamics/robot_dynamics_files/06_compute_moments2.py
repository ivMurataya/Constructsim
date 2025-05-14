"""
Inside this script, add the necessary code for making the cardboard box rotate about the X, Y, and Z axis, 
compute and display the corresponding moments of inertia Ix,Iy,Iz.
"""

"""
a = acceleration
r = torque 
I  = Innertia
a=r/I



w = final angular velocity
a = acceletation 
T = Time
w0 = Initial angular    

w=w0+aT

"""

import rospy
from gazebo_utils.services import (reset_world, set_pose, get_twist, disable_gravity, apply_body_wrench)
from geometry_msgs.msg import (Pose, Point, Wrench, Vector3)

reset_world()
disable_gravity()
torque = 0.1


set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(torque,0,0)), rospy.Duration(1))
rospy.sleep(1)
tw = get_twist('cardboard_box')
Ix = torque / tw.angular.x 


set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(0,torque,0)), rospy.Duration(1))
rospy.sleep(1)
tw = get_twist('cardboard_box')
Iy = torque / tw.angular.y


set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(0,0,torque)), rospy.Duration(1))
rospy.sleep(1)
tw = get_twist('cardboard_box')
Iz = torque / tw.angular.z 

print(Ix,Iy,Iz)

