import rospy
from gazebo_utils.services import (reset_world, set_pose, get_twist, disable_gravity, apply_body_wrench)
from geometry_msgs.msg import (Pose, Point, Wrench, Vector3)

reset_world()
disable_gravity()

set_pose('cardboard_box', Pose(position=Point(0,0,1.5)))
apply_body_wrench('cardboard_box', Wrench(torque=Vector3(0,0,0.1)), rospy.Duration(1))
rospy.sleep(1)
print(get_twist('cardboard_box'))
