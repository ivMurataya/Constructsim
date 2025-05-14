import rospy
from gazebo_utils.services import (set_pose, get_pose)
p = get_pose('cardboard_box')
p.position.x -= 0.05
set_pose('cardboard_box', p)
