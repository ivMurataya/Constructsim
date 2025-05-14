#!/usr/bin/env python

import rospy, sys
from gazebo_utils.services import (reset_world, set_pose, get_pose)

n = int(sys.argv[1])

reset_world()

for _ in range(n):
    p = get_pose('cardboard_box')
    p.position.x -= 0.01
    set_pose('cardboard_box', p)
    rospy.sleep(0.01)
