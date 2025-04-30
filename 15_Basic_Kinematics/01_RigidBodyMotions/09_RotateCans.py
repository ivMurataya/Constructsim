#! /usr/bin/env python

import math, rospy
from tf.transformations import quaternion_about_axis, quaternion_multiply
from utilities import set_model_state
from geometry_msgs.msg import Pose, Point, Quaternion

for angle in range(0,360,30):
    can = angle/30
    theta = math.radians(angle)
    xp = 0.35 * math.cos(theta) + 1
    yp = 0.35 * math.sin(theta)
    model_name = 'coke_can_' + str(can)
    position = Point(xp,yp,1.05)
    q_z = quaternion_about_axis(theta, (0,0,1))
    q_y = quaternion_about_axis(math.radians(90), (0,1,0))
    q_zy = quaternion_multiply(q_z, q_y)
    orientation = Quaternion(*q_zy)
    set_model_state(model_name, Pose(position, Quaternion(*q_zy)))
    rospy.sleep(0.1)
