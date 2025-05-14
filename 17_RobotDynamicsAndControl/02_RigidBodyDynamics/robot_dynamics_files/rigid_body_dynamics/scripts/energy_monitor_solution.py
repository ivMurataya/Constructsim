#!/usr/bin/env python

import math, rospy, numpy as np

from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float32

rospy.init_node('energy_monitor', anonymous=True)

def pose2d(pose):
    x = pose.position.x
    y = pose.position.z
    theta = 2 * math.atan2(pose.orientation.y, pose.orientation.w)
    return x, y, theta

def twist2d(twist):
    vx = twist.linear.x
    vy = twist.linear.z
    w = twist.angular.y
    return vx, vy, w

def callback(msg):
    global pub_kinetic
    global pub_potential
    x2, y2, th2 = pose2d(msg.pose[2])
    vx2, vy2, w2 = twist2d(msg.twist[2])
    x3, y3, th3 = pose2d(msg.pose[3])
    vx3, vy3, w3 = twist2d(msg.twist[3])
    xc2 = x2 + 0.5 * math.sin(th2)
    yc2 = y2 + 0.5 * math.cos(th2)
    xc3 = x3 + 0.5 * math.sin(th3)
    yc3 = y3 + 0.5 * math.cos(th3)
    vxc2 = vx2 - w2*(yc2-y2)
    vyc2 = vy2 - w2*(xc2-x2)
    vxc3 = vx3 - w3*(yc3-y3)
    vyc3 = vy3 - w3*(xc3-x3)
    m = 1.0
    g = 9.81
    I = 0.084
    potential_energy = m*g*(yc2+yc3)
    kinetic_energy = m*(vxc2*vxc2+vyc2*vyc2) / 2 + m*(vxc3*vxc3+vyc3*vyc3) / 2 + I*np.array(w2)*np.array(w2) / 2 + I*np.array(w3)*np.array(w3) / 2    
    
    ke_msg = Float32(kinetic_energy)
    pe_msg = Float32(potential_energy)
    
    pub_kinetic.publish(ke_msg)
    pub_potential.publish(pe_msg)

sub = rospy.Subscriber('/gazebo/link_states', LinkStates, callback)

pub_kinetic = rospy.Publisher('/kinetic_energy', Float32, queue_size=10)
pub_potential = rospy.Publisher('/potential_energy', Float32, queue_size=10)
rospy.spin()