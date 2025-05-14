#!/usr/bin/env python

import math, rospy, numpy as np

from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float32MultiArray

def pose2d(pose):
    x, y = pose.position.x, pose.position.z
    theta = 2 * math.atan2(pose.orientation.y, pose.orientation.w)
    return (x, y, theta)

def twist2d(twist):
    return (twist.linear.x, twist.linear.z, twist.angular.y)

def callback(msg_in):
    global pub
    x2, y2, th2 = pose2d(msg_in.pose[2])
    vx2, vy2, w2 = twist2d(msg_in.twist[2])
    x3, y3, th3 = pose2d(msg_in.pose[3])
    vx3, vy3, w3 = twist2d(msg_in.twist[3])
    xc2 = x2 + 0.5 * math.cos(th2)
    yc2 = y2 + 0.5 * math.sin(th2)
    xc3 = x3 + 0.5 * math.cos(th3)
    yc3 = y3 + 0.5 * math.sin(th3)
    vxc2 = vx2 + w2*(0.5*math.cos(th2))
    vyc2 = vy2 - w2*(0.5*math.sin(th2))
    vxc3 = vx3 + w3*(0.5*math.cos(th3))
    vyc3 = vy3 - w3*(0.5*math.sin(th3))
    m = 1.0
    g = 9.81
    I = 0.084
    pe = m*g*(yc2+yc3)
    ke = m*(vxc2*vxc2+vyc2*vyc2) / 2 + m*(vxc3*vxc3+vyc3*vyc3) / 2 + I*np.array(w2)*np.array(w2) / 2 + I*np.array(w3)*np.array(w3) / 2    
    
    data = [xc2,yc2,th2,vxc2,vyc2,w2,xc3,yc3,th3,vxc3,vyc3,w3,pe,ke]
    msg_out = Float32MultiArray(data=data)
    pub.publish(msg_out)

if __name__ == '__main__':
    rospy.init_node('dynamics_monitor', anonymous=True)
    sub = rospy.Subscriber('/gazebo/link_states', LinkStates, callback)
    pub = rospy.Publisher('/dynamic_data', Float32MultiArray, queue_size=10)
    rospy.spin()
