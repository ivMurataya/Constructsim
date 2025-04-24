#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import time
from datetime import datetime
import numpy as np

class TurtlebotMoves():
    def __init__(self):
        # Positions
        self.odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,self.odometryCallback)
        # Velocities
        self.vel_topic = '/cmd_vel'
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        # Time
        self.time = 0
        self.ini = 0
        
    def odometryCallback(self,odom_msg):
        position_x = odom_msg.pose.pose.position.x
        print('X position of the robot: ',position_x)
        stamp = odom_msg.header.stamp
        if self.time!=0:
            self.time = stamp.secs + stamp.nsecs * 1e-9
        else:
            self.time = stamp.secs + stamp.nsecs * 1e-9
            self.ini = self.time
        
    def move(self):
        # time
        t = self.time - self.ini
        # velocity
        vel_msg = Twist()
        #####################################################################################################
        # TO DO: insert the desired velocity function
        vel_msg.linear.x = 0.002*t
        #####################################################################################################
        self.vel_pub.publish(vel_msg)

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0.0
        self.vel_pub.publish(vel_msg)

if __name__ == '__main__':
    # init node
    node_name = 'move_velocity'
    rospy.init_node(node_name, anonymous=True)
    # Rate of publishing
    rate = rospy.Rate(2)
    # call Turtlebot class
    mover = TurtlebotMoves()
    
    rospy.loginfo('Initializing robot commands ............................')
    time.sleep(2)

    rospy.on_shutdown(mover.stop)

    while not rospy.is_shutdown():
        mover.move()
        rate.sleep()
