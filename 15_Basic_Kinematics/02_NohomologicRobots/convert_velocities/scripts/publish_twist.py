#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_node')
pub = rospy.Publisher('/twist_vels', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.75 #Move the robot with a linear velocity in the x axis
move.angular.z = 0.9 #Move the with an angular velocity in the z axis

while not rospy.is_shutdown(): 
  pub.publish(move)
  rate.sleep()     


#rosrun my_publisher_example_pkg simple_topic_publisher_cmd.py 