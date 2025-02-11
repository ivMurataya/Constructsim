#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist             

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
                                           
rate = rospy.Rate(2)
mov = Twist()

mov.angular.z = 0.5
mov.linear.x = 0.5

# Create a loop that will go until someone stops the program execution
while not rospy.is_shutdown():
  # Publish the message within the 'count' variable
  pub.publish(mov)
  rate.sleep()      

mov.angular.z = 0
mov.linear.x = 0      
pub.publish(mov)               


#rosrun my_publisher_example_pkg simple_topic_publisher_cmd.py 
