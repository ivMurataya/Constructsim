#! /usr/bin/env python

import rospy
# import the class TurtleBotProject
from turtlebot_project import *


if __name__ == '__main__':
    # init node
    node_name = 'move_out_of_the_maze'
    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo('Initializing node {}'.format(node_name))
    rate = rospy.Rate(10)
    # call Turtlebot class
    mover = TurtlebotProject()

    # move the Turtlebot
    mover.move()
    
        