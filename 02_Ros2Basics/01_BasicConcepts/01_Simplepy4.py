#!/usr/bin/env python

# Indicate which interpreter should be used to execute the script. 
import rclpy 
import time 
from rclpy.node import Node

# We define a method named main, which is standard in Python.
def main (args=None):
    # initialize the ROS communication
    rclpy.init(args=args)

    # Create a Node
    node = Node('mars_rover_1')

    i = 0 
    max_i = 50
    while i < max_i:
        i += 1 
        ros_time_stamp = node.get_clock().now()
        # print a message to the terminal
        node.get_logger().info(str(i)+": Mars rover 1 is alive ... " + str(ros_time_stamp))
        time.sleep(1)
    #shutdown the ROS comunication
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function


"""
    Here are two main new elements to discuss:
    node = Node('mars_rover_1'): Here we initialize a ROS2 node with the name mars_rover_1. 
    We can give it any name we want.

    ros_time_stamp = node.get_clock().now(): Instead of using the time.time() which retrieves the real time system clock time, 
    the node.get_clock().now() gets the time used by ALL ROS2 systems, which don't necessarily be the same as the system clock. 
    
    For example with simulations we can have a different clock from the real world. 
    So from now on, time has to be retrieve in the ROS2 way.
    
    Also, in ROS2 the recommended way to print messages are LOGS. 
    So from now on use the node.get_logger().info() instead of print().




"""
