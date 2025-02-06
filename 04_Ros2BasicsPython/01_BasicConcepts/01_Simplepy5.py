#!/usr/bin/env python

import rclpy
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self, rover_name, timer_period=0.2):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        self._rover_name = rover_name
        super().__init__(self._rover_name)
        # create a timer sending two parameters:
        # - the duration between two callbacks (0.2 seconds)
        # - the timer function (timer_callback)
        self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        self.get_logger().info(self._rover_name +" is alive..."+str(ros_time_stamp))

def main(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="mars_rover_1", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
We have to avoid using system time with ROS2 because time is a vital element for all your programs to work synchronizing.
To address this we need to introduce two new concepts:

    Custom Node() creation.
    timer callbacks.

The correct way of working with ROS2 nodes, is to create a custom ROS2 node class that inherits from the parent class Node.


In this case we are creating a class named HeartbeatNode, that inherits from Node.
In the init function we have **two arguments that we can modify as needed:
    rover_name: This will be used to name the node.
    timer_period: We will use it for our timer


Another new concept here is rclpy.spin(node).

The spin() function is used to tell ROS2 to process and manage incoming callbacks within the given ROS2 node.
In this case, it primarily manages the timer_callback.
You can see spin() as a highly efficient INFINITE WHILE LOOP.




"""

