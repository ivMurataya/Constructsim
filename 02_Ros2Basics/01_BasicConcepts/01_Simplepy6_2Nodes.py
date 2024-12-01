#!/usr/bin/env python

# Indicate which interpreter should be used to execute the script. 
import rclpy 
import time 
from rclpy.node import Node

class HeartbeatNode(Node):
    def __init__(self, rover_name, timer_period=0.2):
        # call super() in the constructor to initialize the Node object
        # the parameter we pass is the node name
        self._rover_name = rover_name
        super().__init__(self._rover_name)

        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        self.get_logger().info("Hola")
        self.get_logger().info(self._rover_name +" is alive..." + str(ros_time_stamp))



def main(args=None):
   # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="Mars_Rover1", timer_period=1.0)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()


def main_shutdown(args=None):
    # initialize the ROS2 communication
    rclpy.init(args=args)
    # declare the node constructor
    node = HeartbeatNode(rover_name="Mars_Rover_2", timer_period=1.5)
    # keeps the node alive, waits for a request to kill the node (ctrl+c)
    rclpy.spin(node)
    # shutdown the ROS2 communication
    rclpy.shutdown()


if __name__ == '__main__':
    main() #call the main function




"""
To run two nodes at the same time is nessesary to add 2 entry points, referring to the 2 functions we want to launch

 entry_points={
        'console_scripts': [
            'heartbeat_executable = mars_rover_systems.heartbeat:main',
            'heartbeat_executable_shutdown = mars_rover_systems.heartbeat:main_shutdown'
            
        ],

We can run the prgram as: ros2 run PACKAGE heartbeat_executable OR heartbeat_executable_shutdown

"""
