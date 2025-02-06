#!/usr/bin/env python

import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
import signal

class MoveRoverNode(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('move_rover_node')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Here you have the callback method
        # create a Twist message
        msg = Twist()
        # define the linear x-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.linear.x = 0.5
        # define the angular z-axis velocity of /cmd_vel Topic parameter to 0.5
        msg.angular.z = 0.5
        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)

    def stop_rover(self):
        stopMsg = Twist()
        self.publisher_.publish(stopMsg)
        self.get_logger().info('Stoping the Robot')

            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_publisher = MoveRoverNode()
    
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    def signal_handler(sig, frame):
        simple_publisher.stop_rover()
        simple_publisher.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    

    rclpy.spin(simple_publisher)
    # Explicity destroys the node
    

if __name__ == '__main__':
    main()

"""
ros2 pkg create --build-type ament_python publisher_pkg --dependencies rclpy std_msgs geometry_msgs

Launch File: 
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher_pkg',
            executable='simple_publisher',
            output='screen'),
    ])

Setup File:
import os
from glob import glob
(os.path.join('share', package_name), glob('launch/*.launch.py'))
'simple_publisher = publisher_pkg.simple_publisher:main'

Compile:
colcon build --packages-select publisher_pkg

"""
