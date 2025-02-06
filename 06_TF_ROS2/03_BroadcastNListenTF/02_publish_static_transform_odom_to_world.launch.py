#! /usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_tf_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_turtle_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    return LaunchDescription(
        [
            static_tf_pub
        ]
    )

"""
The above launch file defines a Node called static_tf_pub. 
This Node starts the static_transform_publisher executable from the tf2_ros package. 
This node is configured to publish a static transform between the world and odom 
frames using the arguments. The arguments to the node are the translation and rotation 
(rpy) of the transform.
"""
