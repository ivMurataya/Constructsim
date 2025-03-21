from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='get_object_pose_action_client',
            executable='simple_node',
            output='screen'),
    ])