
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
 
            Node(
            package='localization_server',
            executable='initial_pose_node',
            output='screen',
            emulate_tty=True),
    ])