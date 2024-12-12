from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logs_test',
            executable='log_test_node',
            output='screen',
            emulate_tty=True
            ),
    ])
