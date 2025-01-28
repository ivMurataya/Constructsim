#Package created with command:
#ros2 pkg create my_package --build-type ament_cmake --dependencies rclcpp
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='simple_node',
            output='screen'),
    ])
