from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='simple_node',
            output='screen'),
    ])
"""
package=Package name: Name of the package that contains the code of the ROS2 program to execute
executable=Name of python executable: Name of the Python executable file that you want to execute, we can add multiple entry points
output=Type of output: Through which channel you will print the output of the program

"""
