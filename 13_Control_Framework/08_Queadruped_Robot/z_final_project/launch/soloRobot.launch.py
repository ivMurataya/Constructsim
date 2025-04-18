import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

#/home/user/ros2_ws/src/ros2_control_course/common/joint_trajectory_publisher/launch/joint_trajectory_publisher.launch.py
def generate_launch_description():
 
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )
    

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
 
    ])



    