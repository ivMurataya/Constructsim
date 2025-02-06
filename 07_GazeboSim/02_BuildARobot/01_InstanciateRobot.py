"""
<!--  Create a basic robot structure -->

<?xml version="1.0"?>

<robot name="robot">
    <!-- Link - chassis -->
    <link name="link_chassis">
        <inertial>
            <mass value="10" />
            <origin xyz="0 0 0.3" rpy="0 0 0" />
            <inertia ixx="1.5417" ixy="0" ixz="0" iyy="3.467" iyz="0" izz="4.742" />
        </inertial>

        <collision>
            <geometry>
                <box size="2 1.3 0.4" />
            </geometry>
        </collision>

        <visual>
            <geometry>
                <box size="2 1.3 0.4" />
            </geometry>
            <material name="Red">
                <color rgba="1 0 0 1" />
            </material>
        </visual>
    </link>
</robot>
"""
#------------------------------------------------------------------------------------
# In ros2_ws/src/robot_description/setup.py
import os
from glob import glob
        (os.path.join('share', package_name,'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
#------------------------------------------------------------------------------------
# Create a launch File 
#ros2_ws/src/robot_description/launch/rviz.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path])}]
    )

    # Load RViz Configuration File #
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(package_directory, "rviz", rviz_config_file)
    print("RViz Config Loaded !")

    # RViz2 Launch Configuration (RViz) #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            robot_state_publisher_node,
            rviz_node,
        ]
    )
