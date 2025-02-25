import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

map_file = os.path.join(get_package_share_directory('cartographer_slam'), 'config',"turtlebot_area.yaml")
#nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config',"amcl_config.yaml")

#Update to amcl_config_initialized.yaml to start with the initial position
#nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config',"amcl_config_initialized.yaml")

#Update to amcl_config_global_loc.yaml to start with the initial position
nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config',"amcl_config_global_loc.yaml")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} 
                       ]

            ),
          Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]
            ),
            Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
            )
    ])