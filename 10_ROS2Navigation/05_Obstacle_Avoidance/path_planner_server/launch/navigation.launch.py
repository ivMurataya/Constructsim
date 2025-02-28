import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
 
map_file = os.path.join(get_package_share_directory('path_planner_server'), 'config',"turtlebot_area.yaml")
nav2_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config',"amcl_config_global_loc.yaml")

planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config',"planner_server.yaml")
controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config',"controller.yaml")
bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config',"bt_navigator.yaml")
recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config',"recovery.yaml")

def generate_launch_description():
    return LaunchDescription([
        # Map Server
        Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename':map_file} 
                    ]
        ),
        #Nav2 AMCL
        Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
        ),
        #Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),
        #Controller
        Node(
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[controller_yaml]
            ),
        #Manager of recovery behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'
        ),
        #Behavior tree navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]
            ),
        #Nav2_lifecycle_manager
          Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]
            ),

    ])