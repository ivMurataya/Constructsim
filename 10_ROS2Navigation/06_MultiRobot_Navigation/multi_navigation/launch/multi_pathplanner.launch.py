import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
 

tb1_planner_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb1_planner_server.yaml")
tb1_controller_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb1_controller.yaml")
tb1_bt_navigator_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb1_bt_navigator.yaml")
tb1_recovery_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb1_recovery.yaml")

tb0_planner_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb0_planner_server.yaml")
tb0_controller_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb0_controller.yaml")
tb0_bt_navigator_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb0_bt_navigator.yaml")
tb0_recovery_yaml = os.path.join(get_package_share_directory('multi_navigation'), 'config',"tb0_recovery.yaml")

def generate_launch_description():
    return LaunchDescription([

            #Planner
        Node(
            namespace='tb3_0',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb0_planner_yaml]
        ),
        #Controller
        Node(
            namespace='tb3_0',
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[tb0_controller_yaml]
            ),
        #Manager of recovery behaviors
        Node(
            namespace='tb3_0',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[tb0_recovery_yaml],
            output='screen'
        ),
        #Behavior tree navigator
        Node(
            namespace='tb3_0',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb0_bt_navigator_yaml]
            ),


        #Planner
        Node(
            namespace='tb3_1',
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[tb1_planner_yaml]
        ),
        #Controller
        Node(
            namespace='tb3_1',
            name='controller_server',
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[tb1_controller_yaml]
            ),
        #Manager of recovery behaviors
        Node(
            namespace='tb3_1',
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[tb1_recovery_yaml],
            output='screen'
        ),
        #Behavior tree navigator
        Node(
            namespace='tb3_1',
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[tb1_bt_navigator_yaml]
            ),
        #Nav2_lifecycle_manager
          Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': [
                                        'tb3_0/planner_server',
                                        'tb3_0/controller_server',
                                        'tb3_0/recoveries_server',
                                        'tb3_0/bt_navigator',
                                        'tb3_1/planner_server',
                                        'tb3_1/controller_server',
                                        'tb3_1/recoveries_server',
                                        'tb3_1/bt_navigator']}]
            ),

    ])