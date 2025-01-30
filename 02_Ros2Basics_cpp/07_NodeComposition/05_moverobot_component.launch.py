import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    #First, start a component container, named my_container:
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            #Second, load the my_components::MoveRobot component into the container
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::MoveRobot',
                    name='moverobot'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
