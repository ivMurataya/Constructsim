import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    
    # Trace
    trace = Trace(
        session_name='callback_function'
    )
    # Nodes
    callback_function_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='callback_function.py',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        trace,
        callback_function_node
    ])
    
"""
    NOTE that you are adding a strange trace ClassNode launch.
    This will allow you to see the callback sequence in time of the node executed in the script callback_function.py.
    All the info will be stored in a tracing folder named: ~/.ros/tracing/callback_function.


"""
