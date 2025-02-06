import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    urdf_file = 'unicycle.urdf'
    package_description = 'unicycle_robot_pkg'

    urdf = os.path.join(get_package_share_directory(
        package_description), 'urdf', urdf_file)

    xml = open(urdf, 'r').read()

    xml = xml.replace('"', '\\"')

    spawn_args = '{name: \"my_robot\", xml: \"' + xml + '\" }'
    
    spawn_robot =  ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', spawn_args],
            output='screen')

    return LaunchDescription([

     spawn_robot,
    ])


"""


When executing this launch file, it calls generate_launch_description() first and processes the sequence of statements in its function body.

The first few lines are used to read the robot description data into the variable spawn_args.

The ExecuteProcess action called spawn_robot is defined with the corresponding cmd argument. This means this launch file performs a service call to the /spawn_entity service as if you would execute that service call from the command line.

Finally, add spawn_robot to the LaunchDescription list.

That is it. This file allows you to spawn a robot model into a simulation without running the robot_state_publisher node. Spawn it and have a look at the problem.

"""
