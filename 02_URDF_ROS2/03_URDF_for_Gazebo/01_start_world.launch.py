#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    """
    You get the absolute path to the packages stated here. You need this to find the launch 
    files in that package, like the gazebo_ros case. You can also use it to set the path to those Gazebo packages.
    """
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('my_box_bot_gazebo')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    """You get the path where the package my_box_bot_description is installed. 
    You need this to add that path to the Gazebo's model paths. 
    That way, it will find meshes and other useful files for Gazebo."""
    description_package_name = "my_box_bot_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    """
    You get the path to the models used in the world file. 
    Gazebo will find all the models you place inside that model's folder inside the my_box_bot_gazebo package."""
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path


    """
    Gazebo will ONLY work with model-related files, such as meshes and textures if those files are inside 
    its GAZEBO_MODEL_PATH environment variable. The same happens with the files related to plugins but in GAZEBO_PLUGIN_PATH.

    By default, Gazebo always looks inside ~/.gazebo/models. So, the fast and dirty way of making 
    Gazebo find your model files is to copy or soft link your files inside that folder.

    However, you usually want your packages to work in ANY ROS2-enabled system. 
    To this end, it is necessary to add the file paths you want Gazebo to find to the GAZEBO_MODEL_PATH, shown below:
    """
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_box_bot_gazebo, 'worlds', 'box_bot_empty.world'), ''],
          description='SDF world file'),
        gazebo
    ])
