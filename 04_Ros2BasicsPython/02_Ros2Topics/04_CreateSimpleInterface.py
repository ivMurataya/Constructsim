
"""specify this is a CMake package with the --build-type flag set to ament_cmake"""

# ros2 pkg create --build-type ament_cmake custom_interfaces --dependencies rclcpp std_msgs
"""
To create a new interface, you have to follow the next steps:

    Create a directory named msg inside your package
    Inside this directory, create a file named name_of_your_message.msg 
    Modify the CMakeLists.txt file 
    Modifypackage.xml file
    Compile and source
    Use in your node
"""

#------------  RoverEvents.msg ------------
#std_msgs/String info # Info about discovery, issue, action
#geometry_msgs/Pose rover_location # Location of the mars rover when the info was generated


#------------ CMakeLists.txt file ------------------
"""
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverEvents.msg"
   DEPENDENCIES std_msgs geometry_msgs
)

ament_package()

"""

# Add the following lines to the package.xml file:
"""
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
"""

# colcon build --packages-select custom_interfaces
# source install/setup.bash
# ros2 interface show custom_interfaces/msg/Age












