/*


Tasks Part I:

In robot_manager.h and robot_manager.cpp:

    Create a class called "RobotManager".
    Create four member variables:
        robot_name and robot_model of type std::string
        battery_capacity_kwh of type float
        total_operation_hours of type int
    Write a member function print_specifications() which will log the class attributes to the terminal using a LOG_INFO() macro for which you will have to include the <ros/ros.h> header .
    Set the initial values for battery_capacity_kwh and total_operation_hours at variable initialization.


catkin_create_pkg my_robot_manager roscpp

*/
//----------------------------------------------------------------------------------------------------------------
//include/my_robot_manager/robot_manager.h
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include <ros/ros.h>
#include <string>

class RobotManager {
public:
    // Constructor
    RobotManager(const std::string& name, const std::string& model);

    // Function to print robot specifications
    void print_specifications() const;

private:
    std::string robot_name;
    std::string robot_model;
    float battery_capacity_kwh = 10.5;  // Default value
    int total_operation_hours = 0;      // Default value
};

#endif // ROBOT_MANAGER_H

//----------------------------------------------------------------------------------------------------------------
//src/robot_manager.cpp

#include "../include/my_robot_manager/robot_manager.h"

RobotManager::RobotManager(const std::string& name, const std::string& model)
    : robot_name(name), robot_model(model) {}

void RobotManager::print_specifications() const {
    ROS_INFO("Robot Name: %s", robot_name.c_str());
    ROS_INFO("Robot Model: %s", robot_model.c_str());
    ROS_INFO("Battery Capacity: %.2f kWh", battery_capacity_kwh);
    ROS_INFO("Total Operation Hours: %d", total_operation_hours);
}

//----------------------------------------------------------------------------------------------------------------
/*
Now that the library has been generated, write a main() function in a separate source code file that you can call robot_manager_node.cpp. This is to keep your code clean an organized.

In robot_manager_node.cpp:

    Import your robot_manager library.
    Create a standard C++ main function
    Initialize a ROS node named "robot_manager" by calling ros::init(argc, argv, "robot_manager");
    Instantiate one RobotManager object and afterwards assign a value to robot_name and robot_model.
    Add a call to member function print_specifications().

*/
//----------------------------------------------------------------------------------------------------------------
//src/robot_manager_node.cpp
#include "../include/my_robot_manager/robot_manager.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "robot_manager");
    
    // Create a NodeHandle (needed for ROS logging)
    ros::NodeHandle nh;

    // Instantiate a RobotManager object
    RobotManager my_robot("RoboX", "TX-200");

    // Print the robot specifications
    my_robot.print_specifications();

    return 0;
}

//----------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.0.2)
project(my_robot_manager)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES my_robot_manager
  CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/robot_manager.cpp
 )
# add the executable
add_executable(robot_manager_node src/robot_manager_node.cpp src/robot_manager.cpp)
target_link_libraries(robot_manager_node ${catkin_LIBRARIES} my_robot_manager_lib)
//----------------------------------------------------------------------------------------------------------------
//rosrun my_robot_manager robot_manager_node








