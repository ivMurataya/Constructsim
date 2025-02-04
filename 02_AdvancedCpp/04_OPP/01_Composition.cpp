//-------------------------------------------------------------------------------------------------------------------------------
///home/user/catkin_ws/src/robot_manager_composition/include/robot_manager_composition/robot_manager_composition.h
#ifndef ROBOT_MANAGER_COMPOSITION_H
#define ROBOT_MANAGER_COMPOSITION_H

#include "robot_manager_composition/system_information.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <std_srvs/SetBool.h>


class RobotManagerComposition  {
public:
    //Default constructor
    RobotManagerComposition();
    RobotManagerComposition (ros::NodeHandle *nh, ComputerUnit computer_unit);

private:
    ros::NodeHandle nh;
    ros::ServiceServer hello_service;  // New service server
        
    ComputerUnit computer_unit;

    bool ConfigOutputCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
};

#endif // ROBOT_MANAGER_H
//-------------------------------------------------------------------------------------------------------------------------------
///home/user/catkin_ws/src/robot_manager_composition/src/robot_manager_composition.cpp
#include "../include/robot_manager_composition/robot_manager_composition.h"
#include "robot_manager_composition/system_information.h"
#include "std_srvs/SetBool.h"

// Default constructor
RobotManagerComposition::RobotManagerComposition() {
    // Advertise the "robot_manager_output" service using the internal NodeHandle
    hello_service = nh.advertiseService("robot_manager_output", &RobotManagerComposition::ConfigOutputCallback, this);
}

//Parametrized constructor
RobotManagerComposition ::RobotManagerComposition (ros::NodeHandle* nh, ComputerUnit computer ) {
    // Advertise the "hello_service"    
    computer_unit = computer;
    hello_service = nh->advertiseService("robot_manager_output", &RobotManagerComposition ::ConfigOutputCallback, this);
}


// Service callback function
bool RobotManagerComposition ::ConfigOutputCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    ROS_INFO("Console output %s ",req.data   ? "enable" : "disabled");
    if(req.data){
        computer_unit.print_info();
    }
    res.success = true;
    res.message = "Console output " + std::string(req.data ? "enabled" : "disabled");
    return true;
}


//-------------------------------------------------------------------------------------------------------------------------------
//CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(robot_manager_composition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_manager_composition
#  CATKIN_DEPENDS roscpp std_srvs
#  DEPENDS system_lib
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES robot_manager_composition
  CATKIN_DEPENDS roscpp  std_srvs
  #DEPENDS system_lib
) 

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/robot_manager_composition.cpp
 )


# add the executable
add_executable(robot_manager_composition_node src/robot_manager_composition_node.cpp src/robot_manager_composition.cpp src/system_information.cpp)
target_link_libraries(robot_manager_composition_node ${catkin_LIBRARIES} robot_manager_composition_lib)


# Add standalone executable file
add_executable(system_information_test src/system_information_test_executable.cpp src/system_information.cpp)
target_link_libraries(system_information_test ${catkin_LIBRARIES})


//-------------------------------------------------------------------------------------------------------------------------------

//src/robot_manager_composition_node.cpp
#include "../include/robot_manager_composition/robot_manager_composition.h"
#include "robot_manager_composition/system_information.h"
#include <ros/ros.h>
#include <iostream>
 
using namespace std;

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "robot_manager");
    
    //Use the default Constructor
    //RobotManagerComposition  my_robot;

    // Create a 
    ros::NodeHandle nh;
    ComputerUnit computer_unit1;
    // Instantiate a RobotManager object
    RobotManagerComposition  my_robot(&nh,  computer_unit1);
    


    ros::spin();

    return 0;
}
