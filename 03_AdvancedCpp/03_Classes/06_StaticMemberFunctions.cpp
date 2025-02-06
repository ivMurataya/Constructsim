/*

Part 1

    Add a public static int variable called robot_count to use as a shared counter by all instances of the RobotManager class
    Initialize it to 0
    Increase the counter with every new object the class instantiates

This is just an example, if you pass Turty1 as robot name to the constructor you should get:




catkin_create_pkg my_robot_manager roscpp

*/
//----------------------------------------------------------------------------------------------------------------
//include/my_robot_manager/robot_manager.h
#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H

#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <ros/ros.h>
#include <sstream>
#include <string>
#include "nav_msgs/Odometry.h"

class RobotManager {
public:

    static int robot_count;
    // Constructor
    RobotManager( ros::NodeHandle *nh ,const std::string& topic, const std::string& name, const std::string& model);

    // Function to print robot specifications
    void print_specifications() const;
    

private:
    std::string robot_name;
    std::string robot_model;

    ros::Subscriber odom_subscriber;
    std::string odometry_topic ;

    float battery_capacity_kwh = 10.5;  // Default value
    int total_operation_hours = 0;      // Default value
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif // ROBOT_MANAGER_H


//----------------------------------------------------------------------------------------------------------------
//src/robot_manager.cpp

#include "../include/my_robot_manager/robot_manager.h"
#include "nav_msgs/Odometry.h"

RobotManager::RobotManager(ros::NodeHandle* nh, const std::string& topic,const std::string& name, const std::string& model) {
    
    odometry_topic = topic;
    robot_name = name;
    robot_model = model;
    odom_subscriber = nh->subscribe(topic, 1000, &RobotManager::odom_callback, this);
    robot_count++;
    ROS_INFO("Robot %d created.", robot_count);
    }

void RobotManager::print_specifications() const {
    ROS_INFO("Robot Name: %s", robot_name.c_str());
    ROS_INFO("Robot Model: %s", robot_model.c_str());
    ROS_INFO("Battery Capacity: %.2f kWh", battery_capacity_kwh);
    ROS_INFO("Total Operation Hours: %d", total_operation_hours);
}

// Callback function for odometry messages
void RobotManager::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("%s Data - Position: [x: %f, y: %f, z: %f]",
             robot_name.c_str(),
             msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z);
}
//----------------------------------------------------------------------------------------------------------------
/*


Part 2

    Modify robot_manager_node.cpp and create two different instances of the same RobotManager class but changing the name and odometry topic so that they match the topics that you get from the two simulated robots.
    Hint use rostopic list to find out the name of the name of the second robot odometry topic.

For instance, if you pass Turty1 and Turty2 as robot name to the constructor of each instance you should get:



*/
//----------------------------------------------------------------------------------------------------------------
//src/robot_manager_node.cpp

#include "../include/my_robot_manager/robot_manager.h"
#include <ros/ros.h>
#include <iostream>
 
using namespace std;
int RobotManager::robot_count = 0;

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "robot_manager");
    
    // Create a NodeHandle (needed for ROS logging)
    ros::NodeHandle nh;

    // Instantiate a RobotManager object
    RobotManager my_robot(&nh, "robot1/odom","Turty1", "TX-200");
    RobotManager my_robot2(&nh, "robot2/odom","Turty2", "TX-200");
    

    // Print the robot specifications
    my_robot.print_specifications();
    my_robot2.print_specifications();
    // Keep the node alive to listen to odometry messages
    cout << "Total refill counts: " << my_robot.robot_count << endl;
    ros::spin();

    return 0;
}


//----------------------------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.0.2)
project(my_robot_manager)


find_package(catkin REQUIRED COMPONENTS
  roscpp
nav_msgs
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES my_robot_manager
  CATKIN_DEPENDS roscpp nav_msgs
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

// <depend>nav_msgs</depend> to package.xml
//rosrun my_robot_manager robot_manager_node






