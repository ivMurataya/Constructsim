/*
In this exercise you will modify the class that you created in the previous exercises. Now, instead of printing the [X,Y] coordinates of the robot in the odom_callback() callback, we will create a new service, which will perform this task when called.

Go ahead and pull up your code editor and open your previous files to continue coding. Specifically, navigate inside the my_robot_manager package directory and open robot_manager.h, robot_manager.cpp and robot_manager_node.cpp.

Tasks:

    Firstly, you should declare two new private member variables current_x_position and current_y_position of type float.
    Then access the position coordinates data received in the odometry callback and assign it to the new variables current_x_position and current_y_position.
    Modify the function body of odom_callback() and remove the line that logs ththe current robot position when the callback gets executed: 


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
#include <std_srvs/Trigger.h>


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
    ros::ServiceServer hello_service;  // New service server
    std::string odometry_topic ;

    float battery_capacity_kwh = 10.5;  // Default value
    int total_operation_hours = 0;      // Default value

    float current_x_position = 0.0;
    float current_y_position = 0.0;   
    
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    bool hello_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
};

#endif // ROBOT_MANAGER_H


//----------------------------------------------------------------------------------------------------------------
//src/robot_manager.cpp

#include "../include/my_robot_manager/robot_manager.h"
#include "nav_msgs/Odometry.h"
//#include <std_srvs/Trigger.h>

RobotManager::RobotManager(ros::NodeHandle* nh, const std::string& topic,const std::string& name, const std::string& model) {
    
    odometry_topic = topic;
    robot_name = name;
    robot_model = model;
    odom_subscriber = nh->subscribe(topic, 1000, &RobotManager::odom_callback, this);

    // Advertise the "hello_service"
    std::string service_name = robot_name + "/log_current_position";
    hello_service = nh->advertiseService(service_name, &RobotManager::hello_callback, this);


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
    this->current_x_position = msg->pose.pose.position.x;
    this->current_y_position = msg->pose.pose.position.y;
   /* ROS_INFO("%s Data - Position: [x: %f, y: %f, z: %f]",
             robot_name.c_str(),
             msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z);*/

}

// Service callback function
bool RobotManager::hello_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    ROS_INFO("%s Says Hello, Odom Data X: %f   Data Y: %f" , robot_name.c_str(), this->current_x_position,this->current_y_position);
    res.success = true;
    res.message = "Service called successfully!";
    return true;
}

//----------------------------------------------------------------------------------------------------------------
/*
Part 2
Create a 'Trigger service' that accepts a Trigger request and returns a TriggerResponse response. The name of the service should be the name of the robot + /log_current_position. The moment that a service call is received, the trigger callback method must log the current robot coordinates depending on the new private member variables current_x_position and current_y_position values. 
Hint: include the <std_srvs/Trigger.h> library in your code and add as package dependency std_srvs

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
  std_srvs
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES my_robot_manager
  CATKIN_DEPENDS roscpp nav_msgs std_srvs
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

// <depend>nav_msgs</depend> <depend>std_srvs</depend> to package.xml
//rosrun my_robot_manager robot_manager_node 
//rosservice call /Turty1/log_current_position "{}"






