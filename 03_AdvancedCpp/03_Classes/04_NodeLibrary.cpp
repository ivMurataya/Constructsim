/*


In this part you will modify the class that you created in the previous exercise (Exercise 3.4).

    Edit the package's package.xml file in order to add nav_msgs as a new dependency to your existing package. Then add the corresponding include statement to your code.

    Declare all your current class's member variables as private but keep the method print_specifications() public.

    Write a parametrized constructor that takes a pointer to a ros::NodeHandle object and three strings as arguments. One string must represent a topic name to subscribe to, the other one the robot name, and the third one the robot model.

    Next declare two additional private member variables: odom_subscriber of type ros::Subscriber and a string called odometry_topic as a with default value /odom.

    Using the parameterized constructor, initialize the class variables at the time of instantiating the class. Use this line to initialize the odom_subscriber variable:

odom_subscriber = nh->subscribe(odometry_topic, 1000, &RobotManager::odom_callback, this);

Also define a new class member function odom_callback() that has the following signature:

void RobotManager::odom_callback(const nav_msgs::Odometry::ConstPtr &msg);

Within it, use ROS's logging system to print out the robot name and the value of the x and y positions from the incomming nav_msgs/Odometry message, like so:

 ROS_INFO("%s position (x,y): %lf , %lf", robot_name.c_str(), msg->pose.pose.position.x, msg->pose.pose.position.y);

Once you're done writing both files make sure your CMakeLists.txt file knows how to compile robot_manager.cpp into a library that other programs can link to.



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
    }

void RobotManager::print_specifications() const {
    ROS_INFO("Robot Name: %s", robot_name.c_str());
    ROS_INFO("Robot Model: %s", robot_model.c_str());
    ROS_INFO("Battery Capacity: %.2f kWh", battery_capacity_kwh);
    ROS_INFO("Total Operation Hours: %d", total_operation_hours);
}

// Callback function for odometry messages
void RobotManager::odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    ROS_INFO("Received Odometry Data - Position: [x: %f, y: %f, z: %f]",
             msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z);
}

//----------------------------------------------------------------------------------------------------------------
/*


Inside of robot_manager_node.cpp:

    Create a ROS node handle using ros::NodeHandle nh just after the ros::init line that initializes a node.
    Modify how you create an object of class RobotManager to initialize it using the parameterized constructor. Remember that you have to supply it with the ros node handle address, the topic name /robot1/odom, and a robot name and model as parameters for the constructor to use.
    Keep your call to the member function print_specifications() to have your robot's data printed out.
    Finally add a call to ros::spin() to keep the program running and processing callbacks

Build, source, then run:

rosrun my_robot_manager robot_manager_node

After printing the robot specification once, the odometry callback is calledcalled continuously to print the x,y coordinates to the console:


*/
//----------------------------------------------------------------------------------------------------------------
//src/robot_manager_node.cpp
//#include "robot_manager.h"
#include "../include/my_robot_manager/robot_manager.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "robot_manager");
    
    // Create a NodeHandle (needed for ROS logging)
    ros::NodeHandle nh;

    // Instantiate a RobotManager object
    RobotManager my_robot(&nh, "robot1/odom","RoboX", "TX-200");

    // Print the robot specifications
    my_robot.print_specifications();
    // Keep the node alive to listen to odometry messages
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







