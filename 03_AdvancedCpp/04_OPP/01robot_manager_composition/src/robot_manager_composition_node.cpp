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