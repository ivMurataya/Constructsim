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