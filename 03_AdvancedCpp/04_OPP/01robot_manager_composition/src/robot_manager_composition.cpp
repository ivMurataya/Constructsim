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
