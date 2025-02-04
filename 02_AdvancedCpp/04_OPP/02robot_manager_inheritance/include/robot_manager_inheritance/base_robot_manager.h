#pragma once
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

class RobotManagerBase {

public:
  RobotManagerBase(){};
  RobotManagerBase(ros::NodeHandle *node_handle);

protected:
  ros::NodeHandle *nh;
  void init_config_output_srv();
  bool output_enabled = false;

private:
  ros::ServiceServer config_output_srv;
  bool ConfigOutputCallback(std_srvs::SetBoolRequest &req,
                            std_srvs::SetBoolResponse &response);
};
