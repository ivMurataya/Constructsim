#include "robot_manager_inheritance/mobile_robot_robot_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_manager");
  ros::NodeHandle nh;
  MobileRobotRobotManager mrrm = MobileRobotRobotManager(&nh);
  ros::spin();
}
