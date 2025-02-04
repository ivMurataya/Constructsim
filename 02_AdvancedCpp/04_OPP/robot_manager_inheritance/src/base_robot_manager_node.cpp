#include "robot_manager_inheritance/base_robot_manager.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_manager_using_inheritance");
  ros::NodeHandle nh;
  RobotManagerBase rmb = RobotManagerBase(&nh);
  ros::spin();
}
