#include "robot_manager_inheritance/mobile_robot_robot_manager.h"
#include "robot_manager_inheritance/base_robot_manager.h"
#include <ros/ros.h>

MobileRobotRobotManager::MobileRobotRobotManager(ros::NodeHandle *node_handle) {
  // assign NodeHandle to nh variable from parent class
  nh = node_handle;
  init_config_output_srv();
  init_odom_subscriber();
}

void MobileRobotRobotManager::init_odom_subscriber() {
  odom_subscriber = nh->subscribe(
      odometry_topic, 1000, &MobileRobotRobotManager::odom_callback, this);
  ROS_INFO("Odometry subscriber created");
}

void MobileRobotRobotManager::odom_callback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  if (output_enabled) {
    current_x_position = msg->pose.pose.position.x;
    current_y_position = msg->pose.pose.position.y;
    ROS_INFO("Position (x,y): %lf , %lf", current_x_position,
             current_y_position);
  }
}
