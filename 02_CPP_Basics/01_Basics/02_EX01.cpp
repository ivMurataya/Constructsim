#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move();

  float x_1 = rosbot.get_position(1);
  float y_1 = rosbot.get_position(2);

  ROS_INFO_STREAM(x_1 << " and " << y_1);

  rosbot.move();

  float x_2 = rosbot.get_position(1);
  float y_2 = rosbot.get_position(2);

  ROS_INFO_STREAM(x_2 << " and " << y_2);

  return 0;
}
