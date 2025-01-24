#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  
  rosbot.move_forward(5.0);

  float x_limit = 10.0;
  float x = rosbot.get_position(1);
  ROS_INFO_STREAM("X reached: " << x);
 
  while (x < x_limit) {
      rosbot.move_forward(1.0);
      x = rosbot.get_position(1);

  }
  rosbot.stop_moving();

  return 0;
}
