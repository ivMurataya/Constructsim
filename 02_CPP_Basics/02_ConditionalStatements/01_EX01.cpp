#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  
  rosbot.move_forward(5.0);

  float x_limit = 10.0;
  float x = rosbot.get_position(1);
  ROS_INFO_STREAM("X reached: " << x);
 
  if (x <= x_limit) {
      rosbot.move_forward(1);
      rosbot.stop_moving();
  } else {
      rosbot.stop_moving();
  }

  return 0;
}
/*
You will create a simple program that does the following: instruct the robot to move forward for a certain amount of seconds. 
You can pick any number between 1 and 10 seconds for this duration. 
Then using an if statement check if the robot has reached a specific goal position or not. 
You can choose whatever goal position you want, a good value is beteen 1 and 5 meters. 
If the robot has not reached the goal position, the robot will move forward again for one second. 
If it has overpassed it, the robot will stop.
*/
