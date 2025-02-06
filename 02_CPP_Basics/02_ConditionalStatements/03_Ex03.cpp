#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>
#include <list>


int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move();

  list<float> list_coordinates;
  list_coordinates = rosbot.get_position_full();
  for (float coordinate : list_coordinates) {
    ROS_INFO_STREAM(coordinate << ", ");
  }

  return 0;
}

/*
    Make the robot move with the method move() for as many seconds as you wish
    Call the method get_position_full() to obtain a list with the coordinates
    Use a for loop to print the coordinates
*/
