#include "rosbot_control/rosbot_class.h"
#include <iostream>
#include <list>
#include <map>
#include <ros/ros.h>
#include <string>

using namespace std;

int main(int argc, char **argv) {
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move();

  float x_0 = rosbot.get_position(1);
  double t_0 = rosbot.get_time();

  ROS_INFO_STREAM(x_0 << " and " << t_0);
  rosbot.move();

  float x_1 = rosbot.get_position(1);
  double t_1 = rosbot.get_time();
  ROS_INFO_STREAM(x_1 << " and " << t_1);

  map<double, float> x_t_dictionary;
  x_t_dictionary[t_0] = x_0;
  x_t_dictionary[t_1] = x_1;

  for (auto item : x_t_dictionary) {
    ROS_INFO_STREAM("Time " << item.first << ", position " << item.second
                            << " \n");
  }

  return 0;
}
/*
 Get the x coordinate of the robot by calling the get_position() method, and also get the time of simulation by calling the get_time() method. Note that get_position() and get_time() are methods of the RosbotClass and you need to type rosbot.get_time() instead of just get_time() which is what you would write to call a regular function. (Hint: the timestamp will have a data type of double).

Then, make the robot move by calling the method move() which also belongs to the RosbotClass class.

Repeat step 1: take the x position and the timestamp.

Instead of printing them in the shell, you will initialize a dictionary and store the time obtained as a key, and the x position as a value. Do it for all the values you obtained of x, 1 time, 2 times, .. , as many as you want.

Print the dictionary with the code provided in the Dictionaries section.
*/
