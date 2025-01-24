/*


This C++ line includes a header file named rosbot_class.h from the rosbot_control directory.

The #include directive is a preprocessor directive that tells the compiler to include the contents of the specified header file in the source code file.

By including the rosbot_class.h header file, the C++ program can access the classes, functions, and variables defined in that header file. In this case the rosbot_class.h file contains the class definition and implementation for a class called RosbotClass which we will use to make the robot move.

*/
#include "rosbot_control/rosbot_class.h"

/*This line of C++ code includes the ROS (Robot Operating System) header file ros.h from the ros package, we import this library to make use of the classes and functions that ROS provides.*/
#include <iostream>
#include <ros/ros.h>

//This line means that we can use the code in the std namespace without typing std:: before it. For example, we can write cout instead of std::cout.
using namespace std;

int main(int argc, char **argv) {
//y calling ros::init(), the C++ program creates a ROS node with the specified name, in this case rosbot_node and establishes a connection with the ROS master node. This line of code is typically included at the beginning of any ROS program.
  ros::init(argc, argv, "rosbot_node");

  RosbotClass rosbot;
  rosbot.move();

  /*Finally, we are calling the get_position() method provided by the RosbotClass class. This method gives us the position of the robot in x, y and z coordinates. In this particular case, passing number 1 to the method means we are asking for the coordinate x, number 2 for coordinate y and number 3 for coordinate z.*/

  float x_0 = rosbot.get_position(1);
  double t_0 = rosbot.get_time();

  rosbot.move();

  float x_1 = rosbot.get_position(1);
  double t_1 = rosbot.get_time();

  float speed = (x_1 - x_0)/(t_1 - t_0); 
  ROS_INFO_STREAM("Speed is lower than 1 m/s? " << (speed<=1.0) << "\n");


  return 0;
}
