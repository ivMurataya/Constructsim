/*

For this exercise you will have to create a new ROS package called exercises_unit_2 that depends on roscpp and robot_commander. 
Inside of it create a new empty file in the src directory called robot_position_as_array.cpp.

As mentioned, your program will make use of the RobotCommander class which is already provided to you and defined 
within the robot_commander ROS package located inside ~/catkin_ws/src/modern_cpp_course. 
To use this class you will have to add this line to your code: #include "robot_commander/robot_commander.h".

To get the current robot position it is important that you add this line of code to your source code file:

RobotCommander my_robot;

This will create an object called my_robot. Don't worry much about what objects are, this will be explained on an upcomming unit. 
What is important for you is that you can use this syntax here to get the x,y,z position of the robot:

my_robot.get_x_position();
  my_robot.get_y_position();
  my_robot.get_z_position();

Tasks:

In your code you will have to:

    Create a standard C++ main function
    Initialize a ros node named "position_array" by calling ros::init(argc, argv, "position_array");
    Create an object of type RobotCommander as shown above
    Declare an array composed of 3 float values
    Proceed to populate the array with the robot's x position by using my_robot.get_x_position(); as shown above
    Get and populate the values for the y and z coordinates similarly
    Finally print the values of all elements of the array

cd ~/catkin_ws/src
catkin_create_pkg exercises_unit_2 roscpp robot_commander
touch ~/catkin_ws/src/exercises_unit_2/src/robot_position_as_array.cpp
*/
#include "robot_commander/robot_commander.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "position_array");

  RobotCommander my_robot;

  float my_array[3];
  my_array[0] = my_robot.get_x_position();
  my_array[1] = my_robot.get_y_position();
  my_array[2] = my_robot.get_z_position();

  for (int i = 0; i < 3; i++)
    std::cout << my_array[i] << " ";
    std::cout << std::endl;
  return 0;
}
/*
add_executable(robot_position_as_array src/robot_position_as_array.cpp)
target_link_libraries(robot_position_as_array ${catkin_LIBRARIES})

rosrun exercises_unit_2 robot_position_as_array

*/
