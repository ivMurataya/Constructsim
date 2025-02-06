#include "robot_commander/robot_commander.h"
#include <ros/ros.h>
#include <deque>

int main(int argc, char **argv) {
  ros::init(argc, argv, "heading_vector");

  RobotCommander my_robot;

  // command the robot to keep going in circles
  my_robot.move_in_circles();

  int total_values;
  std::cout << "Enter the number of heading values that you want to capture: ";
  // get user input from the keyboard
  std::cin >> total_values;

  // INSERT YOUR CODE BELOW THIS LINE

  // declare a vector of floats
  std::deque<float> headings;

  int i = 0;
  while (i < total_values) {
   float a = my_robot.get_heading();
    headings.push_front(a);
    std::cout << "Collecting heading value: " << a << std::endl;
    i++;
    usleep(1000000);
  }

  // new line
  std::cout << std::endl;
  std::cout << "Displaying saved data" << std::endl;
  for (int i = 0; i < headings.size(); i++) {
    std::cout << "Heading value " << i << ": " << headings.at(i) << std::endl;
  }


  // INSERT YOUR CODE ABOVE THIS LINE

  // stop the robot
  my_robot.stop_moving();

  return 0;
}
/*


This exercise is very similar to the previous one just that we will be storing the latest collected value always at the first position.

To start add a new blank file named robot_heading_as_deque.cpp inside the src directory of the exercises_unit_2 package.

The program should use an object of class RobotCommander to continuously get the current heading angle from the robot and store that data to a deque.

Base your program on the starter code from previous exercise.

In your part of the code you will have to:

    Declare a deque object for storing float values, you can name it headings
    Use the get_heading() method of the RobotCommander class to retieve the current robot heading
    Add the heading data to the front of the deque container
    Print the each collected data value to the console at the time it gets collected, like so:


*/
