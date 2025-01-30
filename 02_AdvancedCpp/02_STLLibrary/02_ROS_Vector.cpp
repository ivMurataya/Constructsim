#include "robot_commander/robot_commander.h"
#include <ros/ros.h>

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
    
    

    
  // INSERT YOUR CODE ABOVE THIS LINE

  // stop the robot
  my_robot.stop_moving();

  return 0;
}
/*Your goal for this exercise is to utilize a RobotCommander object to continuously get the current heading angle from the robot and store that data to a vector.

To start, add a new blank file named robot_heading_as_vector.cpp inside the src directory of the exercises_unit_2 package from the previous exercise (Exercise 2.1).

You can use the stater code from below, or if you want you can write your own program from scratch. Just remember to include the robot_commander library by adding this line #include "robot_commander/robot_commander.h".

The starter code:

    Creates a standard C++ main function
    Initializes a ros node named "heading_vector" by calling ros::init(argc, argv, "heading_vector");
    Creates an object of type RobotCommander
    Uses the move_in_circles() method of the RobotCommander class so that the robot moves continuously while we capture data
    Asks the user to input the number of values to be captured
    Stops the robot before the program exits

In your part of the code you will have to:

    Declare a vector object for storing float values, you can name it headings
    Use the get_heading() method of the RobotCommander class to retieve the current robot heading
    Populate the headings vector with the values that you get
------------------------------------------------------------------------------------------------    
add_executable(robot_heading_as_vector src/robot_heading_as_vector.cpp)
target_link_libraries(robot_heading_as_vector ${catkin_LIBRARIES})

    */
