#include "rosbot_control/rosbot_class.h"
#include <ros/ros.h>

#include <string>

using namespace std;

// New implemented class
class RosbotMove {
    
public:
    
  RosbotMove(string a) { left_or_right = a; };
  string left_or_right;
    
  // Inherit RosbotClass from the file rosbot_class.h
  RosbotClass rosbot;
  void avoid_wall();
};

void RosbotMove::avoid_wall() {
  // Start moving
  rosbot.move_forward(1);
    
  // Evaluate distance with laser
  while (rosbot.get_laser(0) > 1.75) {
    ROS_INFO_STREAM("Laser frontal reading: " << rosbot.get_laser(0));
    rosbot.move_forward(1);
  }
    
  // Decide if left or right to avoid the wall
  if (left_or_right == "left") {
      
    rosbot.turn("counterclockwise", 3);
    rosbot.move_forward(5);
      
  } else if (left_or_right == "right") {
      
    rosbot.turn("clockwise", 3);
    rosbot.move_forward(5);

  } else {
    ROS_INFO_STREAM("Error: specify a direction for the movement: left or "
                    "right");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Rosbot_move_node");
  RosbotMove rosbot_moves("left");
  rosbot_moves.avoid_wall();
}

/*
Import the RosbotClass
Use the get_laser(), move_forward() and turn() functions of that class
Create the constructor of your class
Create a function to avoid the wall that contains the trajectory that the robot must do, as painted on the image above: make the robot move until the frontal laser returns a small value, which means that the robot is too close to the wall and has to turn.
Use a while loop to continuously evaluate this distance, and keep the robot moving until it is too close to the wall.
Use an if - else conditional to decide to go left or right once the robot is close to the wall
*/
