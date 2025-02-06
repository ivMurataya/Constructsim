#include "robot_commander/robot_commander.h"
#include <ros/ros.h>
#include <set>
#include <iostream>
#include <unistd.h>  // For usleep
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "heading_vector");

    RobotCommander my_robot;
    set<pair<float, float>> coordinates;  // ✅ Store (X, Y) coordinates

    int total_values;
    std::cout << "Enter the number of polygons: ";
    std::cin >> total_values;

    int time;
    std::cout << "Time moving: ";
    std::cin >> time;

    int i = 0;
    while (i < total_values) {
        my_robot.move_forward(time);
        my_robot.turn(0.5, 2.5);  // ✅ Fixed incorrect function call
        my_robot.stop_moving();
        
        float x = my_robot.get_x_position();  // ✅ Get X position
        float y = my_robot.get_y_position();  // ✅ Get Y position
        coordinates.insert({x, y});  // ✅ Insert (X, Y) pair

        std::cout << "Value " << i << " = (" << x << ", " << y << ")" << std::endl;
        
        usleep(1000000);  // ✅ Wait for 1 second
        i++;  
    }

    // ✅ Printing saved (X, Y) coordinates
    std::cout << "\nStored (X, Y) coordinates:\n";
    for (const auto& coord : coordinates) {
        std::cout << "(" << coord.first << ", " << coord.second << ")\n";
    }

    my_robot.stop_moving();
    return 0;
}


/*


Add a new blank file named path_coordinates.cpp inside the src directory of the exercises_unit_2 package.

As in previous exercises, the program should use an object of class RobotCommander and call its methods to interact with the robot.

In your code you will have to:

    Make the robot trace a polygonal path (of your choice).
    Use the methods move_forward(int time) to move forward for n seconds and move_backward(int time) to move backwards for n seconds.
    Use the method turn(float velocity_value, int n_secs) to turn the robot. Specify the velocity (in rad/s) with velocity_value and the duration of the turn with n_secs.
    Use the method stop_moving() for stopping the robot.
    Store the (X,Y) coordinates of the vertex of the plygon traced in a set.
    Use the methods get_x_position() and get_y_position() to get the current coordinates.
    Print the (X,Y) coordinates of the vertex.
    Change the code to store the coordinates in a multiset to observe the difference.

Stored (X, Y) coordinates:
(-0.183177, 0.550106)
(-0.148554, 0.156413)
(0.125974, 0.797508)
(0.188638, -0.0518845)
(0.396989, -0.0027071)
(0.506828, 0.687987)
(0.635032, 0.313946)
*/
