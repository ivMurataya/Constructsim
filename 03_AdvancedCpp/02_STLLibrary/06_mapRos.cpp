#include "robot_commander/robot_commander.h"
#include <ros/ros.h>
#include <map>
#include <iostream>
#include <unistd.h>  // For usleep

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "heading_vector");

    RobotCommander my_robot;
    map<int, pair<float, float>> vertices;  // ✅ Store (key -> (X, Y)) pairs

    int total_values;
    std::cout << "Enter the number of polygons: ";
    std::cin >> total_values;

    int time;
    std::cout << "Time moving: ";
    std::cin >> time;

    for (int i = 1; i <= total_values; i++) {  // ✅ Start from 1 and increment
        my_robot.move_forward(time);
        my_robot.turn(0.5, 1.5);
        my_robot.stop_moving();
        
        float x = my_robot.get_x_position();  // ✅ Get X position
        float y = my_robot.get_y_position();  // ✅ Get Y position
        vertices[i] = {x, y};  // ✅ Store using key i

        std::cout << "Vertex " << i << " = (" << x << ", " << y << ")" << std::endl;
        
        usleep(1000000);
    }

    // ✅ Printing stored vertices
    std::cout << "\nStored Vertices:\n";
    for (const auto& v : vertices) {
        std::cout << "Vertex " << v.first << ": (" << v.second.first << ", " << v.second.second << ")\n";
    }

    my_robot.stop_moving();
    return 0;
}
/*
Stored Vertices:
Vertex 1: (0.408189, -0.00173129)
Vertex 2: (0.762384, 0.177066)
Vertex 3: (1.0052, 0.491583)
Vertex 4: (1.09672, 0.878403)
Vertex 5: (1.00613, 1.26406)
[ INFO] [1738280112.711235760, 1235.150000000]: Stopping the robot
*/
