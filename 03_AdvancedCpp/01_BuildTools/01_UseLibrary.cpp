/*
cd ~/catkin_ws/src
catkin_create_pkg odom_subs nav_msgs rospy roscpp
mkdir -p ~/catkin_ws/src/odom_subs/src
cd ~/catkin_ws/src/odom_subs/src
touch odom_listener.cpp
*/

//-----------------------------------------------------------------------------
/*Copy the following contents in the odom_listener.cpp file. 
Observe carefully that we have included the test_library.h 
to access the function Calculation, written in test_library.cpp
*/
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "../../test_library/include/test_library/test_library.h"


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  float x = msg->pose.pose.position.x;
  float y = msg->pose.pose.position.y;
  ros::Duration(1).sleep();
  display_pos(x,y);
  
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "odom_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);
  
  ros::spin();

  return 0;
}

//-----------------------------------------------------------------------------
// We need to make sure the test_library package is included in the find_package section of CMakeLists.txt:
cmake_minimum_required(VERSION 3.0.2)
project(odom_subs)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  test_library
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_listener src/odom_listener.cpp)
add_dependencies(odom_listener nav_msgs_generate_messages_cpp)
 target_link_libraries(odom_listener
   ${catkin_LIBRARIES}
)

//-----------------------------------------------------------------------------
/*
Now let's compile
cd catkin_ws
catkin_make
source devel/setup.bash


*/


