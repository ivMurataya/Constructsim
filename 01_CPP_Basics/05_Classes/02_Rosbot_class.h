/*, we are including some ROS-related modules that we will use, 
and also the modules to use lists and strings in your program, just like in previous lessons*/
#ifndef ROSBOT_CLASS_H
#define ROSBOT_CLASS_H
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <list>
#include <ros/ros.h>
#include <string>

using namespace std;

//we can see the definition of the class:
class RosbotClass {
/*The first variables we are going to declare are ROS related, and they communicate directly with the robot. 
That's why we set them private, to keep better control of them.
Also, the two methods that receive directly messages from the robot are set to private*/

private: //Now we can see an access specifier:
  // Communicate with nodes
  ros::NodeHandle n;
  // Laser data
  ros::Subscriber laser_sub;
  std::vector<float> laser_range;
  std::string laser_topic;
  // Velocity data
  ros::Publisher vel_pub;
  geometry_msgs::Twist vel_msg;
  std::string vel_topic;
  // Odometry data
  ros::Subscriber odom_sub;
  std::string odom_topic;
  float x_pos;
  float y_pos;
  float z_pos;

  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

public:
/*Then, as we said in this chapter, comes the constructor of the class. 
In this case we don't need to pass initial parameters to the RosbotClass, 
so this constructor remains emtpy. It just initializes the class when an object is instanciated.*/
  RosbotClass();
//See below the initialized functions that you have been using in order to move the robots around
  void move();
  void move_forward(int n_secs);
  void move_backwards(int n_secs);
  void turn(string clock, int n_secs);
  void stop_moving();
  float get_position(int param);
  std::list<float> get_position_full();
  double get_time();
  float get_laser(int index);
  float *get_laser_full();
};

#endif
