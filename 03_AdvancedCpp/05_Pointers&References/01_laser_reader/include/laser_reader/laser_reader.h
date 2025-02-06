#ifndef LASER_TURTLE_H
#define LASER_TURTLE_H
#include "sensor_msgs/LaserScan.h"
#include <ros/ros.h>

class TurtleClass {
private:
  ros::NodeHandle n;
  ros::Subscriber laser_sub;
  void laser_callback(const sensor_msgs::LaserScan::ConstPtr &laser_msg);

  /* DEFINE HERE THE POINTER TO THE LASER RANGES ARRAY */
  float *last_laser_ranges;

public:
  TurtleClass();
  ~TurtleClass();
};

#endif
