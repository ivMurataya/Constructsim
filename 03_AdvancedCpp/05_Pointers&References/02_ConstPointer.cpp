#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserSubscriberNode
{
public:
  LaserSubscriberNode() : laser_scan(nullptr)
  {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Subscribe to the laser scan topic
    laser_scan_sub = nh.subscribe("/kobuki/laser/scan", 10, &LaserSubscriberNode::laserScanCallback, this);
  }

private:
  // Laser scan callback function
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
  {
    // Store the laser scan data using the const pointer member variable
    laser_scan = msg.get();

    // Print the range at index 360 (robot front)
    if (!laser_scan->ranges.empty())
    {
      float range = laser_scan->ranges[360];
      ROS_INFO("Range: %f", range);
    }
  }

  // The const qualifier indicates that the pointer itself is constant
  // meaning it cannot be reassigned to point to a different object
  const sensor_msgs::LaserScan* laser_scan;
    
  ros::Subscriber laser_scan_sub;
};

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "laser_subscriber_node");

  // Create an instance of the LaserSubscriberNode class
  LaserSubscriberNode node;

  // Spin the node
  ros::spin();

  return 0;
}
