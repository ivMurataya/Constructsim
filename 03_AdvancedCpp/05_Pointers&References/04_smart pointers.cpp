#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LaserSubscriberNode {
public:
  LaserSubscriberNode() : laser_scan_(nullptr) {
    // Initialize the ROS node
    ros::NodeHandle nh;

    // Subscribe to the laser scan topic
    laser_scan_sub =
        nh.subscribe("/kobuki/laser/scan", 10,
                     &LaserSubscriberNode::laserScanCallback, this);
  }

  void printClosestObstacleDistance() const {
    float min_range = laser_scan_->ranges[0];
    int min_index = 0;

    // Find the index of the minimum range in the laser scan data
    for (int i = 1; i < laser_scan_->ranges.size(); i++) {
      if (laser_scan_->ranges[i] < min_range) {
        min_range = laser_scan_->ranges[i];
        min_index = i;
      }
    }

    // Calculate the angle at which the closest obstacle is found
    float angle =
        laser_scan_->angle_min + min_index * laser_scan_->angle_increment;

    // Print the distance to the closest obstacle and the corresponding angle in
    // radian
    std::cout << "Closest obstacle distance: " << min_range
              << ", Angle (rad): " << angle << std::endl;
  }

private:
  // Laser scan callback function
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr &msg) {
    // Store the laser scan data using a shared pointer
    laser_scan_ = std::make_shared<sensor_msgs::LaserScan>(*msg);

    if (laser_scan_ != nullptr && !laser_scan_->ranges.empty()) {
      // Call the function to print the distance to the closest obstacle
      printClosestObstacleDistance();
    }
  }

  std::shared_ptr<sensor_msgs::LaserScan> laser_scan_;
  ros::Subscriber laser_scan_sub;
};

int main(int argc, char **argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "laser_subscriber_node");

  // Create an instance of the LaserSubscriberNode class
  LaserSubscriberNode node;

  // Spin the node
  ros::spin();

  return 0;
}
