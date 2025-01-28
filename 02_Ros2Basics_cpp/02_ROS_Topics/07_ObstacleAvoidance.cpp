#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class WallDetector : public rclcpp::Node
{
public:
  WallDetector()
  : Node("topics_quiz_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&WallDetector::topic_callback, this, std::placeholders::_1));
    
  }

private:
  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto twist_msg = geometry_msgs::msg::Twist();
     // Extract and print range values from specific indices
      float front_distance = msg->ranges[msg->ranges.size() / 2]; // Middle index (front)
      float right_distance = msg->ranges[5 * msg->ranges.size() / 6]; // Right side
      float left_distance = msg->ranges[msg->ranges.size() / 6]; // Right side
      
    //RCLCPP_INFO(this->get_logger(), "Right: '%f'  FRONT: '%f'  LEFT: '%f'",right_distance,front_distance,left_distance);

     // Logic to determine the robot's movement
    if (front_distance > 1.0 && right_distance > 1.0 && left_distance > 1.0) {
      RCLCPP_INFO(this->get_logger(), "No obstacle nearby. Moving forward.");
      twist_msg.linear.x = 0.5; // Move forward
      twist_msg.angular.z = 0.0;
    } else if (front_distance < 1.0) {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected in front. Turning left.");
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.5; // Turn left
    } else if (right_distance < 1.0) {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected on the right. Turning left.");
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.5; // Turn left
    } else if (left_distance < 1.0) {
      RCLCPP_INFO(this->get_logger(), "Obstacle detected on the left. Turning right.");
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = -0.5; // Turn right
    }
    
    
   publisher_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDetector>());
  rclcpp::shutdown();
  return 0;
}
