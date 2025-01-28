#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
  // Initiate a Node called 'simple_subscriber'
  SimpleSubscriber()
  : Node("odom_subscriber")
  {
    // Create a Subscriber object that will listen to the /counter topic and will call the 'topic_callback' function       // each time it reads something from the topic
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  // Define a function called 'topic_callback' that receives a parameter named 'msg' 
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Print the value 'data' inside the 'msg' parameter
    double x_data = msg->pose.pose.position.x;
    RCLCPP_INFO(this->get_logger(), "Robot X position: '%f'", x_data);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Create a loop that will keep the program in execution
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
