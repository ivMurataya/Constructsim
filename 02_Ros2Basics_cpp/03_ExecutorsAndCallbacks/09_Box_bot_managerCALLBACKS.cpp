#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <unistd.h>
//THIS CODE IS NOT COMPLETED, PLEASE CHECK SOLUTION
using namespace std::chrono_literals;

class BoxBotManager : public rclcpp::Node {
public:
  BoxBotManager(std::string odom_topic_name1,std::string odom_topic_name2,
                std::string odom_topic_name3 ,geometry_msgs::msg::Point goal1,
                geometry_msgs::msg::Point goal2,
                geometry_msgs::msg::Point goal3)
      : Node("box_bot_node") {

    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom2_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom3_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);


    rclcpp::SubscriptionOptions options1;
    rclcpp::SubscriptionOptions options2;
    rclcpp::SubscriptionOptions options3;

    options1.callback_group = odom1_callback_group_;
    options2.callback_group = odom2_callback_group_;
    options3.callback_group = odom3_callback_group_;

    this->  goal_1 = goal1;
    this->  goal_2 = goal2;
    this->  goal_3 = goal3;
    this-> wait_time1 = 1.0;
    

    timer_ = this->create_wall_timer(
        500ms, std::bind(&BoxBotManager::timer_callback, this), callback_group_1);

    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name1, 10,
        std::bind(&BoxBotManager::odom1_callback, this, std::placeholders::_1),
        options1);

    
    subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name2, 10,
        std::bind(&BoxBotManager::odom2_callback, this, std::placeholders::_1),
        options2);

    
    subscription3_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_name3, 10,
        std::bind(&BoxBotManager::odom3_callback, this, std::placeholders::_1),
        options3);
  }

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback Start");
    sleep(this->wait_time1);
    RCLCPP_INFO(this->get_logger(), "Timer 1 Callback End");
  }


    void odom1_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Odom 1: '%f'", msg->pose.pose.position.x);
  }
     void odom2_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Odom 2: '%f'", msg->pose.pose.position.x);
  }
   void odom3_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Odom 3: '%f'", msg->pose.pose.position.x);
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom2_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom3_callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  float wait_time1;
  

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription3_;
  
  geometry_msgs::msg::Point goal_1;
  geometry_msgs::msg::Point goal_2;
  geometry_msgs::msg::Point goal_3;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

    geometry_msgs::msg::Point goal1;
    geometry_msgs::msg::Point goal2;
    geometry_msgs::msg::Point goal3;

    goal1.x = 2.498409;
    goal1.y = -1.132045;

    goal2.x = 0.974281;
    goal2.y = -1.132045;

    goal3.x = -0.507990;
    goal3.y = -1.132045;


  std::shared_ptr<BoxBotManager> two_timer_node =
      std::make_shared<BoxBotManager>("/box_bot_1/odom","/box_bot_2/odom",
                                "/box_bot_3/odom",goal1, goal2, goal3);

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(two_timer_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
