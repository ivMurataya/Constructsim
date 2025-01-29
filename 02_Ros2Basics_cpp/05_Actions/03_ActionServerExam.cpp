//Begin with the first section, where you will import the necessary libraries.
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"

//#include "t3_action_msg/action/move.hpp"
#include "actions_quiz_msg/action/distance.hpp"
#include "geometry_msgs/msg/twist.hpp"

//Next, define your class MyActionServer that inherits from Node
/*
In the constructor of the class, initialize a ROS2 node named my_action_server. Also, and important, create an ActionServer object to which you specify several things:

    The type of Action: Move.
    The ROS2 node that contains the Action Server: in this case, this.
    A callback method to be executed when the Action receives a goal: handle_goal.
    A callback method to be executed if the Client cancels the current goal: handle_cancel.
    A callback method to be executed if the goal is accepted: handle_accepted.

Finally, define a Publisher for the Topic /cmd_vel
*/
class MyActionServer : public rclcpp::Node
{
public:
//Note that you are also creating some "alias" to simplify the code
  //using Move = t3_action_msg::action::Move;
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("my_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
      this,
      "distance_as",
      std::bind(&MyActionServer::handle_goal, this, _1, _2),
      std::bind(&MyActionServer::handle_cancel, this, _1),
      std::bind(&MyActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32>("total_distance", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MyActionServer::odom_callback, this, _1));
    

  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  double start_x_ = 0.0, start_y_ = 0.0;
  bool first_odom_received_ = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!first_odom_received_)
    {
      start_x_ = msg->pose.pose.position.x;
      start_y_ = msg->pose.pose.position.y;
      first_odom_received_ = true;
      RCLCPP_INFO(this->get_logger(), "Odom X: %f , Odom Y: %f", msg->pose.pose.position.x,  msg->pose.pose.position.y);
    }
    latest_odom_ = msg;
    
    
  }

//This method will be called when the Action Server receives a goal. In this case, you are accepting all the incoming goals with rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Distance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

//This method will be called when the Action Server receives a cancellation request from the Client. In this case, you are accepting all the cancellation requests with rclcpp_action::CancelResponse::ACCEPT
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

//This method will be called when the Action Server accepts the goal. Here, you are calling the execute function, which contains the main functionality of the Action
  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }


/*

You have:

    The goal variable, containing the goal message sent by the Client.

    The feedback variable, containing the feedback message that you will send back to the Client.

    The result variable, containing the result message that you will send to the Client when the Action finishes.

    The move variable, containing the Twist message used to send velocities to the robot.

    A loop_rate variable of 1Hz (1 second).

*/
  void execute(const std::shared_ptr<GoalHandleMove> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto & message = feedback->current_dist;
    message = 0.0;
    auto result = std::make_shared<Distance::Result>();
    auto final_total = std_msgs::msg::Float32();
    rclcpp::Rate loop_rate(1);
        
    first_odom_received_ = false;
    double total_distance = 0.0;

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = false;
        result->total_dist = total_distance;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      if (first_odom_received_) {
        total_distance = compute_distance();
        message = total_distance;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");
      }

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      result->total_dist = total_distance;
      final_total.data = total_distance;
      publisher_->publish(final_total);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      RCLCPP_INFO(this->get_logger(), "Goal succeeded with distance = %f", total_distance);
    }
  }

    double compute_distance()
  {
    auto odom_msg = this->get_current_odometry();
    //RCLCPP_INFO(this->get_logger(), "NEW Odom X: %f , Odom Y: %f", odom_msg.pose.pose.position.x,  odom_msg.pose.pose.position.y);
    double dx = odom_msg.pose.pose.position.x - start_x_;
    double dy = odom_msg.pose.pose.position.y - start_y_;
    return std::sqrt(dx * dx + dy * dy);
  }

    nav_msgs::msg::Odometry get_current_odometry()
    {
    if (latest_odom_) {
        
        return *latest_odom_;  // Return stored odometry
    } else {
        return nav_msgs::msg::Odometry();  // Return default odometry if no data received yet
    }
    }

};  // class MyActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<MyActionServer>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}


/*
int32 seconds
---
bool status
float64 total_dist
---
float64 current_dist
*/

/*


    Create another package named actions_quiz. In this package, place the C++ files containing the Actions Server and Client, and the launch files to start them.

    Create the C++ file that will contain the Action Server code.

    Use the data received in the seconds variable to create a loop. Inside this loop, do the following:
        Publish the current distance traveled by the robot to a Topic named /total_distance.
        Publish the current distance traveled by the robot as feedback of the Action.

    When the loop finishes (time has passed) return the result message containing the following:
        The final total distance traveled by the robot.
        A boolean as True

    Create a new launch file, named actions_quiz_server.launch.py, that launches the new Action.

    Test that when calling the /distance_as Action, the Action correctly computes the distance traveled by the robot. This means the distance value increases accordingly as the robot keeps moving.

    Test also that the Action publishes the current distance traveled value to the topic /total_distance.

    Create another C++ file that contains the Action Client code. This Client calls the Action /distance_as, and makes the Action compute the traveled distance for 20 seconds.

    Create the launch file that starts the Action Client and name it actions_quiz_client.launch.py.


*/
