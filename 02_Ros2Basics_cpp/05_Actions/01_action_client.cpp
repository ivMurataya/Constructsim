#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

//Import the ROS2 C++ client libraries to work with Actions (rclcpp_action) and (rclcpp)
#include "t3_action_msg/action/move.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


class MyActionClient : public rclcpp::Node
{
public:
  using Move = t3_action_msg::action::Move;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;


//In the constructor of the class, we initialize a ROS2 node named my_action_client. Also, note that we initialize the goal_done_ variable to false.
  explicit MyActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("my_action_client", node_options), goal_done_(false)
  {
  //Then inside the constructor body we create an ActionClient object that connects to the /move_robot_as Action Server
    this->client_ptr_ = rclcpp_action::create_client<Move>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "move_robot_as");
//Finally, we create a timer object, with a callback method named send_goal
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;
//The line below cancels the timer so that it only gets executed one time (in this example, we don't want to keep sending goals to the Action Server):
    this->timer_->cancel();

    this->goal_done_ = false;
//Here, check if the Action Server is up and running. If not, we print a message to the node's log:
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

//Here, we wait for the Action Server to start for 10 seconds. If it is not ready after the 10 seconds pass.
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

//Next, a Goal() object of the Move action type is created. Then, we access the secs variable of the Action goal and assign it an arbitrary numerical value in seconds. In this example its 5 seconds
    auto goal_msg = Move::Goal();
    goal_msg.secs = 5;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
//Here, define the different callbacks for the Action Client:
//The first line above is to initialize the object that is used to set callback methods for goal acceptance, feedback and completion.
    auto send_goal_options = rclcpp_action::Client<Move>::SendGoalOptions();
//As you can see, they are defined immediately after                
    send_goal_options.goal_response_callback =
      std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&MyActionClient::result_callback, this, _1);

      //Finally, we send the goal to the Action Server using the async_send_goal method
/*We provide two arguments for this method:
    A goal message, in this case, goal_msg
    The callback methods of the client
This async_send_goal() method returns a future to a goal handle. 
This future goal handle will be completed when the Server has processed the goal, 
whether that's being accepted or rejected by the Server. So, you must assign a 
callback method to be triggered when the future is completed 
(the goal has been accepted or rejected). In this case, this method is 
goal_response_callback()
        */
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Move>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

//So, this method is triggered when the goal has been either accepted or rejected by the server. We can know this by checking the goal_handle value
  void goal_response_callback(const GoalHandleMove::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

//Here, print the feedback string to the node's log.
  void feedback_callback(
    GoalHandleMove::SharedPtr,
    const std::shared_ptr<const Move::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback received: %s", feedback->feedback.c_str());
  }

  void result_callback(const GoalHandleMove::WrappedResult & result)
  //First, check the result.code variable to see what happened with your goal. Next, print the status variable from our result
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Result received: %s", result.result->status.c_str());

  }
};  // class MyActionClient


//the main function that uses executors
//Here you are creating an instance of the MyActionClient(). Then we initialize one MultiThreadedExecutor object.
//While the goal has not been completed, while (!action_client->is_goal_done()), you will spin the executor executor.spin_some();
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}


/*
int32 secs
---
string status
---
string feedback
*/
