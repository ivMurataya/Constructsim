#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <memory>
#include <string>

class GetObjectPoseActionClient : public rclcpp::Node {

public:
  explicit GetObjectPoseActionClient(
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
      : Node("get_object_pose_action_client", node_options) {

    RCLCPP_INFO(this->get_logger(),
                "Initializing Node: get_object_pose_action_client...");

    // initialize action client
    this->action_client_ = rclcpp_action::create_client<FindGraspableObjects>(
        this->get_node_base_interface(), this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(), action_name_);
    RCLCPP_INFO(this->get_logger(), "Initialized Action Client for %s",
                action_name_.c_str());

    // indicate initialization
    RCLCPP_INFO(this->get_logger(),
                "Initialized Node: get_object_pose_action_client");
  }

  ~GetObjectPoseActionClient() {
    // indicate termination
    RCLCPP_INFO(this->get_logger(),
                "Terminated Node: get_object_pose_action_client");
  }

  bool is_goal_done() const { return this->goal_done_; }

  void send_goal() {
    // send goal to the action server

    // reset goal done variable to false
    this->goal_done_ = false;

    // check if action client is ready
    if (!this->action_client_) {
      RCLCPP_ERROR(this->get_logger(), "Action Client is Not Initialized");
    } else {
      // do nothing
    }

    // check if action server is ready
    if (!this->action_client_->wait_for_action_server(
            std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action Server Unavailable. Waiting Timed Out.");
      this->goal_done_ = true;
      return;
    } else {
      RCLCPP_INFO(this->get_logger(), "Awaiting Action Server: %s",
                  action_name_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Sending Action Goal...");

    // set action goal
    auto goal_msg = FindGraspableObjects::Goal();
    goal_msg.plan_grasps = false;

    // prepare action goal handler
    auto send_goal_options =
        rclcpp_action::Client<FindGraspableObjects>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&GetObjectPoseActionClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.result_callback =
        std::bind(&GetObjectPoseActionClient::result_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&GetObjectPoseActionClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);

    // send action goal to action server
    auto goal_handle_future =
        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Goal Sent to Action Server: %s",
                action_name_.c_str());
  }

private:
  // using shorthand for lengthy class references
  using FindGraspableObjects = grasping_msgs::action::FindGraspableObjects;
  using FindGraspableObjectsGoalHandle =
      rclcpp_action::ClientGoalHandle<FindGraspableObjects>;

  // declare action client
  rclcpp_action::Client<FindGraspableObjects>::SharedPtr action_client_;

  // declare action client variables
  const std::string action_name_ = "/find_objects";
  bool goal_done_ = false;

  void goal_response_callback(
      const FindGraspableObjectsGoalHandle::SharedPtr &goal_handle) {
    // function to check if action goal is accepted or rejected by server
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal Rejected by Action Server: %s",
                   action_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal Accepted by Action Server: %s",
                  action_name_.c_str());
    }
  }

  void
  feedback_callback(FindGraspableObjectsGoalHandle::SharedPtr,
                    const std::shared_ptr<const FindGraspableObjects::Feedback>
                        action_feedback) {
    // function to check for action feedback from server
    RCLCPP_INFO(this->get_logger(), "Feedback: Ignored");
  }

  void result_callback(
      const FindGraspableObjectsGoalHandle::WrappedResult &action_result) {
    // function to check for action completion and result from server
    RCLCPP_INFO(this->get_logger(), "Getting Result from Action Server: %s",
                action_name_.c_str());
    this->goal_done_ = true;
    switch (action_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was Aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was Canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown Result Code");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Result: Received !");

    // get the first object from detected list of objects
    RCLCPP_INFO(this->get_logger(), "# ~~~~~~~~~~~ ~~~~~~~~~~~ ~~~~~~~~~~~ #");
    RCLCPP_INFO(this->get_logger(), "Result: Object of Interest Position:");
    RCLCPP_INFO(
        this->get_logger(), "PosX: %+0.3f, PosY: %+0.3f, PosZ: %+0.3f",
        action_result.result->objects[0].object.primitive_poses[0].position.x,
        action_result.result->objects[0].object.primitive_poses[0].position.y,
        action_result.result->objects[0].object.primitive_poses[0].position.z);
    RCLCPP_INFO(this->get_logger(), "# ~~~~~~~~~~~ ~~~~~~~~~~~ ~~~~~~~~~~~ #");
  }

}; // class GetObjectPoseActionClient

int main(int argc, char *argv[]) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize action client
  auto action_client = std::make_shared<GetObjectPoseActionClient>();

  // send a goal to action server
  action_client->send_goal();
  while (!action_client->is_goal_done()) {
    rclcpp::spin_some(action_client);
  }

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
