#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

//------------------------------------------------------------------------------------------

// Complete grasping movenet

//------------------------------------------------------------------------------------------

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";


class DrawXCartesianTrajectory {
public:
  DrawXCartesianTrajectory(rclcpp::Node::SharedPtr base_node_)
      : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Draw X Cartesian Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
 // get initial state of gripper
    joint_model_group_gripper_ = move_group_gripper_->getCurrentState()->getJointModelGroup( PLANNING_GROUP_GRIPPER);
    

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);

    // set start state of robot to current state
    move_group_robot_->setStartStateToCurrentState();

        // get current state of gripper
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of gripper to current state
    move_group_gripper_->setStartStateToCurrentState();
    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Draw X Cartesian Trajectory");
  }

  ~DrawXCartesianTrajectory() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Draw X Cartesian Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER,
                "Planning and Executing Draw X Cartesian Trajectory...");

    RCLCPP_INFO(LOGGER, "1.- Home ");
    //setup_joint_value_target(+0.0000, -2.3562, +1.5708, -1.5708, -1.5708, +0.0000);
    setup_named_pose_arm("home");
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
     
    
    RCLCPP_INFO(LOGGER, "2.- Position ");
    setup_goal_pose_target(+0.343, +0.132, +0.264, +1.000, +0.000, +0.000, +0.000);
    plan_trajectory_kinematics();
    execute_trajectory_kinematics();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));


    RCLCPP_INFO(LOGGER, "3.- Open Gripper ");
    setup_named_pose_gripper("gripper_open");
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));


    RCLCPP_INFO(LOGGER, "4.- Down");
    setup_waypoints_target(+0.000, +0.000, -0.075);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    RCLCPP_INFO(LOGGER, "5.-  Close Gripper ");
    setup_joint_value_gripper(+0.500);
    plan_trajectory_gripper();
    execute_trajectory_gripper();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    RCLCPP_INFO(LOGGER, "6.- UP");
    setup_waypoints_target(+0.000, +0.000, +0.075);
    plan_trajectory_cartesian();
    execute_trajectory_cartesian();
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));



    RCLCPP_INFO(LOGGER, "Draw X Cartesian Trajectory Execution Complete");
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
// declare move_group_interface variables for gripper
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;


  // declare joint_model_group for robot
  const JointModelGroup *joint_model_group_robot_;
    // declare joint_model_group for gripper
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
    std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;

  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;

Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  bool plan_success_robot_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void setup_named_pose_arm(std::string pose_name) {
    // Set the joint values for each joint of the arm
    // based on predefined pose names in MoveIt 2
    move_group_robot_->setNamedTarget(pose_name);
}

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

    void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }}

}; // class DrawXCartesianTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("draw_x_cartesian_trajectory");

  // instantiate class
  DrawXCartesianTrajectory draw_x_cartesian_trajectory_node(base_node);

  // execute trajectory plan
  draw_x_cartesian_trajectory_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
