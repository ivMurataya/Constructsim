/*********************************************************************
 ml
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "custom_nav2_planner_plugin/wavy_line_planner.hpp"

namespace nav2_wavyline_planner_namespace_name
{

void WavyLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void WavyLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void WavyLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void WavyLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path WavyLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;

    // You only create zig zag if total_number_of_loop > 3
         RCLCPP_INFO(
    node_->get_logger(), "holaaaaaaaaa");
    float x_sin_increment = 0.0;
    float y_cos_increment = 0.0;
    float angle_increment = 2.0 * PI / total_number_of_loop;
    float i_angle = 0.0;

    if (total_number_of_loop > 3){
      

      i_angle = angle_increment * i;
      
      int A = 40;
      float frequency = 2.0;
      x_sin_increment = A * x_increment * sin (i_angle*frequency);
      y_cos_increment = A * y_increment * cos (i_angle*frequency);

      RCLCPP_WARN(
      node_->get_logger(), ">>>>>>>> SIN X=%f,COS Y=%f, ANGLE=%f, index=%i", x_sin_increment, y_cos_increment, i_angle, i);
    }else{

    }


    pose.pose.position.x = start.pose.position.x + x_increment * i + x_sin_increment;
    pose.pose.position.y = start.pose.position.y + y_increment * i + y_cos_increment;

    RCLCPP_WARN(
      node_->get_logger(), "Pose X=%f, Y=%f, index=%i", pose.pose.position.x, pose.pose.position.y, i);

    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  global_path.poses.push_back(goal);

        RCLCPP_WARN(
      node_->get_logger(), ">>>>>>>>>>> Plann WAVY -/-/-/- line DONE");

  return global_path;
}

}  // namespace nav2_wavyline_planner_namespace_name

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_wavyline_planner_namespace_name::WavyLine, nav2_core::GlobalPlanner)
