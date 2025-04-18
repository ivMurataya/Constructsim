#include "custom_nav2_costmap_plugin/my_custom_costmap.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace my_custom_costmap_namespace_name
{

MyCustomCostmap::MyCustomCostmap()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max())
{
}

// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void
MyCustomCostmap::onInitialize()
{
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  declareParameter("x_oscilation_height", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "x_oscilation_height", x_oscilation_height);

  declareParameter("y_oscilation_range", rclcpp::ParameterValue(10.0));
  node->get_parameter(name_ + "." + "y_oscilation_range", y_oscilation_range);

  declareParameter("obstacle_radius", rclcpp::ParameterValue(30));
  node->get_parameter(name_ + "." + "obstacle_radius", obstacle_radius);

  need_recalculation_ = false;
  current_ = true;
}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void
MyCustomCostmap::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y)
{

  last_min_x_ = *min_x;
  last_min_y_ = *min_y;
  last_max_x_ = *max_x;
  last_max_y_ = *max_y;
  *min_x = -std::numeric_limits<float>::max();
  *min_y = -std::numeric_limits<float>::max();
  *max_x = std::numeric_limits<float>::max();
  *max_y = std::numeric_limits<float>::max();

  need_recalculation_ = true;

}

// The method is called when footprint was changed.
// Here it resets need_recalculation_ variable.
void
MyCustomCostmap::onFootprintChanged()
{
  need_recalculation_ = true;

  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "MyCustomCostmap::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void
MyCustomCostmap::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  if (!enabled_) {
    return;
  }

  // master_array - is a direct pointer to the resulting master_grid.
  // master_grid - is a resulting costmap combined from all layers.
  // By using this pointer all layers will be overwritten!
  // To work with costmap layer and merge it with other costmap layers,
  // please use costmap_ pointer instead (this is pointer to current
  // costmap layer grid) and then call one of updates methods:
  // - updateWithAddition()
  // - updateWithMax()
  // - updateWithOverwrite()
  // - updateWithTrueOverwrite()
  // In this case, using master_array pointer is equal to modifying local costmap_
  // pointer and then calling updateWithTrueOverwrite():
  unsigned char * master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
  // These variables are used to update the costmap only within this window
  // avoiding the updates of whole area.
  //
  // Fixing window coordinates with map size if necessary.
  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);



  // You calculate the new position of circle
  circle_p_x = x_oscilation_height;

  // Calculate center
  circle_p_y = (max_j - min_j)/2;

  if (delta_flag){
    circle_p_y_delta += 10;
  }else{
    circle_p_y_delta -= 10;
  }

  circle_p_y += circle_p_y_delta;
    
  if (circle_p_y >= (((max_j - min_j)/2) + y_oscilation_range)){
    delta_flag = false;
  }else if (circle_p_y <= (((max_j - min_j)/2) - y_oscilation_range))
  {
    delta_flag = true;
  }

  RCLCPP_WARN(rclcpp::get_logger(
  "nav2_costmap_2d"), "circle_p_x = %f", circle_p_x);
  RCLCPP_WARN(rclcpp::get_logger(
  "nav2_costmap_2d"), "circle_p_y = %f", circle_p_y);


  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
      int index = master_grid.getIndex(i, j);

      bool result = pointInsideCircle(i, j);

      if (result){
        master_array[index] = 200;
      }else{
        master_array[index] = 1;
      }
      
    }
  }



  
}

bool
MyCustomCostmap::pointInsideCircle(int x_point, int y_point)
{

  int delta_x = abs(circle_p_x - x_point);
  int delta_y = abs(circle_p_y - y_point);
  bool delta_x_in = (delta_x <= obstacle_radius);
  bool delta_y_in = (delta_y <= obstacle_radius);
  bool inside_circle = (delta_x_in && delta_y_in);

  // RCLCPP_WARN(rclcpp::get_logger(
  //     "nav2_costmap_2d"), "Point Inside Cicle = %d", inside_circle);

  return inside_circle;
}

}  // namespace my_custom_costmap_namespace_name

// This is the macro allowing a my_custom_costmap_namespace_name::MyCustomCostmap class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_custom_costmap_namespace_name::MyCustomCostmap, nav2_costmap_2d::Layer)
