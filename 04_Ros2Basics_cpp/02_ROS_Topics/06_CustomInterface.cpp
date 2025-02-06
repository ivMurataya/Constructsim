#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/age.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class PublishAge : public rclcpp::Node
{
public:
  PublishAge()
  : Node("move_robot")
  {
    publisher_ = this->create_publisher<custom_interfaces::msg::Age>("age", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PublishAge::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = custom_interfaces::msg::Age();
    message.years = 4;
    message.months = 11;
    message.days = 21;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_interfaces::msg::Age>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishAge>());
  rclcpp::shutdown();
  return 0;
}
/*
--------------------- CREATE CUSTOM INTERFACE ---------------------------------------
To create a new message, perform the following steps:
Create a directory named 'msg' inside your package
Inside this directory, create a file named Name_of_your_message.msg (more information below)
Modify the CMakeLists.txt file (more information below)
Modify package.xml file (more information below)
Compile and source
Use in code


ros2 pkg create --build-type ament_cmake custom_interfaces --dependencies rclcpp std_msgs
cd ~/ros2_ws/src/custom_interfaces
mkdir msg


The Age.msg file must contain this:
float32 years
float32 months
float32 days


Edit two functions inside CMakeLists.txt:
find_package()
rosidl_generate_interfaces()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
)


Modify package.xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>


ros2 interface show custom_interfaces/msg/Age


*/
/*
-------------------- USE CUSTOM INTERFACES ----------------------------------------------
Add to your CMakeLists.txt the following extra lines:

find_package(custom_interfaces REQUIRED) # This is the package that contains the custom interface

add_executable(age_publisher_node src/publish_age.cpp)
ament_target_dependencies(age_publisher_node rclcpp std_msgs custom_interfaces) # Note that we are also adding the package which contains the custom interface as a dependency of the node that will be using this custom interface

install(TARGETS
   age_publisher_node
   DESTINATION lib/${PROJECT_NAME}
 )


package.xml file
<depend>custom_interfaces</depend>


*/

