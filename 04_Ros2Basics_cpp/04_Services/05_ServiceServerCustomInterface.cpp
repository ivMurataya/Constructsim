#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "custom_interfaces/srv/my_custom_service_message.hpp"


#include <memory>

using MyCustomServiceMessage  = custom_interfaces::srv::MyCustomServiceMessage;//std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("service_rotate")
  {

    srv_ = create_service<MyCustomServiceMessage>("rotate", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Service Ready ");
  }

private:
  rclcpp::Service<MyCustomServiceMessage>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(
      const std::shared_ptr<MyCustomServiceMessage::Request> request,
      const std::shared_ptr<MyCustomServiceMessage::Response> response) 
    {
        
        auto message = geometry_msgs::msg::Twist();
        if (request->move == "LEFT"){
            message.linear.x = 0.2;
            message.angular.z = 0.2;
            publisher_->publish(message);
            response->success = true;
            
        }
        else if (request->move == "RIGTH") {
            message.linear.x = 0.2;
            message.angular.z = -0.2;
            publisher_->publish(message);
            response->success = true;
            
        }
        else if (request->move == "STOP") {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            response->success = true;
            
        }
        
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServerNode>());
  rclcpp::shutdown();
  return 0;
}


/*
ros2 pkg create --build-type ament_cmake movement_pkg --dependencies rclcpp custom_interfaces std_msgs geometry_msgs sensor_msgs

find_package(custom_interfaces REQUIRED)

add_executable(rotate_server_node src/movement_server.cpp)
ament_target_dependencies(rotate_server_node rclcpp geometry_msgs custom_interfaces)

install(TARGETS
   rotate_server_node
   DESTINATION lib/${PROJECT_NAME}
 )
 
 <depend>custom_interfaces</depend>
 */
