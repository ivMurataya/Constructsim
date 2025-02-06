#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <memory>

using Empty = std_srvs::srv::Empty;
using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node
{
public:
  ServerNode()
  : Node("service_rotate")
  {

    srv_ = create_service<SetBool>("rotate", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp::Service<SetBool>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  void moving_callback(
      const std::shared_ptr<SetBool::Request> request,
      const std::shared_ptr<SetBool::Response>
          response) 
    {

        auto message = geometry_msgs::msg::Twist();
        if (request->data == true){
            message.linear.x = 0.2;
            message.angular.z = 0.2;
            publisher_->publish(message);
            response->success = true;
            response->message = "Robot Rotating";
        }
        else if (request->data == false) {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
            publisher_->publish(message);
            response->success = true;
            response->message = "Robot Stop";
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
