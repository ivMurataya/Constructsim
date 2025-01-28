#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

//First, define your class, which inherits from the rclcpp::Node class.
class SimplePublisher : public rclcpp::Node
{
public:
//Next, have the constructor of your class:
  SimplePublisher()
  : Node("simple_publisher"), count_(0) //Within the constructor, initialize your node by calling the constructor of the superclass node, and also initialize a variable named count_ to 0.
  {
  /*Also, create your publisher_ and timer_ objects within the constructor. 
  As you will see later, they are created in the private section of your class, 
  as shared pointers to these objects. Note that the timer object is bound to a 
  function named timer_callback, which you will see next. This timer object will be triggered every 500ms.*/
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

/*In the private section, you have the definition of the timer_callback function
 introduced before. Inside this function, you create an Int32 message, given the 
 value of the count_ variable. Then, you will increase the value of the count variable 
 in 1, and you will publish the message into your Topic. Remember that this 
 function will be called every 500ms, as defined in the timer_ object.*/
private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
  /*Also, in the private section, you are creating the shared pointers to your Publisher and timer objects defined above, and you are also creating the variable count.*/
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

/*Finally, create a SimplePublisher object on the main function, and make it spin until somebody terminates the program (Ctrl+C).*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
