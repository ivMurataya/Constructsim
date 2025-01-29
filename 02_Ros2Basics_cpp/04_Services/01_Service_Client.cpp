#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_client");
  
  //This is the line where you create the Client:
  //You can see that you create a Client that uses the Empty Service type and connects to a Service named /stop
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =
    node->create_client<std_srvs::srv::Empty>("stop");

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    //You can see that you create a Client that uses the Empty Service type and connects to a Service named /moving
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

/*Send an asynchronous request to the Service Server using the async_send_request() method. 
Then, store the response from the Server in the variable result_future. 
This result_future variable will contains what is known as a future object. 
After making the request, the Server will immediately return result_future, 
which indicates whether the call and response are finished (but it does not
 contain the value of the response itself). You will read more about 
 this in the next section of this chapter.*/

  auto result_future = client->async_send_request(request);
  // Wait for the result.
//   /Finally, you spin the node until this result_future is completed (the service is completed):
  if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
  //If it is finished successfully, you then get the value of the Server response and print a message indicating that the robot is moving:
    auto result = result_future.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot is moving");
  } else {
  //If it is finished successfully, you then get the value of the Server response and print a message indicating that the robot is moving:
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /moving");
  }

  rclcpp::shutdown();
  return 0;
}
