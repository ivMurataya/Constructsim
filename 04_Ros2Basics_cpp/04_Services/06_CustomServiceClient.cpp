#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "custom_interfaces/srv/my_custom_service_message.hpp"


#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;
using MyCustomServiceMessage  = custom_interfaces::srv::MyCustomServiceMessage;


class ServiceClient : public rclcpp::Node {
private:
  rclcpp::Client<MyCustomServiceMessage>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;

  void timer_callback() {
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
  }

  void send_async_request() {
  
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }


    auto request = std::make_shared<MyCustomServiceMessage::Request>();
    request->move = "LEFT";  
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;


    auto status = result_future.wait_for(1s);

    if (status != std::future_status::ready) {

      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }


  void
  response_callback(rclcpp::Client<MyCustomServiceMessage>::SharedFuture future) {

    // Get response value
    auto response = future.get();
    bool a = response->success ;
    
    RCLCPP_INFO(this->get_logger(), "Response: success '%s'", a ? "true" : "false");

    service_done_ = true;
  }

public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<MyCustomServiceMessage>("rotate");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
  }

  bool is_service_done() const { return this->service_done_; }
};



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto service_client = std::make_shared<ServiceClient>();
  //As you can see, in this case we are not using the spin_until_future_complete() method at all. This is because our node is already spinning. Therefore, we don't to spin the node until the result is complete, but just to check if the result is complete.
  while (!service_client->is_service_done()) {
    rclcpp::spin_some(service_client);
  }

  rclcpp::shutdown();
  return 0;
}
