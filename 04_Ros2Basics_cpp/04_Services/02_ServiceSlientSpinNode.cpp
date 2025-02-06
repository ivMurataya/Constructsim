#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_srvs/srv/empty.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;

//we have now packaged the client code inside a clas
class ServiceClient : public rclcpp::Node {
private:
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;
  bool service_called_ = false;

//Inside our class, we have added a timer method that does a couple of things. 
//First, it checks if the service has been already called or not using the service_called_ flag.
  void timer_callback() {
    if (!service_called_) {
      RCLCPP_INFO(this->get_logger(), "Send Async Request");
      send_async_request();
    } else {
      RCLCPP_INFO(this->get_logger(), "Timer Callback Executed");
    }
  }

  void send_async_request() {
  //Then we have the send_async_request() method. Here we do several things. First we check if the service is available or not:
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

//If the service is available, it will send a request to it and set the service_called_ flag to true
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    //As you can see, in this case we have specified in the request a callback function for handling the service response
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;

    // Now check for the response after a timeout of 1 second
    auto status = result_future.wait_for(1s);
/*An asynchronous client will immediately return future, a value that indicates whether 
the call and response is finished (not the value of the response itself), 
after sending a request to a service.

Therefore, you can check this future to see if there’s a response or not from the service*/
    if (status != std::future_status::ready) {

      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }


/*This method will be called when the service returns a response. 
So here, we just print a log showing the result and set the service_done_ flag to true. 
As we are using an Empty interface for this service, we don't need to get the response 
(as there is no response value). However, it is shown in the comments how that would be done*/
  void
  response_callback(rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {

    // Get response value
    // auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: success");
    service_done_ = true;
  }

public:
  ServiceClient() : Node("service_client") {
    client_ = this->create_client<std_srvs::srv::Empty>("moving");
    timer_ = this->create_wall_timer(
        1s, std::bind(&ServiceClient::timer_callback, this));
  }
//he method that defines when the service has been completed is a simple boolean that check the value of service_done_
  bool is_service_done() const { return this->service_done_; }
};


/*As you can see, in this case we are not using the spin_until_future_complete()
 method at all. This is because our node is already spinning. 
 Therefore, we don't to spin the node until the result is complete,
  but just to check if the result is complete.*/
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
