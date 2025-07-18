#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class BusywaitPublisher : public rclcpp::Node
{
public:
  BusywaitPublisher() : Node("busywait_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("busywait_topic", 10);
    
    // Service to trigger publishing - replaces wall timer
    service_ = this->create_service<std_srvs::srv::Trigger>(
      "trigger_publish",
      std::bind(&BusywaitPublisher::service_callback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "BusywaitPublisher initialized - publishing via service calls");
  }

private:
  void service_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    (void)request;  // Suppress unused parameter warning
    auto message = std_msgs::msg::String();
    message.data = "BusyWait Message #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    
    response->success = true;
    response->message = "Published message #" + std::to_string(count_ - 1);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  size_t count_;
}; 