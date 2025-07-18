#pragma once

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class BusywaitSubscriber : public rclcpp::Node
{
public:
  BusywaitSubscriber() : Node("busywait_subscriber"), last_message_received_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "busywait_topic", 10,
      std::bind(&BusywaitSubscriber::topic_callback, this, std::placeholders::_1));
    
    // Service client to trigger publisher - replaces wall timer
    service_client_ = this->create_client<std_srvs::srv::Trigger>("trigger_publish");
    
    // Wait for service to be available
    while (!service_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for trigger_publish service...");
    }
    
    // Timer to call service every 1ms - ensures publisher publishes every 1ms
    service_timer_ = this->create_wall_timer(
      1ms, std::bind(&BusywaitSubscriber::service_callback, this));
    
    // Busy wait timer - set to 5ms (longer than service calls)
    // This ensures we always have a message available when we check
    busywait_timer_ = this->create_wall_timer(
      5ms, std::bind(&BusywaitSubscriber::busywait_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "BusywaitSubscriber initialized - service calls every 1ms, busy waiting every 5ms");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Store the latest message and mark that we received one
    latest_message_ = msg->data;
    last_message_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }

  void service_callback()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    auto future = service_client_->async_send_request(request);
    
    // Quick check for service response
    if (future.wait_for(1ms) == std::future_status::ready) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_DEBUG(this->get_logger(), "Service call successful: %s", response->message.c_str());
      }
    }
  }

  void busywait_callback()
  {
    // Busy wait behavior - always check for messages at our specified interval
    if (last_message_received_) {
      RCLCPP_INFO(this->get_logger(), "Busy wait found message: '%s'", latest_message_.c_str());
      // Reset the flag to track new messages
      last_message_received_ = false;
    } else {
      RCLCPP_WARN(this->get_logger(), "Busy wait - no new message available (this shouldn't happen often with our timing)");
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr service_client_;
  rclcpp::TimerBase::SharedPtr service_timer_;
  rclcpp::TimerBase::SharedPtr busywait_timer_;
  std::string latest_message_;
  bool last_message_received_;
}; 