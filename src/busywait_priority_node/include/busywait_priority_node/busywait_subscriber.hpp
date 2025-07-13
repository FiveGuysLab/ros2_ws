#pragma once

#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class BusywaitSubscriber : public rclcpp::Node
{
public:
  BusywaitSubscriber() : Node("busywait_subscriber"), last_message_received_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "busywait_topic", 10,
      std::bind(&BusywaitSubscriber::topic_callback, this, std::placeholders::_1));
    
    // Busy wait timer - set to 5ms (longer than publisher's 1ms period)
    // This ensures we always have a message available when we check
    timer_ = this->create_wall_timer(
      5ms, std::bind(&BusywaitSubscriber::busywait_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "BusywaitSubscriber initialized - busy waiting every 5ms");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Store the latest message and mark that we received one
    latest_message_ = msg->data;
    last_message_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received message: '%s'", msg->data.c_str());
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
  rclcpp::TimerBase::SharedPtr timer_;
  std::string latest_message_;
  bool last_message_received_;
}; 