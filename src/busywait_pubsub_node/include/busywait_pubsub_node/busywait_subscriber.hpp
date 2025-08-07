#pragma once

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

void little_sleep(std::chrono::milliseconds ms)
{
    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + ms;
    do
    {
        std::this_thread::yield();
    }
    while (std::chrono::high_resolution_clock::now() < end);
}

class BusywaitSubscriber : public rclcpp::Node
{
public:
  BusywaitSubscriber() : Node("busywait_subscriber"), last_message_received_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "busywait_topic", 10,
      std::bind(&BusywaitSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "BusywaitSubscriber initialized - busy waiting every 5ms");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Store the latest message and mark that we received one
    latest_message_ = msg->data;
    last_message_received_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received message: '%s'", msg->data.c_str());

    little_sleep(40ms);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string latest_message_;
  bool last_message_received_;
}; 