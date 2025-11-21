#pragma once

#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "preemptive_executor/preemptive_executor.hpp"
#include "preemptive_executor/callback_registry.hpp"

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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  BusywaitSubscriber() : Node("busywait_subscriber"), last_message_received_(false)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "busywait_topic", 10,
      std::bind(&BusywaitSubscriber::topic_callback, this, std::placeholders::_1));
      preemptive_executor::CallbackRegistry::register_callback_name(preemptive_executor::CallbackEntity(
        subscription_), "subscriber");
      RCLCPP_INFO(this->get_logger(), "BusywaitSubscriber initialized - busy waiting every 5ms");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Store the latest message and mark that we received one
    latest_message_ = msg->data;
    last_message_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());

    little_sleep(7ms);
  }

  std::string latest_message_;
  bool last_message_received_;
}; 