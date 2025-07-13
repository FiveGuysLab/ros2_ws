#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class BusywaitPublisher : public rclcpp::Node
{
public:
  BusywaitPublisher() : Node("busywait_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("busywait_topic", 10);
    // Publish every 1ms - this will be our baseline publishing rate
    timer_ = this->create_wall_timer(
      1ms, std::bind(&BusywaitPublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "BusywaitPublisher initialized - publishing every 1ms");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "BusyWait Message #" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
}; 