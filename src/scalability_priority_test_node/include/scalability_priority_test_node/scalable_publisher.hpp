#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class ScalablePublisher : public rclcpp::Node
{
public:
  ScalablePublisher(int num_publishers = 1, std::chrono::milliseconds publish_interval = 1ms);
  std::vector<rclcpp::TimerBase::SharedPtr> get_timers() const { return {timer_}; }

private:
  void timer_callback();
  void publish_message(int publisher_id);

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  int num_publishers_;
  std::chrono::milliseconds publish_interval_;
}; 