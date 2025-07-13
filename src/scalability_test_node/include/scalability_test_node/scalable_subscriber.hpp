#pragma once

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ScalableSubscriber : public rclcpp::Node
{
public:
  ScalableSubscriber(int num_subscribers = 1, bool enable_processing = false);
  
  // Statistics getters
  size_t get_total_messages_received() const;
  std::vector<size_t> get_per_subscriber_counts() const;

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg, int subscriber_id);
  void minimal_processing(const std::string& data);

  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  std::vector<size_t> message_counts_;
  mutable std::mutex counts_mutex_;
  int num_subscribers_;
  bool enable_processing_;
  std::atomic<size_t> total_messages_received_;
}; 