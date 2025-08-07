#pragma once

#include <memory>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "pubsub_node/msg/timed_message.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<pubsub_node::msg::TimedMessage>(
      "topic", 10,
      std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

    // Open the output file
    output_file_.open("subscriber_timestamps.csv");
    output_file_ << "uuid,pub_timestamp_sec,pub_timestamp_nanosec,sub_timestamp_sec,sub_timestamp_nanosec,latency_sec,latency_nanosec" << std::endl;
  }

  ~MinimalSubscriber()
  {
    if (output_file_.is_open()) {
      output_file_.close();
    }
  }

private:
  void topic_callback(const pubsub_node::msg::TimedMessage::SharedPtr msg) const
  {
    rclcpp::Time now = this->now();
    rclcpp::Duration latency = now - msg->timestamp;
    
    // Write to file - need to const_cast because we're in a const function but need to write to file
    auto* non_const_this = const_cast<MinimalSubscriber*>(this);
    non_const_this->output_file_ << msg->uuid << ","
                                << msg->timestamp.sec << ","
                                << msg->timestamp.nanosec << ","
                                << now.seconds() << ","
                                << now.nanoseconds() << ","
                                << latency.seconds() << ","
                                << latency.nanoseconds() << std::endl;
    
    RCLCPP_INFO(this->get_logger(), 
                "Received message: UUID=%s, pub_timestamp=%d.%09u, sub_timestamp=%.0f.%09lu, latency=%.0f.%09ld", 
                msg->uuid.c_str(),
                msg->timestamp.sec, msg->timestamp.nanosec,
                now.seconds(), now.nanoseconds(),
                latency.seconds(), latency.nanoseconds());
  }

  rclcpp::Subscription<pubsub_node::msg::TimedMessage>::SharedPtr subscription_;
  std::ofstream output_file_;
};
