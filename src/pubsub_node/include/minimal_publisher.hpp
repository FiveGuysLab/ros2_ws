#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "pubsub_node/msg/timed_message.hpp"
#include <uuid/uuid.h>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<pubsub_node::msg::TimedMessage>("topic", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&MinimalPublisher::timer_callback, this));
      
    // Open the output file
    output_file_.open("publisher_timestamps.csv");
    output_file_ << "uuid,timestamp_sec,timestamp_nanosec" << std::endl;
  }

  ~MinimalPublisher()
  {
    if (output_file_.is_open()) {
      output_file_.close();
    }
  }

private:
  std::string generate_uuid() {
    uuid_t uuid;
    uuid_generate(uuid);
    char uuid_str[37];
    uuid_unparse(uuid, uuid_str);
    return std::string(uuid_str);
  }

  void timer_callback()
  {
    auto message = pubsub_node::msg::TimedMessage();
    message.uuid = generate_uuid();
    message.timestamp = this->now();
    message.data = "Hello, world! " + std::to_string(count_++);
    
    // Write to file
    output_file_ << message.uuid << ","
                 << message.timestamp.sec << ","
                 << message.timestamp.nanosec << std::endl;
    
    RCLCPP_INFO(this->get_logger(), "Publishing: UUID=%s, timestamp=%d.%09d", 
                message.uuid.c_str(),
                message.timestamp.sec,
                message.timestamp.nanosec);
                
    publisher_->publish(message);
  }

  rclcpp::Publisher<pubsub_node::msg::TimedMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
  std::ofstream output_file_;
};
