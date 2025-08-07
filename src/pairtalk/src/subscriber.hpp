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

class Talker : public rclcpp::Node
{
public:
  std::string mouth_topic;
  std::string ear_topic;
  Talker(std::string name, std::string mouthTopic, std::string earTopic, bool shouldStart) : Node(name), mouth_topic(mouthTopic), ear_topic(earTopic)
  {
    RCLCPP_INFO(this->get_logger(), "Creating Talker node with mouth topic '%s' and ear topic '%s'", mouth_topic.c_str(), ear_topic.c_str());

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      ear_topic, 10,
      std::bind(&Talker::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::String>(mouth_topic, 10);

    if (shouldStart)
    {
      RCLCPP_INFO(this->get_logger(), "Starting Talker node on topic %s", mouth_topic.c_str());
      respond();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Talker node initialized but not started.");
    }
  }

  void respond()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    publisher_->publish(message);
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    little_sleep(5ms);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_ = 0;
}; 