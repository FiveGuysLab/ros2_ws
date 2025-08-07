#include "scalability_priority_test_node/scalable_publisher.hpp"

ScalablePublisher::ScalablePublisher(int num_publishers, std::chrono::milliseconds publish_interval)
  : Node("scalable_publisher")
  , count_(0)
  , num_publishers_(num_publishers)
  , publish_interval_(publish_interval)
{
  RCLCPP_INFO(this->get_logger(), "Initializing ScalablePublisher with %d publishers", num_publishers_);
  
  // Create multiple publishers
  publishers_.reserve(num_publishers_);
  for (int i = 0; i < num_publishers_; ++i) {
    std::string topic_name = "scalability_topic_" + std::to_string(i);
    auto publisher = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    publishers_.push_back(publisher);
    RCLCPP_INFO(this->get_logger(), "Created publisher %d for topic: %s", i, topic_name.c_str());
  }
  
  // Create timer for publishing
  timer_ = this->create_wall_timer(
    publish_interval_, std::bind(&ScalablePublisher::timer_callback, this));
  
  RCLCPP_INFO(this->get_logger(), "ScalablePublisher initialized - publishing every %ldms", 
              publish_interval_.count());
}

void ScalablePublisher::timer_callback()
{
  // Publish to all publishers
  for (int i = 0; i < num_publishers_; ++i) {
    publish_message(i);
  }
  
  // Print periodic info
  if (count_ % 100 == 0 && count_ > 0) {
    RCLCPP_INFO(this->get_logger(), "Published %zu cycles, publishers: %d", count_, num_publishers_);
  }
  
  count_++;
}

void ScalablePublisher::publish_message(int publisher_id)
{
  if (publisher_id < 0 || publisher_id >= static_cast<int>(publishers_.size())) {
    RCLCPP_ERROR(this->get_logger(), "Invalid publisher ID: %d", publisher_id);
    return;
  }
  
  auto message = std_msgs::msg::String();
  message.data = "ScalabilityTest_Pub" + std::to_string(publisher_id) + "_Msg" + std::to_string(count_);
  
  publishers_[publisher_id]->publish(message);
  
  // Only log for first publisher to avoid log spam
  if (publisher_id == 0) {
    RCLCPP_DEBUG(this->get_logger(), "Published: '%s'", message.data.c_str());
  }
} 