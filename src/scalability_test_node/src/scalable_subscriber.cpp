#include "../include/scalability_test_node/scalable_subscriber.hpp"

ScalableSubscriber::ScalableSubscriber(int num_subscribers, bool enable_processing)
  : Node("scalable_subscriber")
  , num_subscribers_(num_subscribers)
  , enable_processing_(enable_processing)
  , total_messages_received_(0)
{
  RCLCPP_INFO(this->get_logger(), "Initializing ScalableSubscriber with %d subscribers", num_subscribers_);
  
  // Initialize message count tracking
  message_counts_.resize(num_subscribers_, 0);
  
  // Create multiple subscribers
  subscriptions_.reserve(num_subscribers_);
  for (int i = 0; i < num_subscribers_; ++i) {
    std::string topic_name = "scalability_topic_" + std::to_string(i);
    
    auto subscription = this->create_subscription<std_msgs::msg::String>(
      topic_name, 10,
      [this, i](const std_msgs::msg::String::SharedPtr msg) {
        this->topic_callback(msg, i);
      });
    
    subscriptions_.push_back(subscription);
    RCLCPP_INFO(this->get_logger(), "Created subscriber %d for topic: %s", i, topic_name.c_str());
  }
  
  RCLCPP_INFO(this->get_logger(), "ScalableSubscriber initialized - processing enabled: %s", 
              enable_processing_ ? "true" : "false");
}

void ScalableSubscriber::topic_callback(const std_msgs::msg::String::SharedPtr msg, int subscriber_id)
{
  // Update message counts with mutex protection
  {
    std::lock_guard<std::mutex> lock(counts_mutex_);
    message_counts_[subscriber_id]++;
  }
  total_messages_received_++;
  
  // Extract publisher ID from message data
  // Message format: "ScalabilityTest_Pub{id}_Msg{count}"
  std::string data = msg->data;
  int publisher_id = -1;
  size_t pub_pos = data.find("_Pub");
  if (pub_pos != std::string::npos) {
    size_t id_start = pub_pos + 4; // Skip "_Pub"
    size_t id_end = data.find("_", id_start);
    if (id_end != std::string::npos) {
      std::string pub_id_str = data.substr(id_start, id_end - id_start);
      publisher_id = std::stoi(pub_id_str);
    }
  }
  
  // Print detailed message information
  size_t current_count;
  {
    std::lock_guard<std::mutex> lock(counts_mutex_);
    current_count = message_counts_[subscriber_id];
  }
  
  if (publisher_id >= 0) {
    RCLCPP_INFO(this->get_logger(), 
                "Subscriber_%d received message from Publisher_%d (message count: %zu) - Content: '%s'",
                subscriber_id, publisher_id, current_count, msg->data.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), 
                "Subscriber_%d received message (message count: %zu) - Content: '%s'",
                subscriber_id, current_count, msg->data.c_str());
  }
  
  // Perform minimal processing if enabled
  if (enable_processing_) {
    minimal_processing(msg->data);
  }
  
  // Print periodic total statistics (less frequently to avoid spam)
  if (total_messages_received_ % 10 == 0) {
    RCLCPP_INFO(this->get_logger(), 
                "=== Total messages received across all subscribers: %zu ===", 
                total_messages_received_.load());
  }
}

void ScalableSubscriber::minimal_processing(const std::string& data)
{
  // Simulate minimal processing work
  volatile size_t hash = 0;
  for (char c : data) {
    hash = hash * 31 + c;
  }
  
  // Prevent optimization from removing the computation
  (void)hash;
}

size_t ScalableSubscriber::get_total_messages_received() const
{
  return total_messages_received_.load();
}

std::vector<size_t> ScalableSubscriber::get_per_subscriber_counts() const
{
  std::lock_guard<std::mutex> lock(counts_mutex_);
  return message_counts_;
} 