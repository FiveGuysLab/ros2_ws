#include "../include/busywait_priority_node/busywait_publisher.hpp"
#include "../include/busywait_priority_node/busywait_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
// #include "priority_executor/priority_executor.hpp"
// #include "priority_executor/priority_memory_strategy.hpp"
#include "preemptive_executor/preemptive_executor.hpp"
#include "preemptive_executor/rt_memory_strategy.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<BusywaitPublisher>();
  auto subscriber_node = std::make_shared<BusywaitSubscriber>();

  // Set up the priority executor with priority memory strategy
  rclcpp::ExecutorOptions options;
  auto strategy = std::make_shared<preemptive_executor::memory_strategies::allocator_memory_strategy::RTAllocatorMemoryStrategy<>>();
  options.memory_strategy = strategy;
  auto parser = preemptive_executor::ChainYamlParser();
  parser.load_yaml_file("/home/fiveguys/fydp/ros2_ws/src/busywait_priority_node/src/user_chains.yaml");
  parser.parse();
  
  auto executor = std::make_unique<preemptive_executor::PreemptiveExecutor>(options, strategy, parser.get_user_chains());
  
  // Set the priority memory strategy reference
  // executor->prio_memory_strategy_ = strategy;

  // Configure priority settings for publisher timer
  // Set deadline of 1ms (1000 microseconds) for the publisher timer
  // strategy->set_executable_deadline(
  //   publisher_node->timer_->get_timer_handle(), 
  //   1000, // 1ms period in microseconds
  //   TIMER, 
  //   0 // chain_id
  // );
  
  // // Set timer handle for the publisher (required for chain management)
  // strategy->get_priority_settings(publisher_node->timer_->get_timer_handle())
  //   ->timer_handle = publisher_node->timer_;
  
  // // Mark publisher timer as first in chain
  // strategy->set_first_in_chain(publisher_node->timer_->get_timer_handle());

  // // Configure priority settings for subscriber subscription
  // strategy->set_executable_deadline(
  //   subscriber_node->subscription_->get_subscription_handle(),
  //   1000, // Same period as publisher
  //   SUBSCRIPTION,
  //   0 // chain_id
  // );
  
  // // Mark subscriber subscription as last in chain
  // strategy->set_last_in_chain(subscriber_node->subscription_->get_subscription_handle());

  // Add nodes to executor
  executor->add_node(publisher_node);
  executor->add_node(subscriber_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting busywait priority nodes with priority executor...");
  RCLCPP_INFO(rclcpp::get_logger("main"), "Publisher: 1ms interval, Subscriber: 5ms busy wait");
  RCLCPP_INFO(rclcpp::get_logger("main"), "Using single-threaded priority executor");

  // Print priority settings for debugging
  // std::cout << "Publisher Timer Priority Settings:\n" 
  //           << *strategy->get_priority_settings(publisher_node->timer_->get_timer_handle()) 
  //           << std::endl;
  // std::cout << "Subscriber Subscription Priority Settings:\n" 
  //           << *strategy->get_priority_settings(subscriber_node->subscription_->get_subscription_handle()) 
  //           << std::endl;

  executor->spin();

  rclcpp::shutdown();
  return 0;
} 