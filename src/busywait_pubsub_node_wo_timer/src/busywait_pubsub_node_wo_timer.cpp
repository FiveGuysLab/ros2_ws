#include "busywait_pubsub_node_wo_timer/busywait_publisher_node_wo_timer.hpp"
#include "busywait_pubsub_node_wo_timer/busywait_subscriber_node_wo_timer.hpp"

#include <rclcpp/rclcpp.hpp>
// Comment out the standard executor include
// #include <rclcpp/executors/single_threaded_executor.hpp>
// Add the custom executor include
#include "priority_executor/default_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<BusywaitPublisher>();
  auto subscriber_node = std::make_shared<BusywaitSubscriber>();

  // Comment out the standard executor
  // rclcpp::executors::SingleThreadedExecutor executor;
  // Use the custom executor instead
  ROSDefaultExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting busywait pubsub nodes...");
  RCLCPP_INFO(rclcpp::get_logger("main"), "Publisher: 1ms interval, Subscriber: 5ms busy wait");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
