#include "../include/busywait_pubsub_node/busywait_publisher.hpp"
#include "../include/busywait_pubsub_node/busywait_subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
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

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
