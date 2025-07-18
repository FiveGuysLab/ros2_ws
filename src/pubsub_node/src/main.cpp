#include "../include/minimal_publisher.hpp"
#include "../include/minimal_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
// Comment out the standard executor include
// #include <rclcpp/executors/single_threaded_executor.hpp>
// Add the custom executor include
#include "priority_executor/default_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  // Comment out the standard executor
  // rclcpp::executors::SingleThreadedExecutor executor;
  // Use the custom executor instead
  ROSDefaultExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
