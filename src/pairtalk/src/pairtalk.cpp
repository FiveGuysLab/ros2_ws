#include "subscriber.hpp"

#include "rclcpp/rclcpp.hpp"
// Comment out the standard executor include
// #include <rclcpp/executors/single_threaded_executor.hpp>
// Add the custom executor include
#include "priority_executor/default_executor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<Talker>("publisher", "chanA", "chanB", true);
  auto subscriber_node = std::make_shared<Talker>("subscriber", "chanB", "chanA", false);

  ROSDefaultExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting nodes...");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
