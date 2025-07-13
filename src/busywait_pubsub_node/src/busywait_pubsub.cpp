#include "../include/busywait_pubsub_node/busywait_publisher.hpp"
#include "../include/busywait_pubsub_node/busywait_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<BusywaitPublisher>();
  auto subscriber_node = std::make_shared<BusywaitSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting busywait pubsub nodes...");
  RCLCPP_INFO(rclcpp::get_logger("main"), "Publisher: 1ms interval, Subscriber: 5ms busy wait");

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
