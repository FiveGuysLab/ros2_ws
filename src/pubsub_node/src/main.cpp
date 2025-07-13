#include "../include/minimal_publisher.hpp"
#include "../include/minimal_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<MinimalPublisher>();
  auto subscriber_node = std::make_shared<MinimalSubscriber>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
