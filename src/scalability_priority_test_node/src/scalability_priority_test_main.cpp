#include "scalability_priority_test_node/scalable_subscriber.hpp"
#include "scalability_priority_test_node/scalable_publisher.hpp"

#include "rclcpp/rclcpp.hpp"
#include "priority_executor/priority_executor.hpp"
#include "priority_executor/priority_memory_strategy.hpp"
#include <iostream>
#include <csignal>
#include <memory>
#include <thread>

std::shared_ptr<ScalableSubscriber> g_subscriber;
std::shared_ptr<ScalablePublisher> g_publisher;

void print_usage(const char* program_name)
{
  std::cout << "Usage: " << program_name << " [options]\n"
            << "Options:\n"
            << "  --publishers <num>     Number of publishers (default: 1)\n"
            << "  --subscribers <num>    Number of subscribers (default: 5)\n"
            << "  --interval <ms>        Publishing interval in milliseconds (default: 100)\n"
            << "  --processing           Enable processing in subscribers (default: false)\n"
            << "  --duration <sec>       Test duration in seconds (default: infinite - run until Ctrl+C)\n"
            << "  --help                 Show this help message\n"
            << std::endl;
}

int main(int argc, char * argv[])
{
  int num_publishers = 1;
  int num_subscribers = 5;
  int publish_interval_ms = 100;
  bool enable_processing = false;
  int test_duration_sec = -1;
  
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") {
      print_usage(argv[0]);
      return 0;
    } else if (arg == "--publishers" && i + 1 < argc) {
      num_publishers = std::atoi(argv[++i]);
    } else if (arg == "--subscribers" && i + 1 < argc) {
      num_subscribers = std::atoi(argv[++i]);
    } else if (arg == "--interval" && i + 1 < argc) {
      publish_interval_ms = std::atoi(argv[++i]);
    } else if (arg == "--duration" && i + 1 < argc) {
      test_duration_sec = std::atoi(argv[++i]);
    } else if (arg == "--processing") {
      enable_processing = true;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      print_usage(argv[0]);
      return 1;
    }
  }
  
  if (num_publishers <= 0 || num_subscribers <= 0 || publish_interval_ms <= 0) {
    std::cerr << "Error: Publishers, subscribers, and interval must be positive." << std::endl;
    return 1;
  }
  if (test_duration_sec == 0) {
    std::cerr << "Error: Duration must be positive or omitted for infinite duration." << std::endl;
    return 1;
  }
  
  rclcpp::init(argc, argv);
  
  g_publisher = std::make_shared<ScalablePublisher>(num_publishers, std::chrono::milliseconds(publish_interval_ms));
  g_subscriber = std::make_shared<ScalableSubscriber>(num_subscribers, enable_processing);
  
  // Set up the priority executor with priority memory strategy
  rclcpp::ExecutorOptions options;
  auto strategy = std::make_shared<PriorityMemoryStrategy<>>();
  options.memory_strategy = strategy;
  auto executor = std::make_unique<timed_executor::TimedExecutor>(options, "scalability_priority_executor");
  executor->prio_memory_strategy_ = strategy;

  // Set priorities for all publisher timers
  for (const auto& timer : g_publisher->get_timers()) {
    strategy->set_executable_deadline(
      timer->get_timer_handle(),
      publish_interval_ms * 1000, // microseconds
      TIMER,
      0 // chain_id
    );
    strategy->get_priority_settings(timer->get_timer_handle())->timer_handle = timer;
    strategy->set_first_in_chain(timer->get_timer_handle());
  }

  // Set static priorities for all subscriber subscriptions
  int high_priority = 2;
  int low_priority = 1;
  int num_high = num_subscribers / 4; // 25% get high priority
  for (size_t i = 0; i < g_subscriber->get_subscriptions().size(); ++i) {
    auto sub = g_subscriber->get_subscriptions()[i];
    int prio = (i < static_cast<size_t>(num_high)) ? high_priority : low_priority;
    strategy->set_executable_priority(
      sub->get_subscription_handle(),
      prio,
      SUBSCRIPTION
    );
    strategy->set_last_in_chain(sub->get_subscription_handle());
  }

  executor->add_node(g_publisher);
  executor->add_node(g_subscriber);
  
  std::cout << "\n=== Scalability Priority Test Configuration ===" << std::endl;
  std::cout << "Publishers: " << num_publishers << std::endl;
  std::cout << "Subscribers: " << num_subscribers << std::endl;
  std::cout << "Publishing interval: " << publish_interval_ms << " ms" << std::endl;
  std::cout << "Processing enabled: " << (enable_processing ? "true" : "false") << std::endl;
  if (test_duration_sec > 0) {
    std::cout << "Test duration: " << test_duration_sec << " seconds" << std::endl;
  } else {
    std::cout << "Test duration: infinite (press Ctrl+C to stop)" << std::endl;
  }
  std::cout << "===============================================" << std::endl;
  
  RCLCPP_INFO(rclcpp::get_logger("scalability_priority_test"), "Starting scalability priority test...");
  
  auto start_time = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point end_time;
  bool has_duration = test_duration_sec > 0;

  executor->spin();

  auto final_time = std::chrono::high_resolution_clock::now();
  auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(final_time - start_time);
  std::cout << "\n=== Final Test Results ===" << std::endl;
  std::cout << "Total runtime: " << total_elapsed.count() << " seconds" << std::endl;
  std::cout << "Total messages received: " << g_subscriber->get_total_messages_received() << std::endl;
  auto per_subscriber_counts = g_subscriber->get_per_subscriber_counts();
  std::cout << "Per-subscriber message counts:" << std::endl;
  for (int i = 0; i < static_cast<int>(per_subscriber_counts.size()); ++i) {
    std::cout << "  Subscriber " << i << ": " << per_subscriber_counts[i] << " messages" << std::endl;
  }
  std::cout << "Cleaning up..." << std::endl;
  std::cout << "Canceling executor..." << std::endl;
  executor->cancel();
  std::cout << "Removing nodes from executor..." << std::endl;
  executor->remove_node(g_publisher);
  executor->remove_node(g_subscriber);
  std::cout << "Resetting publisher..." << std::endl;
  g_publisher.reset();
  std::cout << "Resetting subscriber..." << std::endl;
  g_subscriber.reset();
  std::cout << "Waiting for cleanup to complete..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Shutting down ROS..." << std::endl;
  rclcpp::shutdown();
  std::cout << "Cleanup complete." << std::endl;
  return 0;
} 