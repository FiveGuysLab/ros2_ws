#include "../include/scalability_test_node/scalable_publisher.hpp"
#include "../include/scalability_test_node/scalable_subscriber.hpp"

#include <rclcpp/rclcpp.hpp>
// Comment out the standard executor include
// #include <rclcpp/executors/single_threaded_executor.hpp>
// Add the custom executor include
#include "priority_executor/default_executor.hpp"
#include <iostream>
#include <csignal>
#include <memory>
#include <thread>

// Global variables for signal handling
std::shared_ptr<ScalableSubscriber> g_subscriber;
std::shared_ptr<ScalablePublisher> g_publisher;
volatile sig_atomic_t g_shutdown_requested = 0;

void signal_handler(int signal)
{
  (void)signal;
  g_shutdown_requested = 1;
  std::cout << "\nShutdown requested..." << std::endl;
}

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
  // Default parameters
  int num_publishers = 1;
  int num_subscribers = 5;
  int publish_interval_ms = 100;
  bool enable_processing = false;
  int test_duration_sec = -1;  // -1 means infinite duration
  
  // Parse command line arguments
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
  
  // Validate parameters
  if (num_publishers <= 0 || num_subscribers <= 0 || publish_interval_ms <= 0) {
    std::cerr << "Error: Publishers, subscribers, and interval must be positive." << std::endl;
    return 1;
  }
  
  if (test_duration_sec == 0) {
    std::cerr << "Error: Duration must be positive or omitted for infinite duration." << std::endl;
    return 1;
  }
  
  rclcpp::init(argc, argv);
  
  // Set up signal handler
  signal(SIGINT, signal_handler);
  
  // Create nodes
  g_publisher = std::make_shared<ScalablePublisher>(
    num_publishers, std::chrono::milliseconds(publish_interval_ms));
  g_subscriber = std::make_shared<ScalableSubscriber>(num_subscribers, enable_processing);
  
  // Create executor
  // Comment out the standard executor
  // rclcpp::executors::SingleThreadedExecutor executor;
  // Use the custom executor instead
  ROSDefaultExecutor executor;
  executor.add_node(g_publisher);
  executor.add_node(g_subscriber);
  
  // Print configuration
  std::cout << "\n=== Scalability Test Configuration ===" << std::endl;
  std::cout << "Publishers: " << num_publishers << std::endl;
  std::cout << "Subscribers: " << num_subscribers << std::endl;
  std::cout << "Publishing interval: " << publish_interval_ms << " ms" << std::endl;
  std::cout << "Processing enabled: " << (enable_processing ? "true" : "false") << std::endl;
  if (test_duration_sec > 0) {
    std::cout << "Test duration: " << test_duration_sec << " seconds" << std::endl;
  } else {
    std::cout << "Test duration: infinite (press Ctrl+C to stop)" << std::endl;
  }
  std::cout << "=======================================" << std::endl;
  
  RCLCPP_INFO(rclcpp::get_logger("scalability_test"), "Starting scalability test...");
  
  // Run test
  auto start_time = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point end_time;
  bool has_duration = test_duration_sec > 0;
  
  if (has_duration) {
    end_time = start_time + std::chrono::seconds(test_duration_sec);
  }
  
  while (rclcpp::ok() && !g_shutdown_requested) {
    auto current_time = std::chrono::high_resolution_clock::now();
    
    // Check if test duration is reached (only if duration was specified)
    if (has_duration && current_time >= end_time) {
      std::cout << "Test duration reached. Stopping..." << std::endl;
      break;
    }
    
    // Spin the executor with a timeout
    executor.spin_once(std::chrono::milliseconds(10));
  }
  
  // Print final statistics
  auto final_time = std::chrono::high_resolution_clock::now();
  auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(final_time - start_time);
  
  std::cout << "\n=== Final Test Results ===" << std::endl;
  std::cout << "Total runtime: " << total_elapsed.count() << " seconds" << std::endl;
  std::cout << "Total messages received: " << g_subscriber->get_total_messages_received() << std::endl;
  
  // Print per-subscriber statistics
  auto per_subscriber_counts = g_subscriber->get_per_subscriber_counts();
  std::cout << "Per-subscriber message counts:" << std::endl;
  for (int i = 0; i < static_cast<int>(per_subscriber_counts.size()); ++i) {
    std::cout << "  Subscriber " << i << ": " << per_subscriber_counts[i] << " messages" << std::endl;
  }
  
  // Proper cleanup sequence
  std::cout << "Cleaning up..." << std::endl;
  
  // Cancel executor to stop any pending operations
  std::cout << "Canceling executor..." << std::endl;
  executor.cancel();

  // Remove nodes from executor first
  std::cout << "Removing nodes from executor..." << std::endl;
  executor.remove_node(g_publisher);
  executor.remove_node(g_subscriber);
  
  // Reset shared pointers to trigger proper destruction
  std::cout << "Resetting publisher..." << std::endl;
  g_publisher.reset();
  std::cout << "Resetting subscriber..." << std::endl;
  g_subscriber.reset();
  
  // Small delay to allow cleanup
  std::cout << "Waiting for cleanup to complete..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  std::cout << "Shutting down ROS..." << std::endl;
  rclcpp::shutdown();
  std::cout << "Cleanup complete." << std::endl;
  return 0;
} 