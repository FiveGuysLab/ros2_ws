cmake_minimum_required(VERSION 3.8)
project(multithread_priority_test_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(priority_executor REQUIRED)

add_executable(multithread_priority_test 
  src/multithread_priority_test_main.cpp
  src/scalable_publisher.cpp
  src/scalable_subscriber.cpp
)
target_include_directories(multithread_priority_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multithread_priority_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  multithread_priority_test
  "rclcpp"
  "std_msgs"
  "priority_executor"
)

install(TARGETS multithread_priority_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()