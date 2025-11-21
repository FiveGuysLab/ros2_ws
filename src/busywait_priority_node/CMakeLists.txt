cmake_minimum_required(VERSION 3.8)
project(busywait_priority_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(preemptive_executor REQUIRED)
# find_package(simple_timer REQUIRED)

add_executable(busywait_priority src/busywait_priority.cpp)
target_include_directories(busywait_priority PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(busywait_priority PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(
  busywait_priority
  "rclcpp"
  "std_msgs"
  "preemptive_executor"
  # "simple_timer"
)

install(TARGETS busywait_priority
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 