cmake_minimum_required(VERSION 3.8)
project(pairtalk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(priority_executor REQUIRED)

add_executable(pairtalk src/pairtalk.cpp)
target_include_directories(pairtalk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pairtalk PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pairtalk
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "priority_executor"
)

install(TARGETS pairtalk
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
