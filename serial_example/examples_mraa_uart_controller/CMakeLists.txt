cmake_minimum_required(VERSION 3.5)
project(examples_mraa_uart_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MRAA
include_directories(/home/ros/mraa\ workspace/mraa/api/)
link_directories(/home/ros/mraa\ workspace/mraa/build/src/)

add_executable(mraa_uart_controller mraa_uart_controller.cpp)
ament_target_dependencies(mraa_uart_controller rclcpp std_msgs)
target_link_libraries(mraa_uart_controller mraa)


install(TARGETS

  mraa_uart_controller
  
  DESTINATION lib/${PROJECT_NAME})#subscriber_not_composable  subscriber_lambda

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
