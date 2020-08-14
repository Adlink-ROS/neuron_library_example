cmake_minimum_required(VERSION 3.5)
project(nsdk_example_serial)

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
SET(MRAA_INCLUDE_PATH "/opt/adlink/neuron-sdk/neuron-library/include/")
if (NOT "$ENV{NLIB_MRAA_INCLUDE_PATH}" STREQUAL "")
  SET(MRAA_INCLUDE_PATH "$ENV{NLIB_MRAA_INCLUDE_PATH}")
endif()
SET(MRAA_LIBRARY_PATH "/opt/adlink/neuron-sdk/neuron-library/lib/")
if (NOT "$ENV{NLIB_MRAA_LIBRARY_PATH}" STREQUAL "")
  SET(MRAA_LIBRARY_PATH "$ENV{NLIB_MRAA_LIBRARY_PATH}")
endif()
include_directories(${MRAA_INCLUDE_PATH})
link_directories(${MRAA_LIBRARY_PATH})

# RPATH settings: must be set before add_executable
## Don't skip RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH FALSE)
## Use RPATH while building
SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
## Set the RPATH
SET(CMAKE_INSTALL_RPATH ${MRAA_LIBRARY_PATH}) 
## Use RPATH while install
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(uart_message_subscriber src/message_subscriber.cpp)
add_executable(mraa_uart_controller src/mraa_uart_controller.cpp)
add_executable(data_publisher src/data_publisher.cpp)
ament_target_dependencies(data_publisher rclcpp std_msgs)
ament_target_dependencies(uart_message_subscriber rclcpp std_msgs)
ament_target_dependencies(mraa_uart_controller rclcpp std_msgs)
target_link_libraries(mraa_uart_controller mraa)

install(TARGETS
  uart_message_subscriber
  mraa_uart_controller
  data_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
