cmake_minimum_required(VERSION 3.5)
project(nsdk_example_pwm)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

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

add_executable(pwm_publisher src/pwm_publisher.cpp)
ament_target_dependencies(pwm_publisher 
  "rclcpp_components"
  "rclcpp")
install(TARGETS pwm_publisher DESTINATION lib/${PROJECT_NAME})
add_executable(pwm_control_subscriber src/pwm_control_subscriber.cpp)
ament_target_dependencies(pwm_control_subscriber 
    "rclcpp_components"
    "rclcpp")
install(TARGETS pwm_control_subscriber DESTINATION lib/${PROJECT_NAME})
target_link_libraries(pwm_control_subscriber mraa)
ament_package()
