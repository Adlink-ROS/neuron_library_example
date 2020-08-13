cmake_minimum_required(VERSION 3.5)
project(neuronlib_led_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LedService.srv"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

