cmake_minimum_required(VERSION 3.8)
project(sigyn_to_sensor_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Custom message interfaces
find_package(sigyn_interfaces REQUIRED)

# Include directories
include_directories(include)

# TeensyV2 Bridge Node - Main communication bridge
add_executable(teensy_bridge_node
  src/teensy_bridge_node.cpp
  src/teensy_bridge.cpp
  src/message_parser.cpp
  src/safety_publisher.cpp
)

ament_target_dependencies(teensy_bridge_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  rcl_interfaces
  sigyn_interfaces
)

# Battery Monitor Node - Dedicated battery monitoring
add_executable(battery_monitor_node
  src/battery_monitor_node.cpp
  src/battery_monitor.cpp
)

ament_target_dependencies(battery_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  rcl_interfaces
  sigyn_interfaces
)

# Performance Monitor Node - System performance tracking
add_executable(performance_monitor_node
  src/performance_monitor_node.cpp
  src/performance_monitor.cpp
)

ament_target_dependencies(performance_monitor_node
  rclcpp
  std_msgs
  diagnostic_msgs
  rcl_interfaces
  sigyn_interfaces
)

# Safety Coordinator Node - E-stop and safety management
add_executable(safety_coordinator_node
  src/safety_coordinator_node.cpp
  src/safety_coordinator.cpp
)

ament_target_dependencies(safety_coordinator_node
  rclcpp
  std_msgs
  std_srvs
  diagnostic_msgs
  rcl_interfaces
  sigyn_interfaces
)

# IMU Monitor Node - Dual BNO055 sensor monitoring
add_executable(imu_monitor_node
  src/imu_monitor_node.cpp
  src/message_parser.cpp
)

ament_target_dependencies(imu_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
  rcl_interfaces
  sigyn_interfaces
)

# Install executables
install(TARGETS
  teensy_bridge_node
  battery_monitor_node
  performance_monitor_node
  safety_coordinator_node
  imu_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
