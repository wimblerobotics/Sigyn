cmake_minimum_required(VERSION 3.8)
project(r2c_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz-sim8 REQUIRED)

# Build sensor hardware interface library (real hardware)
add_library(sensor_hardware_interface SHARED
  src/sensor_hardware_interface.cpp
)
target_include_directories(sensor_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# Build sensor hardware interface library (Gazebo simulation)
add_library(sensor_hardware_interface_sim SHARED
  src/sensor_hardware_interface_sim.cpp
)
target_include_directories(sensor_hardware_interface_sim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_hardware_interface_sim
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  gz_ros2_control
  gz-sim8
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface sensor_hardware_interface_plugin.xml)
pluginlib_export_plugin_description_file(gz_ros2_control sensor_hardware_interface_plugin.xml)

# Build sensor state broadcaster controller
add_library(sensor_state_broadcaster SHARED
  src/sensor_state_broadcaster.cpp
)
target_include_directories(sensor_state_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_state_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# Export controller plugin
pluginlib_export_plugin_description_file(controller_interface sensor_state_broadcaster_plugin.xml)

# Build sensor command controller (simulation only)
add_library(sensor_command_controller SHARED
  src/sensor_command_controller.cpp
)
target_include_directories(sensor_command_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sensor_command_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Export command controller plugin
pluginlib_export_plugin_description_file(controller_interface sensor_command_controller_plugin.xml)

# Install controller libraries
install(
  TARGETS sensor_state_broadcaster sensor_command_controller
  DESTINATION lib
)

# Install hardware interface libraries
install(
  TARGETS sensor_hardware_interface sensor_hardware_interface_sim
  DESTINATION lib
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install directories
install(
  DIRECTORY
    urdf
    launch
    config
    worlds
    docs
  DESTINATION
    share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/temperature_heater.py
  scripts/monitor_robot.py
  scripts/temperature_sensor_sim.py
  scripts/twist_to_stamped.py
  scripts/sensor_simulator.py
  scripts/sensor_reader.py
  scripts/sensor_writer.py
  scripts/load_sensor_hardware.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  sensor_hardware_interface
  sensor_hardware_interface_sim
  sensor_state_broadcaster
  sensor_command_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  gz_ros2_control
)

ament_package()
