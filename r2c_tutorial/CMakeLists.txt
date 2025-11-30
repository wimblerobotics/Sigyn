cmake_minimum_required(VERSION 3.8)
project(r2c_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
