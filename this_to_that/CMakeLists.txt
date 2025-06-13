cmake_minimum_required(VERSION 3.8)
project(this_to_that)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(min_max_curr_rviz_overlay REQUIRED)  # Depend on custom message package instead of wifi_viz
find_package(std_msgs REQUIRED)

# Install Python modules AND data files defined in setup.py
ament_python_install_package(${PROJECT_NAME})

# Remove CMake installation rules for config and launch files
# Let setup.py handle these

# install(FILES
#   config/config.yaml
#   DESTINATION share/${PROJECT_NAME}/config
# )

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/launch
#   OPTIONAL
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
