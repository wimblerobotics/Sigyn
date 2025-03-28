cmake_minimum_required(VERSION 3.5)
project(pi_servo)

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

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(controller_interface)
find_package(controller_manager_msgs)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# add_library(${PROJECT_NAME} SHARED
#   src/pi_servo.cpp
# )

# include_directories(
#   include
#   ${conotroller_interface_INCLUDE_DIRS}
#   ${controller_manager_msgs_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${hardware_interface_INCLUDE_DIRS}
#   ${pluginlib_INCLUDE_DIRS}
# )

# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   hardware_interface
#   pluginlib
# )

# pluginlib_export_plugin_description_file(hardware_interface pi_servo_plugin.xml)

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

ament_package()
