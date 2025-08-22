cmake_minimum_required(VERSION 3.8)
project(sigyn_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(sigyn_lidar_node src/sigyn_lidar_node.cpp src/ld06_protocol.cpp src/ld06_motion_compensated.cpp src/multi_lidar_fuser.cpp src/deskewer.cpp)
ament_target_dependencies(sigyn_lidar_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

# New modular node
add_executable(modular_sigyn_lidar_node src/modular_sigyn_lidar_node.cpp src/modular_ld06_parser.cpp src/ld06_protocol.cpp src/multi_lidar_fuser.cpp)
ament_target_dependencies(modular_sigyn_lidar_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS sigyn_lidar_node modular_sigyn_lidar_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
