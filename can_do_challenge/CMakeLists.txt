cmake_minimum_required(VERSION 3.8)
project(can_do_challenge)

# find dependencies
find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(can_do_challenge_node 
  src/can_do_challenge_node.cpp
  src/bt_nodes.cpp
)

ament_target_dependencies(can_do_challenge_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  behaviortree_cpp_v3
  tf2
  tf2_geometry_msgs
)

target_link_libraries(can_do_challenge_node)

target_include_directories(can_do_challenge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  can_do_challenge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch bt_xml config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
