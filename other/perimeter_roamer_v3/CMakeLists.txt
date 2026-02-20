cmake_minimum_required(VERSION 3.8)
project(perimeter_roamer_v3)

# find dependencies
find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# Find SQLite3
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

add_executable(perimeter_roamer 
  src/perimeter_roamer_node.cpp
  src/bt_nodes.cpp
)

ament_target_dependencies(perimeter_roamer
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  sensor_msgs
  behaviortree_cpp_v3
  tf2
  tf2_geometry_msgs
)

target_link_libraries(perimeter_roamer ${SQLITE3_LIBRARIES})
target_include_directories(perimeter_roamer PRIVATE ${SQLITE3_INCLUDE_DIRS})
target_compile_options(perimeter_roamer PRIVATE ${SQLITE3_CFLAGS_OTHER})

target_include_directories(perimeter_roamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  perimeter_roamer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch bt_xml config scripts data DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/battery_simulator.py
  scripts/capture_waypoints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()