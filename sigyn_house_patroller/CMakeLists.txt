cmake_minimum_required(VERSION 3.8)
project(sigyn_house_patroller)

# Suppress CMake policy warnings for PCL compatibility
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Added -Wno-dev to suppress warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-dev")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sqlite3_vendor REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Generate messages, services and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThreatAlert.msg"
  "msg/PatrolStatus.msg"
  "msg/RoomIdentification.msg"
  "msg/SystemHealth.msg"
  "srv/SetPatrolMode.srv"
  "srv/GetRoomInfo.srv"
  "action/PatrolToWaypoint.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs nav_msgs sensor_msgs
)

# Core library
add_library(sigyn_house_patroller_core
  src/core/navigation_coordinator.cpp
  src/core/waypoint_manager.cpp
  src/core/patrol_manager.cpp
  src/core/threat_detection_manager.cpp
  src/behavior_tree/behavior_tree_manager.cpp
)

# Commented out other files for incremental debugging
# src/core/threat_detection_manager.cpp

target_include_directories(sigyn_house_patroller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Verified target name and dependencies
ament_target_dependencies(sigyn_house_patroller_core
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp
  nav2_core
  nav2_util
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  lifecycle_msgs
  ament_index_cpp
)

target_link_libraries(sigyn_house_patroller_core ${SQLITE3_LIBRARIES})
target_include_directories(sigyn_house_patroller_core PUBLIC ${SQLITE3_INCLUDE_DIRS})

# Commented out all unnecessary executables and libraries for incremental debugging
# Feature recognizer nodes
add_executable(battery_monitor_node src/nodes/battery_monitor_node.cpp)
add_executable(temperature_monitor_node src/nodes/temperature_monitor_node.cpp)
add_executable(door_monitor_node src/nodes/door_monitor_node.cpp)
add_executable(change_detector_node src/nodes/change_detector_node.cpp)
add_executable(localization_corrector_node src/nodes/localization_corrector_node.cpp)

# Main patrol nodes
add_executable(patrol_coordinator_node src/nodes/patrol_coordinator_node.cpp)
add_executable(threat_response_node src/nodes/threat_response_node.cpp)
add_executable(behavior_tree_manager_node src/nodes/behavior_tree_manager_node.cpp)

# Behavior tree plugins
add_library(sigyn_patrol_bt_plugins SHARED
  src/behavior_tree/check_battery_level.cpp
  src/behavior_tree/check_door_state.cpp
  src/behavior_tree/check_temperature.cpp
  src/behavior_tree/navigate_to_waypoint.cpp
  src/behavior_tree/perform_room_check.cpp
  src/behavior_tree/send_threat_alert.cpp
)

# Link all executables
set(EXEC_TARGETS
  battery_monitor_node
  temperature_monitor_node
  door_monitor_node
  change_detector_node
  localization_corrector_node
  patrol_coordinator_node
  threat_response_node
  behavior_tree_manager_node
)

foreach(target ${EXEC_TARGETS})
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav2_msgs
    nav2_behavior_tree
    behaviortree_cpp
    nav2_core
    nav2_util
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
  )
  
  target_link_libraries(${target} sigyn_house_patroller_core)
endforeach()

# Behavior tree plugin setup
target_include_directories(sigyn_patrol_bt_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(sigyn_patrol_bt_plugins
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp
  nav2_core
  nav2_util
)

target_link_libraries(sigyn_patrol_bt_plugins sigyn_house_patroller_core)

# Install targets
install(TARGETS sigyn_house_patroller_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${EXEC_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sigyn_patrol_bt_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav2_behavior_tree
  behaviortree_cpp
  nav2_core
  nav2_util
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
)

# Generate message targets
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sigyn_house_patroller_core "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
