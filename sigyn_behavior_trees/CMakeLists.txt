cmake_minimum_required(VERSION 3.8)
project(sigyn_behavior_trees)

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

# find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${nav2_util_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

set(dependencies
  behaviortree_cpp
  builtin_interfaces
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  rclcpp
  rclcpp_action
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SaySomething.action"
)

add_library(sigyn_behavior_trees_plugin_node SHARED
  src/say_something_lib.cpp
)

list(APPEND plugin_libs sigyn_behavior_trees_plugin_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_library(sigyn_behavior_tree_action_server SHARED
  src/say_something_server.cpp
)

target_compile_definitions(sigyn_behavior_tree_action_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(sigyn_behavior_tree_action_server
  ${dependencies}
)
rclcpp_components_register_node(sigyn_behavior_tree_action_server PLUGIN "sigyn_behavior_trees::SaySomethingActionServer" EXECUTABLE say_something_action_server)
install(TARGETS
  sigyn_behavior_tree_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sigyn_behavior_trees_plugin_node "${cpp_typesupport_target}")

rosidl_target_interfaces(sigyn_behavior_tree_action_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_package()
