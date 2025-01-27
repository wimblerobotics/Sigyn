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
find_package(geometry_msgs REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)

# find_package(nav2_costmap_2d REQUIRED)
# find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(
  include
  ${behaviortree_cpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

set(dependencies
  behaviortree_cpp
  builtin_interfaces
  geometry_msgs
  nav2_behaviors
  nav2_core
  rclcpp
  rclcpp_action
  tf2_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveAShortDistanceAhead.action"
  "action/SaySomething.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(sigyn_say_something_plugin_node SHARED
  src/say_something.cpp
)

list(APPEND plugin_libs sigyn_say_something_plugin_node)

add_library(sigyn_move_a_short_distance_ahead_plugin_node SHARED
  src/move_a_short_distance_ahead.cpp
)

list(APPEND plugin_libs sigyn_move_a_short_distance_ahead_plugin_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  target_link_libraries(${bt_plugin} "${cpp_typesupport_target}")
endforeach()

set(action_servers
  MoveAShortDistanceAheadActionServer
  SaySomethingActionServer
)

foreach(action_server ${action_servers})
  add_library(sigyn_${action_server}
    src/${action_server}.cpp
  )

  ament_target_dependencies(sigyn_${action_server}
    ${dependencies}
  )

  ament_target_dependencies(sigyn_${action_server}
    ${dependencies}
  )

  target_link_libraries(sigyn_${action_server}
    ${plugin_libs}
  )

  rclcpp_components_register_node(sigyn_${action_server} PLUGIN "sigyn_behavior_trees::${action_server}" EXECUTABLE ${action_server})
  install(TARGETS
    sigyn_${action_server}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()

add_executable(bt_test1
  src/bt_test1_main.cpp
)
ament_target_dependencies(bt_test1
  ${dependencies}
)
target_link_libraries(bt_test1
  "${cpp_typesupport_target}"
  sigyn_say_something_plugin_node
)

install(TARGETS bt_test1
  DESTINATION lib/${PROJECT_NAME})

add_executable(bt_test2
  src/bt_test2_main.cpp
)
ament_target_dependencies(bt_test2
  ${dependencies}
)
target_link_libraries(bt_test2
  "${cpp_typesupport_target}"
  sigyn_say_something_plugin_node
)

install(TARGETS bt_test2
  DESTINATION lib/${PROJECT_NAME})

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
