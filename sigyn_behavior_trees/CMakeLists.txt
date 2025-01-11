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

add_compile_options(-g)

# find dependencies
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${nav2_core_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${nav2_util_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  # ${tf2_ros_INCLUDE_DIRS}
)

set(library_name sigyn_behavior_trees_plugin)

set(dependencies
  behaviortree_cpp_v3
  builtin_interfaces
  nav2_behaviors
  nav2_core
  nav2_costmap_2d
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  # tf2_ros
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SaySomething.action"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(${library_name} SHARED
  src/say_something_lib.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${library_name}  "${cpp_typesupport_target}")


pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# add_executable(sigyn_bt1
#   src/bt1.cpp
#   src/say_something.cpp

#   # src/wait.cpp
# )

# target_include_directories(sigyn_bt1 PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# ${behaviortree_cpp_v3_INCLUDE_DIRS}
# ${nav2_core_INCLUDE_DIRS}
# ${nav2_costmap_2d_INCLUDE_DIRS}
# ${nav2_util_INCLUDE_DIRS}
# ${rclcpp_INCLUDE_DIRS}
# ${tf2_ros_INCLUDE_DIRS}
# )

# target_compile_features(sigyn_bt1 PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
# ament_target_dependencies(sigyn_bt1
#   behaviortree_cpp_v3
#   rclcpp
#   rclcpp_action
#   tf2_ros
# )

# target_link_libraries(sigyn_bt1
#   ${nav2_behaviors_LIBRARIES}
#   ${nav2_core_LIBRARIES}
#   ${nav2_costmap_2d_LIBRARIES}
#   ${nav2_util_LIBRARIES}
#   ${rclcpp_LIBRARIES}
#   ${rclcpp_action_LIBRARIES}
#   ${behaviortree_cpp_v3_LIBRARIES}
# )

# install(TARGETS sigyn_bt1
#   DESTINATION lib/${PROJECT_NAME})

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)
# add_library(nav2_say_something_behavior SHARED
#   src/say_something_lib.cpp
#   # src/wait.cpp
# )

# target_include_directories(nav2_say_something_behavior PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${behaviortree_cpp_v3_INCLUDE_DIRS}
#   ${nav2_core_INCLUDE_DIRS}
#   ${nav2_costmap_2d_INCLUDE_DIRS}
#   ${nav2_util_INCLUDE_DIRS}
#   ${rclcpp_INCLUDE_DIRS}
#   ${tf2_ros_INCLUDE_DIRS}
# )

# ament_target_dependencies(rosidl_default_runtime)
  # nav2_say_something_behavior
  # behaviortree_cpp_v3
  # rclcpp
  # rclcpp_action
  # tf2_ros
# )

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)

#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   # set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   # set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
