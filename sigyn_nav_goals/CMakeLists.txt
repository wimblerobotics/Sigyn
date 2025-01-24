cmake_minimum_required(VERSION 3.8)
project(sigyn_nav_goals)

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
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

include_directories(
  include
  ${nav_msgs_INCLUDE_DIRS}
  ${nav2_common_INCLUDE_DIRS}
  ${nav2_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${unique_identifier_msgs_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  std_msgs
)

add_executable(unknown_points_finder_node
  src/unknown_points_finder_node.cpp
)

ament_target_dependencies(unknown_points_finder_node
  ${dependencies}
)

install(TARGETS
  unknown_points_finder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install includes.
install(DIRECTORY
  include
  DESTINATION include
)

# Install directories.
install(DIRECTORY
  config include launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config include launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_package()