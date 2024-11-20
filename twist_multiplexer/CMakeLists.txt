cmake_minimum_required(VERSION 3.5)
project(twist_multiplexer)

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
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msgs)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(twist_multiplexer
  src/twist_multiplexer_node.cpp
)

ament_target_dependencies(
  twist_multiplexer
  rclcpp
  geometry_msgs
  msgs
  yaml-cpp
)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

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

install(TARGETS
  twist_multiplexer
  DESTINATION lib/${PROJECT_NAME})

  if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
