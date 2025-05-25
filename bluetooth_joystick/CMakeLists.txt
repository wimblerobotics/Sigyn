cmake_minimum_required(VERSION 3.5)
project(bluetooth_joystick)

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
find_package(msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(bluetooth_joystick src/joystick_node.cpp)
ament_target_dependencies(
  bluetooth_joystick
  geometry_msgs
  msgs
  rclcpp
  Threads
)

install(TARGETS bluetooth_joystick
  DESTINATION lib/${PROJECT_NAME})

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
