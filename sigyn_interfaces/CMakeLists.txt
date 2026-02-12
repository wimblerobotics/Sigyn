cmake_minimum_required(VERSION 3.5)

project(sigyn_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveElevator.action"
  "action/MoveExtender.action"
  "msg/SystemFault.msg"
  "msg/EStopStatus.msg"
  "msg/GripperStatus.msg"
  "msg/GripperPositionCommand.msg"
  "srv/GripperPosition.srv"
  "srv/ResetFault.srv"
  "srv/TeensySdGetDir.srv"
  "srv/TeensySdGetFile.srv"
  DEPENDENCIES std_msgs
)

# install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
