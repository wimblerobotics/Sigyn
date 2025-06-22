cmake_minimum_required(VERSION 3.8)
project(sigyn_to_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sigyn_interfaces REQUIRED)

add_executable(sigyn_to_sensor src/sigyn_to_sensor.cpp)
ament_target_dependencies(sigyn_to_sensor 
  rclcpp 
  geometry_msgs 
  nav_msgs
  sensor_msgs 
  std_msgs
  sigyn_interfaces
)

target_include_directories(sigyn_to_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sigyn_to_sensor PUBLIC c_std_99 cxx_std_17)

install(TARGETS sigyn_to_sensor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
