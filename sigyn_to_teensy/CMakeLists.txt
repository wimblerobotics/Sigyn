cmake_minimum_required(VERSION 3.8)
project(sigyn_to_teensy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teensy_bridge src/teensy_bridge.cpp)
ament_target_dependencies(teensy_bridge 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  std_msgs
)

target_include_directories(teensy_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(teensy_bridge PUBLIC c_std_99 cxx_std_17)

install(TARGETS teensy_bridge
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
