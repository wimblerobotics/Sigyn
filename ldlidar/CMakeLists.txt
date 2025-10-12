cmake_minimum_required(VERSION 3.5)
project(ldlidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

aux_source_directory(. file_name)

if(file_name MATCHES "slbf.cpp")
  add_definitions(-D"USE_SLBF")
  message("-- #define USE_SLBF")
endif()

if(file_name MATCHES "slbi.cpp")
  add_definitions(-D"USE_SLBI")
  message("-- #define USE_SLBI")
endif()

file(GLOB MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(ldlidar ${MAIN_SRC})
target_include_directories(ldlidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ldlidar sensor_msgs rclcpp)
target_link_libraries(ldlidar udev)

install(TARGETS ldlidar
  DESTINATION lib/${PROJECT_NAME})

ament_package()