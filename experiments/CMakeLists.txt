cmake_minimum_required(VERSION 3.10)
project(experiments)

set(CMAKE_BUILD_TYPE Debug)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

set(SOURCES_MOTOR_CHARACTERIZATION
  src/motor_characterization.cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(motor_characterization ${SOURCES_MOTOR_CHARACTERIZATION})

ament_target_dependencies(
  motor_characterization
  rclcpp
  geometry_msgs
  msgs
  nav_msgs
  tf2
  tf2_ros
  yaml-cpp
)

target_link_libraries(
  motor_characterization
  ${YAML_CPP_LIBRARIES}
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

# Install Python scripts from model_to_wall_db
install(PROGRAMS
  model_to_wall_db/model_to_wall_db/extract_walls.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME extract_walls
)

install(PROGRAMS
  model_to_wall_db/scripts/query_walls.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME query_walls
)

install(TARGETS
  motor_characterization
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
