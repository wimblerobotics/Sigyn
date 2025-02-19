cmake_minimum_required(VERSION 3.16)
project(sigynpi_gripper_end LANGUAGES CXX)

# Default to C++14
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_compile_options(-g)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # wiringpi
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE
add_library(
  sigynpi_gripper_end
  SHARED
  src/servo_motor_hardware_interface.cpp
)
target_compile_features(sigynpi_gripper_end PUBLIC cxx_std_17)
target_include_directories(sigynpi_gripper_end PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/sigynpi_gripper_end>
)
ament_target_dependencies(
  sigynpi_gripper_end PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/sigynpi_gripper_end
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/sigynpi_gripper_end
# )
install(
  DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS sigynpi_gripper_end
  EXPORT export_sigynpi_gripper_end
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_sigynpi_gripper_end HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
