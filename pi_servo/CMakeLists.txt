cmake_minimum_required(VERSION 3.26)
project(pi_servo)

# ...existing code...

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    sensor_msgs,
    std_msgs
    # wiringpi
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# ...existing code...

add_library(pi_servo_hardware_interface SHARED
    src/pi_servo_hardware_interface.cpp)
target_include_directories(pi_servo_hardware_interface PRIVATE
    include)
ament_target_dependencies(pi_servo_hardware_interface
    rclcpp
    hardware_interface
    ros2_control)

add_executable(servo_controller src/servo_controller.cpp)
ament_target_dependencies(servo_controller rclcpp std_msgs sensor_msgs ros2_control)

install(TARGETS
    pi_servo_hardware_interface
    servo_controller
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
    DESTINATION include/
)

ament_package()
