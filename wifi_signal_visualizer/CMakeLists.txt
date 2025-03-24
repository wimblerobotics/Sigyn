cmake_minimum_required(VERSION 3.5)
project(wifi_signal_visualizer)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LocationWifiSignal.msg"
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/wifi_signal_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
