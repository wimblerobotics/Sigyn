cmake_minimum_required(VERSION 3.5)
project(wifi_signal_visualizer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgsx REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install files.
install(PROGRAMS
  wifi_signal_visualizer/wifi_signal_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
