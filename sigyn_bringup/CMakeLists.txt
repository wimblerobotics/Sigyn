# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

cmake_minimum_required(VERSION 3.8)
project(sigyn_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/apriltag_to_dock_pose.py
  scripts/battery_overlay_publisher.py
  scripts/detection_frame_reporter.py
  scripts/docking_helper.py
  scripts/laser_scan_qos_relay.py
  scripts/nav2_debug_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
