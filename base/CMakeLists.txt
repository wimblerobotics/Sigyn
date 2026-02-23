# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

cmake_minimum_required(VERSION 3.8)
project(base)

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

install(PROGRAMS
  scripts/battery_overlay_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
