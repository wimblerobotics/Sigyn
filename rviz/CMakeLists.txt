# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Wimble Robotics

cmake_minimum_required(VERSION 3.8)
project(rviz)

find_package(ament_cmake REQUIRED)

# Install files.
install(DIRECTORY
  config 
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
