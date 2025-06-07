#!/bin/bash
# Wrapper to source ROS 2 workspace and launch the teensy_bridge node for debugging
source "${PWD}/install/setup.bash"
exec /opt/ros/jazzy/bin/ros2 run sigyn_to_teensy teensy_bridge