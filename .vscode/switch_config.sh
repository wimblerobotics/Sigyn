#!/bin/bash
# Script to switch IntelliSense configurations for different project types

case "$1" in
    "ros2"|"ROS2")
        echo "Switching to ROS2 configuration..."
        # This would be done through VS Code command palette: C_Cpp.ConfigurationSelect
        echo "Use Ctrl+Shift+P and search for 'C/C++: Select a Configuration' then choose 'ROS2'"
        ;;
    "teensy"|"TeensyV2")
        echo "Switching to TeensyV2 configuration..."
        echo "Use Ctrl+Shift+P and search for 'C/C++: Select a Configuration' then choose 'TeensyV2'"
        ;;
    *)
        echo "Usage: $0 [ros2|teensy]"
        echo "Available configurations:"
        echo "  ros2    - For ROS2 navigation and other C++ code"
        echo "  teensy  - For TeensyV2 Arduino-based code"
        ;;
esac