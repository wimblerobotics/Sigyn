#!/bin/bash
# Quick setup script for ROS environment when running as host user

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f "/home/ros/sigyn_ws/install/setup.bash" ]; then
    source /home/ros/sigyn_ws/install/setup.bash
    echo "✓ Sourced Sigyn workspace"
else
    echo "⚠ Sigyn workspace not built. Run 'colcon build' first."
fi

# Set up common ROS environment variables
export ROS_DOMAIN_ID=0
export RCUTILS_COLORIZED_OUTPUT=1

# Add some useful aliases
alias cb='colcon build'
alias cs='colcon build --symlink-install'
alias ct='colcon test'
alias ll='ls -la'

echo "✓ ROS 2 Jazzy environment ready!"
echo "✓ Working directory: $(pwd)"
echo ""
echo "Quick commands:"
echo "  cb     - colcon build"
echo "  cs     - colcon build --symlink-install"
echo "  source setup_ros.sh  - re-source this script"
echo ""
