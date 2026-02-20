#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Setup workspace if it exists
if [ -f ~/sigyn_ws/install/setup.bash ]; then
  source ~/sigyn_ws/install/setup.bash
fi

# Execute the command passed as arguments
exec "$@"