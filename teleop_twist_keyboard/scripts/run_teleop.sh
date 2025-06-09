#!/bin/bash

# Simple script to run teleop_twist_keyboard with proper terminal access
# Usage: ./run_teleop.sh

echo "Starting teleop_twist_keyboard..."
echo "Make sure to source your workspace first:"
echo "  source ~/sigyn_ws/install/setup.bash"
echo ""
echo "Use CTRL+C to quit"
echo ""

# Source the workspace if not already sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    echo "Sourcing workspace..."
    source ~/sigyn_ws/install/setup.bash
fi

# Run the teleop node
ros2 run teleop_twist_keyboard teleop_twist_keyboard
