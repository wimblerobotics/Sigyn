#!/bin/bash

# Test script for teleop_twist_keyboard
# This script tests the teleop functionality and verifies terminal restoration

echo "=== Teleop Twist Keyboard Test ==="
echo "This will test the teleop with custom parameters and verify terminal restoration"
echo ""

# Source the workspace
source ~/sigyn_ws/install/setup.bash

echo "1. Testing teleop with custom speed/turn parameters and repeat rate..."
echo "   Parameters: speed=0.25, turn=0.3, repeat_rate=10.0"
echo "   (This eliminates key repeat delay for smooth control)"
echo ""
echo "Press any key to start teleop test (will run for 5 seconds)..."
read -n 1

echo "Starting teleop..."
timeout 5s ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -p speed:=0.25 \
    -p turn:=0.3 \
    -p repeat_rate:=10.0 \
    -p key_timeout:=0.6

echo ""
echo "2. Testing terminal restoration..."
echo "If you can read this and type normally, the terminal was restored correctly!"
echo ""
echo "Type something to test: "
read user_input
echo "You typed: $user_input"
echo ""
echo "=== Test Complete ==="
echo ""
echo "To use teleop with reduced key repeat delay, run:"
echo "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0"
