#!/bin/bash
# Test script to verify Docker detection works correctly

echo "Testing Docker detection functionality..."
echo "========================================"

# Test 1: Normal execution (should show we're not in Docker)
echo "Test 1: Running check_docker.sh normally"
echo "----------------------------------------"
/home/ros/sigyn_ws/src/Sigyn/scripts/check_docker.sh
echo "Exit code: $?"
echo ""

# Test 2: Simulate Docker environment and test the run script's detection
echo "Test 2: Simulating Docker environment"
echo "-------------------------------------"

# Create a temporary .dockerenv file to simulate Docker environment
sudo touch /.dockerenv 2>/dev/null
if [ $? -eq 0 ]; then
    echo "Created temporary /.dockerenv file"
    
    # Test the run script detection
    echo "Testing run script detection with simulated Docker environment:"
    cd /home/ros/sigyn_ws/src/Sigyn/Docker
    timeout 5s ./runSigynAmd.sh --help 2>&1 | head -10
    
    # Clean up
    sudo rm /.dockerenv 2>/dev/null
    echo "Cleaned up temporary /.dockerenv file"
else
    echo "Could not create /.dockerenv (no sudo access). Testing with environment variable instead."
    
    # Test with environment variable
    echo "Testing run script detection with DOCKER_CONTAINER environment variable:"
    cd /home/ros/sigyn_ws/src/Sigyn/Docker
    DOCKER_CONTAINER=true timeout 5s ./runSigynAmd.sh --help 2>&1 | head -10
fi

echo ""
echo "Test 3: Testing --force flag functionality"
echo "------------------------------------------"
# Test the --force flag by simulating Docker environment
DOCKER_CONTAINER=true ./runSigynAmd.sh --force --help 2>&1 | head -15

echo ""
echo "All tests completed!"
