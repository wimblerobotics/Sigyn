#!/bin/bash
# Post-reboot validation script for NVIDIA driver upgrade
# Run this after rebooting amdc to validate the driver installation

set -e

echo "========================================="
echo "Post-Reboot NVIDIA Driver Validation"
echo "========================================="
echo ""

# Check NVIDIA driver version
echo "1. Checking NVIDIA Driver Version:"
echo "-----------------------------------"
nvidia-smi --query-gpu=driver_version,gpu_name --format=csv,noheader
echo ""
nvidia-smi
echo ""

# Check GLX configuration
echo "2. Checking GLX Configuration:"
echo "-------------------------------"
if ! command -v glxinfo &> /dev/null; then
    echo "Installing mesa-utils (contains glxinfo)..."
    sudo apt-get install -y mesa-utils
fi

glxinfo | grep -E "vendor|renderer|version|direct rendering"
echo ""

# Test that we're using NVIDIA, not Mesa
echo "3. Verifying NVIDIA (not Mesa) is being used:"
echo "----------------------------------------------"
renderer=$(glxinfo | grep "OpenGL renderer" | cut -d: -f2)
if echo "$renderer" | grep -qi "nvidia"; then
    echo "✓ SUCCESS: Using NVIDIA GPU"
elif echo "$renderer" | grep -qi "mesa\|llvmpipe"; then
    echo "✗ WARNING: Using Mesa/Software rendering instead of NVIDIA!"
    echo "  This may indicate a driver configuration issue."
else
    echo "? UNKNOWN: Renderer is: $renderer"
fi
echo ""

# Check Xorg log for NVIDIA
echo "4. Checking Xorg log for NVIDIA driver:"
echo "----------------------------------------"
if [ -f /var/log/Xorg.0.log ]; then
    if grep -q "NVIDIA(GPU" /var/log/Xorg.0.log; then
        echo "✓ Xorg is using NVIDIA driver"
        grep "NVIDIA(GPU" /var/log/Xorg.0.log | head -5
    else
        echo "✗ WARNING: Could not find NVIDIA GPU in Xorg log"
    fi
else
    echo "? Could not find /var/log/Xorg.0.log"
fi
echo ""

# Check kernel modules
echo "5. Checking NVIDIA kernel modules:"
echo "-----------------------------------"
lsmod | grep nvidia || echo "✗ No NVIDIA modules loaded!"
echo ""

# Test GLX gears (if display available)
echo "6. Testing GLX with glxgears:"
echo "------------------------------"
if [ -n "$DISPLAY" ]; then
    echo "Running glxgears for 5 seconds..."
    timeout 5s glxgears || echo "(glxgears test completed or timed out)"
else
    echo "No DISPLAY set, skipping glxgears test."
    echo "To test manually with SSH forwarding from sigyn7900a:"
    echo "  ssh -Y amdc glxgears"
fi
echo ""

# Test rviz2 (if available)
echo "7. Testing rviz2 (if available):"
echo "---------------------------------"
if command -v rviz2 &> /dev/null; then
    if [ -n "$DISPLAY" ]; then
        echo "Launching rviz2 for 10 seconds..."
        timeout 10s rviz2 || echo "(rviz2 test completed or timed out)"
        echo "✓ If rviz2 launched without errors, local rendering works!"
    else
        echo "No DISPLAY set, skipping rviz2 local test."
        echo "To test manually with SSH forwarding from sigyn7900a:"
        echo "  ssh -Y amdc 'source ~/.bash_aliases && rviz2'"
    fi
else
    echo "rviz2 not found in PATH. Ensure ROS2 environment is sourced."
fi
echo ""

echo "========================================="
echo "SSH Forwarding Test Instructions"
echo "========================================="
echo ""
echo "From sigyn7900a, run these tests:"
echo ""
echo "1. Update bash_aliases on sigyn7900a first:"
echo "   scp ~/sigyn_ws/src/Sigyn/bashrc sigyn7900a:~/.bash_aliases"
echo "   ssh sigyn7900a 'source ~/.bash_aliases'"
echo ""
echo "2. Test GLX info via SSH:"
echo "   ssh -Y amdc 'glxinfo | grep renderer'"
echo "   (Should show NVIDIA, not Mesa)"
echo ""
echo "3. Test glxgears via SSH:"
echo "   ssh -Y amdc glxgears"
echo "   (Should show spinning gears)"
echo ""
echo "4. Test rviz2 via SSH:"
echo "   ssh -Y amdc 'source ~/.bash_aliases && rviz2'"
echo "   (Should launch without crashes)"
echo ""
echo "========================================="
echo "Validation Complete!"
echo "========================================="
