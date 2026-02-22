#!/bin/bash
# Fix rviz2 SSH forwarding between sigyn7900a and amdc
# This script helps diagnose and fix NVIDIA/GLX issues for SSH-forwarded applications

set -e

echo "========================================="
echo "RViz2 SSH Forwarding Fix Script"
echo "========================================="
echo ""

# Function to check NVIDIA driver
check_nvidia_driver() {
    echo "--- Checking NVIDIA Driver ---"
    if command -v nvidia-smi &> /dev/null; then
        nvidia-smi --query-gpu=driver_version --format=csv,noheader
        nvidia-smi
    else
        echo "ERROR: nvidia-smi not found. Is NVIDIA driver installed?"
        return 1
    fi
    echo ""
}

# Function to check available NVIDIA drivers
check_available_drivers() {
    echo "--- Checking Available NVIDIA Drivers ---"
    apt-cache search '^nvidia-driver-[0-9]+$' | sort -V
    echo ""
    
    echo "--- Recommended drivers ---"
    ubuntu-drivers list
    echo ""
}

# Function to upgrade NVIDIA driver
upgrade_nvidia_driver() {
    local target_version=$1
    
    echo "--- Upgrading to NVIDIA Driver $target_version ---"
    echo "This will:"
    echo "  1. Remove old driver packages"
    echo "  2. Install nvidia-driver-$target_version"
    echo "  3. Rebuild initramfs"
    echo ""
    read -p "Continue? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        return 1
    fi
    
    sudo apt-get update
    sudo apt-get install -y nvidia-driver-$target_version
    echo "Driver installed. A reboot is required."
}

# Function to validate GLX
validate_glx() {
    echo "--- Validating GLX Configuration ---"
    
    if command -v glxinfo &> /dev/null; then
        echo "OpenGL vendor string: $(glxinfo | grep "OpenGL vendor")"
        echo "OpenGL renderer string: $(glxinfo | grep "OpenGL renderer")"
        echo "OpenGL version string: $(glxinfo | grep "OpenGL version")"
        echo ""
        echo "Direct rendering: $(glxinfo | grep "direct rendering")"
    else
        echo "glxinfo not found. Installing mesa-utils..."
        sudo apt-get install -y mesa-utils
        echo "Please run this function again."
        return 1
    fi
    echo ""
}

# Function to test rviz2 locally
test_rviz2_local() {
    echo "--- Testing rviz2 locally ---"
    echo "This will launch rviz2 for 10 seconds..."
    timeout 10s rviz2 || true
    echo "If rviz2 launched without crashing, local GLX is working."
    echo ""
}

# Function to setup VirtualGL + TurboVNC
setup_virtualgl() {
    echo "--- Setting up VirtualGL + TurboVNC ---"
    echo ""
    echo "VirtualGL allows OpenGL applications to run on the GPU"
    echo "while displaying through VNC (more stable than X11 forwarding)"
    echo ""
    
    # Check if already installed
    if command -v vglrun &> /dev/null && command -v vncserver &> /dev/null; then
        echo "VirtualGL and TurboVNC appear to be already installed."
        echo ""
    else
        echo "Installation steps:"
        echo "1. Download VirtualGL:"
        echo "   wget https://sourceforge.net/projects/virtualgl/files/3.1/virtualgl_3.1_amd64.deb"
        echo "   sudo dpkg -i virtualgl_3.1_amd64.deb"
        echo ""
        echo "2. Download TurboVNC:"
        echo "   wget https://sourceforge.net/projects/turbovnc/files/3.1.1/turbovnc_3.1.1_amd64.deb"
        echo "   sudo dpkg -i turbovnc_3.1.1_amd64.deb"
        echo ""
        echo "3. Configure VirtualGL:"
        echo "   sudo /opt/VirtualGL/bin/vglserver_config"
        echo "   (Select option 1 to configure for use with VNC)"
        echo ""
    fi
    
    echo "Usage:"
    echo "On amdc:"
    echo "  /opt/TurboVNC/bin/vncserver :1"
    echo ""
    echo "On sigyn7900a:"
    echo "  ssh -L 5901:localhost:5901 amdc"
    echo "  (then connect VNC viewer to localhost:5901)"
    echo ""
    echo "To launch rviz2 with VirtualGL:"
    echo "  vglrun rviz2"
    echo ""
}

# Main menu
show_menu() {
    echo "========================================="
    echo "What would you like to do?"
    echo "========================================="
    echo "1. Check current NVIDIA driver version"
    echo "2. Check available NVIDIA drivers"
    echo "3. Upgrade NVIDIA driver (specify version)"
    echo "4. Validate GLX configuration"
    echo "5. Test rviz2 locally"
    echo "6. Setup VirtualGL + TurboVNC"
    echo "7. Show SSH X forwarding test commands"
    echo "8. Exit"
    echo ""
    read -p "Enter choice [1-8]: " choice
    
    case $choice in
        1) check_nvidia_driver ;;
        2) check_available_drivers ;;
        3) 
            read -p "Enter driver version (e.g., 550): " version
            upgrade_nvidia_driver $version
            ;;
        4) validate_glx ;;
        5) test_rviz2_local ;;
        6) setup_virtualgl ;;
        7) show_ssh_test_commands ;;
        8) exit 0 ;;
        *) echo "Invalid choice"; exit 1 ;;
    esac
}

show_ssh_test_commands() {
    echo "--- SSH X Forwarding Test Commands ---"
    echo ""
    echo "From sigyn7900a, test SSH X forwarding to amdc:"
    echo ""
    echo "1. Basic test with trusted X11 forwarding:"
    echo "   ssh -Y amdc 'glxinfo | grep -E \"vendor|renderer|version\"'"
    echo ""
    echo "2. Test with glxgears:"
    echo "   ssh -Y amdc glxgears"
    echo ""
    echo "3. Test rviz2:"
    echo "   ssh -Y amdc 'source ~/.bash_aliases && rviz2'"
    echo ""
    echo "If these crash or show \"Mesa/llvmpipe\" instead of NVIDIA,"
    echo "the driver or GLX configuration needs attention."
    echo ""
}

# Run main menu if no arguments provided
if [ $# -eq 0 ]; then
    show_menu
else
    case "$1" in
        check) check_nvidia_driver ;;
        available) check_available_drivers ;;
        upgrade) upgrade_nvidia_driver "$2" ;;
        validate) validate_glx ;;
        test) test_rviz2_local ;;
        virtualgl) setup_virtualgl ;;
        ssh-test) show_ssh_test_commands ;;
        *) echo "Unknown command: $1"; exit 1 ;;
    esac
fi
