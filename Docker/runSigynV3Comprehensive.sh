#!/bin/bash

# Run comprehensive Sigyn development environment
# with full hardware access, networking, and workspace mount
#
# Usage: ./runSigynV3Comprehensive.sh [WORKSPACE_PATH]
#        ./runSigynV3Comprehensive.sh --help
#
# Examples:
#   ./runSigynV3Comprehensive.sh                    # Use default /home/ros/sigyn_ws
#   ./runSigynV3Comprehensive.sh ~/my_other_ws      # Use custom workspace
#   ./runSigynV3Comprehensive.sh /path/to/any/ws    # Use any workspace

set -e

# Configuration
IMAGE_NAME="sigyn-dev-v3:comprehensive"
CONTAINER_NAME="sigyn-dev-comprehensive"

# Default workspace path
DEFAULT_WORKSPACE_PATH="/home/ros/sigyn_ws"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Help function
show_help() {
    echo -e "${BLUE}Sigyn Comprehensive Development Environment${NC}"
    echo ""
    echo -e "${GREEN}Usage:${NC}"
    echo "  $0 [WORKSPACE_PATH]"
    echo "  $0 --help"
    echo ""
    echo -e "${GREEN}Arguments:${NC}"
    echo "  WORKSPACE_PATH    Path to workspace directory to mount (default: $DEFAULT_WORKSPACE_PATH)"
    echo ""
    echo -e "${GREEN}Examples:${NC}"
    echo "  $0                           # Use default workspace"
    echo "  $0 ~/my_other_ws             # Use custom workspace"
    echo "  $0 /path/to/any/ros2_ws      # Use any ROS2 workspace"
    echo ""
    echo -e "${GREEN}Features:${NC}"
    echo "  • ROS2 Jazzy with 70+ packages (Nav2, Cartographer, Gazebo)"
    echo "  • 30+ development aliases (cb, nav, sim, etc.)"
    echo "  • CycloneDDS configured for multi-robot communication"
    echo "  • Full hardware access for robot development"
    echo "  • X11 forwarding for GUI applications"
    echo ""
    exit 0
}

# Parse arguments
if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    show_help
fi

# Set workspace path
if [ -n "$1" ]; then
    WORKSPACE_PATH="$(realpath "$1")"
else
    WORKSPACE_PATH="$DEFAULT_WORKSPACE_PATH"
fi

echo -e "${GREEN}Starting Sigyn comprehensive development environment...${NC}"

# Check if image exists
if ! docker image inspect "$IMAGE_NAME" &> /dev/null; then
    echo -e "${RED}Error: Image $IMAGE_NAME not found${NC}"
    echo -e "${YELLOW}Please build it first with: ./buildSigynV3Comprehensive.sh${NC}"
    exit 1
fi

# Remove existing container if it exists
if docker container inspect "$CONTAINER_NAME" &> /dev/null; then
    echo -e "${YELLOW}Removing existing container...${NC}"
    docker rm -f "$CONTAINER_NAME"
fi

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo -e "${RED}Error: Workspace $WORKSPACE_PATH not found${NC}"
    echo -e "${YELLOW}Please adjust WORKSPACE_PATH in this script or create the workspace${NC}"
    exit 1
fi

echo -e "${GREEN}Starting container with:${NC}"
echo "  Image: $IMAGE_NAME"
echo "  Workspace: $WORKSPACE_PATH -> /workspace"
echo "  Network: host"
echo "  Devices: USB, video access"

# Run the container with comprehensive setup
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --network host \
    --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="USER_ID=$(id -u)" \
    --env="GROUP_ID=$(id -g)" \
    --volume="$WORKSPACE_PATH:/workspace:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/dev:/dev:rw" \
    --volume="/sys:/sys:rw" \
    --volume="/var/run/docker.sock:/var/run/docker.sock:rw" \
    --device-cgroup-rule="c 81:* rmw" \
    --device-cgroup-rule="c 189:* rmw" \
    --workdir="/workspace" \
    "$IMAGE_NAME" \
    bash

echo -e "${GREEN}Container stopped.${NC}"
