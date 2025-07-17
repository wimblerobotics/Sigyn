#!/bin/bash
# Run script for Sigyn M4 (Apple Silicon) Docker container
# This script provides options for running the container on ARM64 Ubuntu VM

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

# Check if we're on ARM64/aarch64
check_architecture() {
    local arch=$(uname -m)
    if [[ "$arch" != "arm64" && "$arch" != "aarch64" ]]; then
        print_warning "This script is designed for ARM64/aarch64 architecture"
        print_info "Detected architecture: $arch"
        print_info "Expected: arm64 or aarch64"
        echo ""
        read -p "Continue anyway? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        print_success "Detected ARM64 architecture: $arch"
    fi
}

# Check Docker and X11
check_prerequisites() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker is not installed!"
        echo ""
        echo "Please install Docker:"
        echo "  sudo apt update"
        echo "  sudo apt install docker.io"
        echo "  sudo usermod -aG docker \$USER"
        echo "  # Then log out and back in"
        exit 1
    fi
    
    if ! docker info &> /dev/null; then
        print_error "Docker is not running or you don't have permission!"
        echo ""
        echo "Try:"
        echo "  sudo systemctl start docker"
        echo "  sudo usermod -aG docker \$USER"
        echo "  # Then log out and back in"
        exit 1
    fi
    
    # Check X11 forwarding
    if [ -z "$DISPLAY" ]; then
        print_warning "DISPLAY variable not set - GUI applications may not work"
        echo "Current DISPLAY: $DISPLAY"
    else
        print_success "X11 display available: $DISPLAY"
    fi
}

# Setup X11 forwarding for Ubuntu VM
setup_x11_forwarding() {
    print_info "Setting up X11 forwarding for Ubuntu VM..."
    
    # Allow local connections to X server
    xhost +local:docker 2>/dev/null || print_warning "Could not configure X11 permissions"
    
    # Create X11 auth file if needed
    XAUTH=/tmp/.docker.xauth
    if [ ! -f $XAUTH ]; then
        touch $XAUTH
        chmod 666 $XAUTH
        if [ ! -z "$DISPLAY" ]; then
            xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || true
        fi
    fi
    
    export XAUTH_FILE=$XAUTH
    print_success "X11 forwarding configured"
}

# Default values
CONTAINER_NAME="sigyn_m4_container"
IMAGE_NAME="sigyn_m4"
WORKSPACE_MOUNT=""
INTERACTIVE="-it"
REMOVE_AFTER_EXIT="--rm"
PRIVILEGED=""
ROS_DOMAIN_ID="0"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --mount-workspace)
            WORKSPACE_MOUNT="-v /home/ros/sigyn_ws:/home/ros/sigyn_ws"
            shift
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --detached)
            INTERACTIVE="-d"
            REMOVE_AFTER_EXIT=""
            shift
            ;;
        --privileged)
            PRIVILEGED="--privileged"
            shift
            ;;
        --ros-domain)
            ROS_DOMAIN_ID="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --mount-workspace     Mount the host workspace directory"
            echo "  --name NAME           Set container name (default: sigyn_m4_container)"
            echo "  --image NAME          Set image name (default: sigyn_m4)"
            echo "  --detached            Run in detached mode"
            echo "  --privileged          Run with privileged access"
            echo "  --ros-domain ID       Set ROS domain ID (default: 0)"
            echo "  --help                Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Basic run"
            echo "  $0 --mount-workspace                 # Run with workspace mounted"
            echo "  $0 --name my_sigyn_m4                # Run with custom name"
            echo "  $0 --detached --name sigyn_bg        # Run in background"
            echo "  $0 --ros-domain 42                   # Custom ROS domain"
            echo ""
            echo "Ubuntu VM Notes:"
            echo "  • Designed for ARM64/aarch64 Ubuntu VM"
            echo "  • X11 forwarding works natively (no XQuartz needed)"
            echo "  • GUI applications should work out of the box"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Pre-flight checks
print_info "Starting Sigyn M4 Docker container..."
echo ""

check_architecture
check_prerequisites
setup_x11_forwarding

# Check if image exists
if ! docker image inspect $IMAGE_NAME &> /dev/null; then
    print_error "Docker image '$IMAGE_NAME' not found!"
    echo ""
    echo "Please build the image first:"
    echo "  ./buildSigynM4.sh"
    echo ""
    echo "Or specify a different image:"
    echo "  $0 --image your_image_name"
    exit 1
fi

# Verify image architecture
IMAGE_ARCH=$(docker image inspect $IMAGE_NAME --format '{{.Architecture}}' 2>/dev/null || echo "unknown")
print_info "Image architecture: $IMAGE_ARCH"

# Build the docker run command
DOCKER_CMD="docker run $INTERACTIVE $REMOVE_AFTER_EXIT $PRIVILEGED --name $CONTAINER_NAME"

# Add workspace mount if specified
if [[ -n "$WORKSPACE_MOUNT" ]]; then
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    
    DOCKER_CMD="$DOCKER_CMD $WORKSPACE_MOUNT"
    DOCKER_CMD="$DOCKER_CMD -w /home/ros/sigyn_ws"
    DOCKER_CMD="$DOCKER_CMD -e USER_ID=$HOST_UID"
    DOCKER_CMD="$DOCKER_CMD -e GROUP_ID=$HOST_GID"
    
    print_info "Mounting workspace with user mapping (UID:$HOST_UID, GID:$HOST_GID)"
fi

# Add X11 forwarding for GUI applications
if [[ -n "$DISPLAY" ]]; then
    DOCKER_CMD="$DOCKER_CMD -e DISPLAY=$DISPLAY"
    DOCKER_CMD="$DOCKER_CMD -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    if [[ -n "$XAUTH_FILE" ]]; then
        DOCKER_CMD="$DOCKER_CMD -v $XAUTH_FILE:/tmp/.docker.xauth:ro"
        DOCKER_CMD="$DOCKER_CMD -e XAUTHORITY=/tmp/.docker.xauth"
    fi
    print_success "X11 forwarding configured for GUI applications"
else
    print_warning "X11 forwarding not available - GUI applications may not work"
fi

# Add additional environment variables
DOCKER_CMD="$DOCKER_CMD -e QT_X11_NO_MITSHM=1"
DOCKER_CMD="$DOCKER_CMD -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# Add shared memory size for better performance
DOCKER_CMD="$DOCKER_CMD --shm-size=1g"

# Add network host for better ROS2 communication
DOCKER_CMD="$DOCKER_CMD --network=host"

# Add platform specification for ARM64
DOCKER_CMD="$DOCKER_CMD --platform linux/arm64"

# Add the image name
DOCKER_CMD="$DOCKER_CMD $IMAGE_NAME"

# Display run configuration
print_info "Container Configuration:"
echo "  📦 Image: $IMAGE_NAME (arch: $IMAGE_ARCH)"
echo "  🏷️  Name: $CONTAINER_NAME"
echo "  🏗️  Platform: linux/arm64"
echo "  🌐 ROS Domain: $ROS_DOMAIN_ID"
echo "  💻 Host arch: $(uname -m)"
if [[ -n "$WORKSPACE_MOUNT" ]]; then
    echo "  📁 Workspace: Mounted"
fi
if [[ -n "$DISPLAY" ]]; then
    echo "  🖥️  Display: $DISPLAY (X11 forwarding enabled)"
fi
echo ""

print_info "Running command:"
echo "$DOCKER_CMD"
echo ""

print_info "Useful commands once inside container:"
echo "  • Launch simulation: ros2 launch base sigyn.launch.py"
echo "  • List topics: ros2 topic list"
echo "  • Monitor topics: ros2 topic hz /scan"
echo "  • Test movement: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist ..."
echo ""

print_info "GUI applications should work with native X11 forwarding"
echo "  • RViz2: rviz2"
echo "  • Gazebo: gz sim"

echo ""
print_info "Starting container..."

# Execute the command
if eval $DOCKER_CMD; then
    print_success "Container started successfully!"
else
    print_error "Failed to start container!"
    echo ""
    print_info "Common issues:"
    echo "  • Docker not running (sudo systemctl start docker)"
    echo "  • Image not built (run ./buildSigynM4.sh first)"
    echo "  • Container name already in use (try --name different_name)"
    echo "  • Permission issues (sudo usermod -aG docker \$USER)"
    echo "  • Architecture mismatch (image: $IMAGE_ARCH, host: $(uname -m))"
    exit 1
fi

# Cleanup X11 permissions
xhost -local:docker 2>/dev/null || true