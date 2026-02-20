#!/bin/bash
# Run script for Sigyn AMD64 Docker container
# This script provides options for running the container

# Check if we're already inside a Docker container
check_if_in_docker() {
    # Quick check using multiple methods
    if [ -f /.dockerenv ] || \
       grep -q docker /proc/1/cgroup 2>/dev/null || \
       [ -n "$DOCKER_CONTAINER" ] || \
       [ -n "$container" ]; then
        return 0  # We are in Docker
    else
        return 1  # We are not in Docker
    fi
}

# Check for Docker-in-Docker situation
if check_if_in_docker; then
    echo "🐳 WARNING: You appear to be running inside a Docker container!"
    echo ""
    echo "This script is designed to run Docker containers from the host system."
    echo "Running Docker-in-Docker can cause issues and is generally not recommended."
    echo ""
    echo "💡 Common scenarios where this happens:"
    echo "   • Running from inside a VS Code Dev Container"
    echo "   • SSH'd into a Docker container"
    echo "   • Running from within another containerized environment"
    echo ""
    echo "🔍 If you want a detailed analysis of your container environment, run:"
    echo "     /home/ros/sigyn_ws/src/Sigyn/scripts/check_docker.sh"
    echo ""
    echo "🛠️  To proceed anyway, you have these options:"
    echo "   1. Exit this container and run the script from your host system (RECOMMENDED)"
    echo "   2. Use '--force' flag to override this warning (NOT RECOMMENDED)"
    echo "   3. Set up Docker socket mounting for proper Docker-in-Docker support"
    echo ""
    
    # Check if --force was provided
    if [[ ! " $* " =~ " --force " ]]; then
        echo "Exiting to prevent Docker-in-Docker issues."
        echo "Use '--force' if you really want to proceed."
        exit 1
    else
        echo "⚠️  --force flag detected. Proceeding with Docker-in-Docker (use at your own risk)."
        echo ""
    fi
fi

# Default values
GPU_SUPPORT="--gpus all"
CONTAINER_NAME="sigyn_container"
WORKSPACE_MOUNT=""
INTERACTIVE="-it"
REMOVE_AFTER_EXIT="--rm"
PRIVILEGED=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-gpu)
            GPU_SUPPORT=""
            shift
            ;;
        --mount-workspace)
            WORKSPACE_MOUNT="-v /home/ros/sigyn_ws:/home/ros/sigyn_ws"
            shift
            ;;
        --name)
            CONTAINER_NAME="$2"
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
        --force)
            # Force flag is handled above, just consume it here
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-gpu              Run without GPU support"
            echo "  --mount-workspace     Mount the host workspace directory"
            echo "  --name NAME           Set container name (default: sigyn_container)"
            echo "  --detached            Run in detached mode"
            echo "  --privileged          Run with privileged access (needed for some hardware)"
            echo "  --force               Force run even if already inside a Docker container (NOT RECOMMENDED)"
            echo "  --help                Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                           # Run with GPU, interactive mode"
            echo "  $0 --no-gpu                 # Run without GPU"
            echo "  $0 --mount-workspace        # Run with workspace mounted"
            echo "  $0 --name my_sigyn          # Run with custom name"
            echo "  $0 --detached --name sigyn  # Run in background"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Build the docker run command
DOCKER_CMD="docker run $GPU_SUPPORT $INTERACTIVE $REMOVE_AFTER_EXIT $PRIVILEGED --name $CONTAINER_NAME"

# Add workspace mount if specified
if [[ -n "$WORKSPACE_MOUNT" ]]; then
    # For mounted workspace, use entrypoint to map host user to ros user
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    
    DOCKER_CMD="$DOCKER_CMD $WORKSPACE_MOUNT"
    # Set working directory to workspace
    DOCKER_CMD="$DOCKER_CMD -w /home/ros/sigyn_ws"
    # Pass host UID/GID to entrypoint for user mapping
    DOCKER_CMD="$DOCKER_CMD -e USER_ID=$HOST_UID"
    DOCKER_CMD="$DOCKER_CMD -e GROUP_ID=$HOST_GID"
    DOCKER_CMD="$DOCKER_CMD -e ROS_DOMAIN_ID=0"
    
    echo "Running with host user mapping (UID:$HOST_UID, GID:$HOST_GID) via entrypoint"
    echo "Host user will be mapped to work as ros user with proper file permissions"
fi

# Add X11 forwarding for GUI applications (VS Code, RViz, Gazebo, etc.)
# Enable X11 forwarding automatically
if command -v xhost >/dev/null 2>&1; then
    echo "Enabling X11 forwarding for GUI applications..."
    xhost +local:docker >/dev/null 2>&1 || echo "Warning: Could not enable X11 forwarding"
fi

DOCKER_CMD="$DOCKER_CMD -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

# Add additional X11 and graphics support
DOCKER_CMD="$DOCKER_CMD -v /dev/dri:/dev/dri"
DOCKER_CMD="$DOCKER_CMD --device=/dev/dri:/dev/dri"

# Add GPU acceleration for GUI applications
DOCKER_CMD="$DOCKER_CMD -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all"

# Allow X11 forwarding (you may need to run 'xhost +local:docker' on host)
DOCKER_CMD="$DOCKER_CMD --env=QT_X11_NO_MITSHM=1"

# Add shared memory size for better performance
DOCKER_CMD="$DOCKER_CMD --shm-size=1g"

# Add network host for better ROS2 communication
DOCKER_CMD="$DOCKER_CMD --network=host"

# Add the image name
DOCKER_CMD="$DOCKER_CMD sigyn_amd"

echo "Running Sigyn container with command:"
echo "$DOCKER_CMD"
echo ""
echo "Note: For GUI applications (RViz2, Gazebo), you may need to run:"
echo "  xhost +local:docker"
echo "on the host system to allow X11 forwarding."
echo ""
echo "Known issues in simulation:"
echo "- Navigation velocity_smoother may fail (requires real-time settings)"
echo "- Controller spawners will timeout (hardware not available in sim)"
echo "- Some custom RViz overlays may not be available"
echo ""

# Execute the command
eval $DOCKER_CMD
