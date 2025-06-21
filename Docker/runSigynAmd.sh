#!/bin/bash
# Run script for Sigyn AMD64 Docker container
# This script provides options for running the container

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
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-gpu              Run without GPU support"
            echo "  --mount-workspace     Mount the host workspace directory"
            echo "  --name NAME           Set container name (default: sigyn_container)"
            echo "  --detached            Run in detached mode"
            echo "  --privileged          Run with privileged access (needed for some hardware)"
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
    # Option 1: Map user IDs (may cause issues with different usernames)
    # HOST_UID=$(id -u)
    # HOST_GID=$(id -g)
    # DOCKER_CMD="$DOCKER_CMD $WORKSPACE_MOUNT --user $HOST_UID:$HOST_GID"
    
    # Option 2: Mount and fix permissions inside container (simpler)
    DOCKER_CMD="$DOCKER_CMD $WORKSPACE_MOUNT"
    echo "Note: You may need to run 'sudo chown -R ros:ros /home/ros/sigyn_ws' inside the container"
fi

# Add X11 forwarding for GUI applications (VS Code, RViz, Gazebo, etc.)
DOCKER_CMD="$DOCKER_CMD -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

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
