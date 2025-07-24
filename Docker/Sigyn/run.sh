#!/bin/bash
# Run script for Sigyn Robot Development Environment
# This script provides comprehensive options for running the Docker container
# with proper device access, workspace mounting, and development tools
#
# Usage: ./run.sh [OPTIONS]

set -e

# Default configuration
IMAGE_TAG="sigyn-dev-v4"
CONTAINER_NAME="sigyn-dev-container"
WORKSPACE_HOST_PATH=""  # No default - user must specify
WORKSPACE_CONTAINER_PATH="/workspace"  # Standard mount point

# Runtime options
GPU_SUPPORT=""
MOUNT_WORKSPACE=""
INTERACTIVE="-it"
REMOVE_AFTER_EXIT="--rm"
PRIVILEGED=""
DETACHED=""
NETWORK="--network=host"
EXTRA_ARGS=""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if we're inside Docker
check_if_in_docker() {
    if [ -f /.dockerenv ] || \
       grep -q docker /proc/1/cgroup 2>/dev/null || \
       [ -n "$DOCKER_CONTAINER" ] || \
       [ -n "$container" ]; then
        return 0  # We are in Docker
    else
        return 1  # We are not in Docker
    fi
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS] [COMMAND]"
    echo ""
    echo "Run the Sigyn Robot Development Environment Docker container"
    echo ""
    echo "OPTIONS:"
    echo "  --gpu                   Enable GPU support (for Gazebo, RViz, etc.)"
    echo "  --mount-workspace       Mount a workspace directory (must specify --workspace-path)"
    echo "  --workspace-path PATH   Host workspace path to mount (required with --mount-workspace)"
    echo "  --name NAME             Set container name (default: sigyn-dev-container)"
    echo "  --tag TAG               Use specific image tag (default: sigyn-dev)"
    echo "  --detached              Run in detached mode (background)"
    echo "  --privileged            Run with privileged access (for hardware devices)"
    echo "  --no-remove             Don't remove container on exit (persistent)"
    echo "  --network NETWORK       Set network mode (default: host)"
    echo "  --device DEVICE         Add device access (e.g., /dev/ttyUSB0)"
    echo "  --volume SRC:DST        Add volume mount"
    echo "  --env VAR=VALUE         Set environment variable"
    echo "  --force                 Force run even if already inside Docker"
    echo "  --help                  Show this help message"
    echo ""
    echo "DEVELOPMENT SHORTCUTS:"
    echo "  --dev                   Enable development mode (requires --workspace-path)"
    echo "  --dev-gpu               Enable development mode with GPU support (requires --workspace-path)"
    echo "  --sim                   Enable simulation mode (GPU, no hardware devices)"
    echo "  --hardware              Enable hardware mode (privileged, devices mounted)"
    echo ""
    echo "EXAMPLES:"
    echo "  $0                                         # Basic interactive container"
    echo "  $0 --mount-workspace --workspace-path ~/my_ws --gpu  # Development with specific workspace"
    echo "  $0 --workspace-path ~/sigyn_ws --dev      # Full development with Sigyn workspace"
    echo "  $0 --workspace-path ~/sigyn_ws --dev-gpu  # Development with GPU support"
    echo "  $0 --detached --name sigyn-bg            # Background container"
    echo "  $0 --sim                                  # Simulation-only mode"
    echo "  $0 --hardware --workspace-path ~/sigyn_ws # Hardware development mode"
    echo "  $0 --device /dev/ttyUSB0             # Add specific device"
    echo "  $0 bash                              # Run bash directly"
    echo "  $0 'ros2 topic list'                 # Run specific command"
    echo ""
    echo "COMMON COMMANDS:"
    echo "  $0 --dev 'cb'                        # Build workspace in container"
    echo "  $0 --dev-gpu 'rviz2'                 # Launch RViz2 with GPU support"
    echo "  $0 --dev 'arduino-ide'               # Launch Arduino IDE"
    echo "  $0 --dev 'code .'                    # Launch VS Code"
    echo ""
}

# Parse command line arguments
COMMAND_ARGS=()
while [[ $# -gt 0 ]]; do
    case $1 in
        --gpu)
            GPU_SUPPORT="--gpus all"
            shift
            ;;
        --mount-workspace)
            if [[ -z "${WORKSPACE_HOST_PATH}" ]]; then
                print_error "--mount-workspace requires --workspace-path to be specified"
                exit 1
            fi
            MOUNT_WORKSPACE="true"
            shift
            ;;
        --workspace-path)
            WORKSPACE_HOST_PATH="$2"
            shift 2
            ;;
        --name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --detached)
            INTERACTIVE="-d"
            DETACHED="true"
            REMOVE_AFTER_EXIT=""
            shift
            ;;
        --privileged)
            PRIVILEGED="--privileged"
            shift
            ;;
        --no-remove)
            REMOVE_AFTER_EXIT=""
            shift
            ;;
        --network)
            NETWORK="--network=$2"
            shift 2
            ;;
        --device)
            EXTRA_ARGS="${EXTRA_ARGS} --device $2"
            shift 2
            ;;
        --volume)
            EXTRA_ARGS="${EXTRA_ARGS} -v $2"
            shift 2
            ;;
        --env)
            EXTRA_ARGS="${EXTRA_ARGS} -e $2"
            shift 2
            ;;
        --dev)
            # Development mode: workspace mounted, privileged (no GPU by default)
            if [[ -z "${WORKSPACE_HOST_PATH}" ]]; then
                print_error "--dev mode requires --workspace-path to be specified"
                print_info "Example: $0 --dev --workspace-path ~/sigyn_ws"
                exit 1
            fi
            MOUNT_WORKSPACE="true"
            PRIVILEGED="--privileged"
            shift
            ;;
        --dev-gpu)
            # Development mode with GPU: workspace mounted, GPU, privileged
            if [[ -z "${WORKSPACE_HOST_PATH}" ]]; then
                print_error "--dev-gpu mode requires --workspace-path to be specified"
                print_info "Example: $0 --dev-gpu --workspace-path ~/sigyn_ws"
                exit 1
            fi
            MOUNT_WORKSPACE="true"
            GPU_SUPPORT="--gpus all"
            PRIVILEGED="--privileged"
            shift
            ;;
        --sim)
            # Simulation mode: GPU support for visualization
            GPU_SUPPORT="--gpus all"
            shift
            ;;
        --hardware)
            # Hardware mode: privileged access and common devices
            PRIVILEGED="--privileged"
            # Add common Teensy/Arduino devices
            EXTRA_ARGS="${EXTRA_ARGS} --device /dev/ttyACM0:/dev/ttyACM0"
            EXTRA_ARGS="${EXTRA_ARGS} --device /dev/ttyUSB0:/dev/ttyUSB0"
            # Add udev for device management
            EXTRA_ARGS="${EXTRA_ARGS} -v /run/udev:/run/udev:ro"
            shift
            ;;
        --force)
            # Force flag will be checked later
            shift
            ;;
        --help)
            show_usage
            exit 0
            ;;
        --)
            shift
            COMMAND_ARGS+=("$@")
            break
            ;;
        -*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            # Remaining arguments are the command to run
            COMMAND_ARGS+=("$1")
            shift
            ;;
    esac
done

# Check for Docker-in-Docker situation
if check_if_in_docker; then
    print_warning "You appear to be running inside a Docker container!"
    echo ""
    print_info "This script is designed to run Docker containers from the host system."
    print_info "Running Docker-in-Docker can cause issues and is generally not recommended."
    echo ""
    print_info "If you want to proceed anyway, use the --force flag."
    print_info "Otherwise, exit this container and run the script from your host system."
    echo ""
    
    # Check if --force was provided in the original arguments
    if [[ ! " $* " =~ " --force " ]]; then
        print_error "Exiting to prevent Docker-in-Docker issues."
        print_info "Use '--force' if you really want to proceed."
        exit 1
    else
        print_warning "--force flag detected. Proceeding with Docker-in-Docker (use at your own risk)."
        echo ""
    fi
fi

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed or not in PATH"
    exit 1
fi

# Check if image exists
if ! docker images -q ${IMAGE_TAG} > /dev/null; then
    print_error "Docker image '${IMAGE_TAG}' not found"
    print_info "Please build the image first using: ./build.sh"
    exit 1
fi

# Check for existing container with the same name
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    print_warning "Container '${CONTAINER_NAME}' already exists"
    print_info "Removing existing container..."
    docker rm -f ${CONTAINER_NAME} > /dev/null
fi

# Build the docker run command
DOCKER_CMD="docker run"

# Add basic options
DOCKER_CMD="${DOCKER_CMD} ${INTERACTIVE} ${REMOVE_AFTER_EXIT} ${PRIVILEGED}"
DOCKER_CMD="${DOCKER_CMD} --name ${CONTAINER_NAME}"
DOCKER_CMD="${DOCKER_CMD} ${NETWORK}"

# Add GPU support if requested
if [[ -n "${GPU_SUPPORT}" ]]; then
    DOCKER_CMD="${DOCKER_CMD} ${GPU_SUPPORT}"
    # Add NVIDIA environment variables
    DOCKER_CMD="${DOCKER_CMD} -e NVIDIA_VISIBLE_DEVICES=all"
    DOCKER_CMD="${DOCKER_CMD} -e NVIDIA_DRIVER_CAPABILITIES=all"
fi

# Add workspace mount if requested
if [[ "${MOUNT_WORKSPACE}" == "true" ]]; then
    if [[ -z "${WORKSPACE_HOST_PATH}" ]]; then
        print_error "Workspace mounting requested but no workspace path specified"
        print_info "Use --workspace-path to specify the host workspace directory"
        exit 1
    fi
    
    if [[ ! -d "${WORKSPACE_HOST_PATH}" ]]; then
        print_error "Workspace path '${WORKSPACE_HOST_PATH}' does not exist"
        print_info "Please ensure the workspace directory exists on the host"
        exit 1
    fi
    
    DOCKER_CMD="${DOCKER_CMD} -v ${WORKSPACE_HOST_PATH}:${WORKSPACE_CONTAINER_PATH}"
    DOCKER_CMD="${DOCKER_CMD} -w ${WORKSPACE_CONTAINER_PATH}"
    
    # Add host user mapping for proper file permissions
    HOST_UID=$(id -u)
    HOST_GID=$(id -g)
    DOCKER_CMD="${DOCKER_CMD} -e USER_ID=${HOST_UID}"
    DOCKER_CMD="${DOCKER_CMD} -e GROUP_ID=${HOST_GID}"
    
    print_info "Workspace mounted: ${WORKSPACE_HOST_PATH} -> ${WORKSPACE_CONTAINER_PATH}"
    print_info "User mapping: UID=${HOST_UID}, GID=${HOST_GID}"
fi

# Add X11 forwarding for GUI applications
if command -v xhost >/dev/null 2>&1; then
    print_info "Enabling X11 forwarding for GUI applications..."
    xhost +local:docker >/dev/null 2>&1 || print_warning "Could not enable X11 forwarding"
fi

DOCKER_CMD="${DOCKER_CMD} -e DISPLAY=${DISPLAY}"
DOCKER_CMD="${DOCKER_CMD} -v /tmp/.X11-unix:/tmp/.X11-unix:rw"

# Add graphics support (only if not using GPU support to avoid conflicts)
if [[ -z "${GPU_SUPPORT}" ]]; then
    DOCKER_CMD="${DOCKER_CMD} -v /dev/dri:/dev/dri"
    if [ -c /dev/dri/renderD128 ]; then
        DOCKER_CMD="${DOCKER_CMD} --device=/dev/dri:/dev/dri"
    fi
fi

# Add shared memory for better performance
DOCKER_CMD="${DOCKER_CMD} --shm-size=1g"

# Add any extra arguments
if [[ -n "${EXTRA_ARGS}" ]]; then
    DOCKER_CMD="${DOCKER_CMD} ${EXTRA_ARGS}"
fi

# Add ROS2 environment variables
DOCKER_CMD="${DOCKER_CMD} -e ROS_DOMAIN_ID=0"

# Add the image name
DOCKER_CMD="${DOCKER_CMD} ${IMAGE_TAG}"

# Add command if specified
if [[ ${#COMMAND_ARGS[@]} -gt 0 ]]; then
    DOCKER_CMD="${DOCKER_CMD} ${COMMAND_ARGS[*]}"
fi

# Display information
print_info "Starting Sigyn Development Container"
print_info "Image: ${IMAGE_TAG}"
print_info "Container: ${CONTAINER_NAME}"
if [[ -n "${GPU_SUPPORT}" ]]; then
    print_info "GPU: Enabled"
fi
if [[ "${MOUNT_WORKSPACE}" == "true" ]]; then
    print_info "Workspace: Mounted"
fi
if [[ -n "${PRIVILEGED}" ]]; then
    print_info "Privileged: Enabled"
fi
echo ""

print_info "Command: ${DOCKER_CMD}"
echo ""

# Execute the command
print_info "Launching container..."
eval ${DOCKER_CMD}

# Cleanup
if command -v xhost >/dev/null 2>&1; then
    xhost -local:docker >/dev/null 2>&1 || true
fi

print_success "Container finished."
