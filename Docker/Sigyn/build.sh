#!/bin/bash
# Build script for Sigyn Robot Development Environment
# This script builds a comprehensive Docker image with ROS2 Jazzy, Gazebo, Arduino IDE, and VSCode
#
# Usage: ./build.sh [OPTIONS]
#
# OPTIONS:
#   --no-cache     Build without using cache
#   --platform     Target platform (default: linux/amd64)
#   --tag          Custom tag name (default: sigyn-dev)
#   --help         Show this help message

set -e

# Default values
IMAGE_TAG="sigyn-dev-v4"
PLATFORM="linux/amd64"
BUILD_ARGS=""
DOCKER_BUILD_ARGS=""
DOCKERFILE="Dockerfile.v4"

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

# Function to show usage
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Build the Sigyn Robot Development Environment Docker image"
    echo ""
    echo "OPTIONS:"
    echo "  --no-cache     Build without using Docker cache"
    echo "  --platform     Target platform (default: linux/amd64)"
    echo "  --tag          Custom tag name (default: sigyn-dev)"
    echo "  --help         Show this help message"
    echo ""
    echo "EXAMPLES:"
    echo "  $0                                    # Build with default settings"
    echo "  $0 --no-cache                        # Build without cache"
    echo "  $0 --tag my-sigyn-dev               # Build with custom tag"
    echo "  $0 --platform linux/arm64           # Build for ARM64"
    echo ""
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cache)
            DOCKER_BUILD_ARGS="${DOCKER_BUILD_ARGS} --no-cache"
            shift
            ;;
        --platform)
            PLATFORM="$2"
            shift 2
            ;;
        --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed or not in PATH"
    exit 1
fi

# Check if we're in the correct directory
if [ ! -f "$DOCKERFILE" ]; then
    print_error "$DOCKERFILE not found. Please run this script from the Docker/Sigyn directory"
    exit 1
fi

# Print build information
print_info "Building Sigyn Robot Development Environment"
print_info "Platform: ${PLATFORM}"
print_info "Image tag: ${IMAGE_TAG}"
print_info "Build args: ${DOCKER_BUILD_ARGS}"
echo ""

# Get build metadata
BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ')
VCS_REF=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

# Build the Docker image
print_info "Starting Docker build..."
echo ""

BUILD_CMD="docker build \
    --platform ${PLATFORM} \
    --build-arg BUILD_DATE=${BUILD_DATE} \
    --build-arg VCS_REF=${VCS_REF} \
    --tag ${IMAGE_TAG} \
    ${DOCKER_BUILD_ARGS} \
    -f ${DOCKERFILE} \
    ."

print_info "Executing: ${BUILD_CMD}"
echo ""

# Execute the build
if eval ${BUILD_CMD}; then
    print_success "Docker image built successfully!"
    echo ""
    print_info "Image details:"
    docker images ${IMAGE_TAG} --format "table {{.Repository}}\t{{.Tag}}\t{{.ID}}\t{{.CreatedAt}}\t{{.Size}}"
    echo ""
    
    print_info "To run the container:"
    echo "  ./run.sh"
    echo ""
    print_info "To run with workspace mounted:"
    echo "  ./run.sh --mount-workspace"
    echo ""
    print_info "To run with GPU support:"
    echo "  ./run.sh --gpu"
    echo ""
    print_info "For all options:"
    echo "  ./run.sh --help"
    
else
    print_error "Docker build failed!"
    exit 1
fi
