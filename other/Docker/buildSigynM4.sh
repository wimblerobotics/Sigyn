#!/bin/bash
# Build script for Sigyn M4 (Apple Silicon) Docker container
# This script builds the Docker image for ARM64/aarch64 architecture

set -e  # Exit on any error

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

# Check if we're on ARM64/aarch64 (for Apple Silicon in VM)
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

# Check Docker installation
check_docker() {
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
}

# Default values
DOCKERFILE="SigynM4"
IMAGE_TAG="sigyn_m4"
BUILD_ARGS=""
NO_CACHE=""
VERBOSE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --dockerfile)
            DOCKERFILE="$2"
            shift 2
            ;;
        --tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        --verbose)
            VERBOSE="--progress=plain"
            shift
            ;;
        --build-arg)
            BUILD_ARGS="$BUILD_ARGS --build-arg $2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --dockerfile FILE     Specify Dockerfile (default: SigynM4)"
            echo "  --tag TAG            Set image tag (default: sigyn_m4)"
            echo "  --no-cache           Build without using cache"
            echo "  --verbose            Verbose build output"
            echo "  --build-arg KEY=VAL  Pass build argument"
            echo "  --help               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Basic build"
            echo "  $0 --no-cache                        # Clean build"
            echo "  $0 --tag my_sigyn_m4                 # Custom tag"
            echo "  $0 --build-arg ROS_DISTRO=jazzy      # Custom ROS version"
            echo ""
            echo "Architecture Support:"
            echo "  • Designed for ARM64/aarch64 (Apple Silicon in VM)"
            echo "  • Builds linux/arm64 Docker images"
            echo "  • X11 forwarding supported (no XQuartz needed in VM)"
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
print_info "Starting Sigyn M4 Docker build process..."
echo ""

check_architecture
check_docker

# Check if Dockerfile exists
if [[ ! -f "$DOCKERFILE" ]]; then
    print_error "Dockerfile '$DOCKERFILE' not found!"
    echo ""
    echo "Please ensure you're running this script from the directory containing the Dockerfile."
    echo "Available files:"
    ls -la | grep -E "(Dockerfile|Sigyn)"
    exit 1
fi

print_success "Pre-flight checks passed"
echo ""

# Display build configuration
print_info "Build Configuration:"
echo "  📁 Dockerfile: $DOCKERFILE"
echo "  🏷️  Image tag: $IMAGE_TAG"
echo "  🏗️  Host architecture: $(uname -m)"
echo "  🐳 Target platform: linux/arm64"
echo "  🐳 Docker version: $(docker --version)"
if [[ -n "$BUILD_ARGS" ]]; then
    echo "  ⚙️  Build args: $BUILD_ARGS"
fi
if [[ -n "$NO_CACHE" ]]; then
    echo "  🚫 Cache: Disabled"
fi
echo ""

# Confirm build
read -p "Proceed with build? (Y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then
    print_info "Build cancelled by user"
    exit 0
fi

# Start build
print_info "Starting Docker build..."
echo ""

BUILD_START_TIME=$(date +%s)

# Build the Docker image
BUILD_COMMAND="docker build $NO_CACHE $VERBOSE $BUILD_ARGS --platform linux/arm64 -t $IMAGE_TAG -f $DOCKERFILE ."

print_info "Build command: $BUILD_COMMAND"
echo ""

if eval $BUILD_COMMAND; then
    BUILD_END_TIME=$(date +%s)
    BUILD_DURATION=$((BUILD_END_TIME - BUILD_START_TIME))
    
    echo ""
    print_success "Docker image built successfully! 🎉"
    echo ""
    print_info "Build Summary:"
    echo "  🏷️  Image: $IMAGE_TAG"
    echo "  ⏱️  Duration: ${BUILD_DURATION}s"
    echo "  📦 Size: $(docker images $IMAGE_TAG --format "table {{.Size}}" | tail -n1)"
    echo "  🏗️  Architecture: $(docker image inspect $IMAGE_TAG --format '{{.Architecture}}')"
    echo ""
    
    print_info "Next steps:"
    echo "  1. Run the container: ./runSigynM4.sh"
    echo "  2. Or run with workspace: ./runSigynM4.sh --mount-workspace"
    echo "  3. For help: ./runSigynM4.sh --help"
    echo ""
    
    print_info "Available images:"
    docker images | grep -E "(REPOSITORY|$IMAGE_TAG)"
    
else
    print_error "Docker build failed!"
    echo ""
    print_info "Common issues on ARM64:"
    echo "  • Make sure Docker is running"
    echo "  • Ensure you have enough disk space"
    echo "  • Check if the base image supports ARM64"
    echo "  • Try building with --no-cache flag"
    echo "  • Verify Docker permissions (try: sudo usermod -aG docker \$USER)"
    echo ""
    exit 1
fi