#!/bin/bash

# --- Configuration ---
DOCKERFILE="SigynVAmd"
IMAGE_TAG="sigyn_vamd"
HOST_ARCH=$(uname -m)
TARGET_PLATFORM="linux/amd64"

# --- Helper Functions ---
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo "❌ Docker is not installed. Please install Docker and try again."
        exit 1
    fi
    if ! docker info &> /dev/null; then
        echo "❌ Docker daemon is not running. Please start Docker and try again."
        exit 1
    fi
}

print_config() {
    echo "ℹ️  Build Configuration:"
    echo "  📁 Dockerfile: $DOCKERFILE"
    echo "  🏷️  Image tag: $IMAGE_TAG"
    echo "  🏗️  Host architecture: $HOST_ARCH"
    echo "  🎯 Target platform: $TARGET_PLATFORM (Emulated)"
    echo "  🚫 Cache: ${NO_CACHE_MSG}"
    echo ""
}

# --- Main Script ---
check_docker

# Default values
BUILD_ARGS=""
NO_CACHE_FLAG=""
NO_CACHE_MSG="Enabled"
VERBOSE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-cache)
            NO_CACHE_FLAG="--no-cache"
            NO_CACHE_MSG="Disabled"
            shift
            ;;
        --verbose)
            VERBOSE="true"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

print_config

read -p "Proceed with build? (Y/n): " confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] || -z $confirm ]] || exit 1

echo "ℹ️  Starting Docker build for AMD64..."
echo "   This will be significantly slower due to emulation."
echo ""

# Define build arguments as an array for safety. This prevents shell expansion issues.
DOCKER_BUILD_ARGS=(
    $NO_CACHE_FLAG
    $BUILD_ARGS
    --platform "$TARGET_PLATFORM"
    -t "$IMAGE_TAG"
    -f "$DOCKERFILE"
    .
)

# Remove any empty elements from the array
DOCKER_BUILD_ARGS=(${DOCKER_BUILD_ARGS[@]})

# Execute build command
if [[ "$VERBOSE" == "true" ]]; then
    echo "🔧 Build command: docker build ${DOCKER_BUILD_ARGS[@]}"
    docker build "${DOCKER_BUILD_ARGS[@]}"
else
    docker build "${DOCKER_BUILD_ARGS[@]}" > build_vamd.log 2>&1
fi

# Check build result
if [ $? -eq 0 ]; then
    echo "✅ Docker image '$IMAGE_TAG' built successfully."
    if [[ "$VERBOSE" != "true" ]]; then
        echo "   Log file saved to: build_vamd.log"
    fi
else
    echo "❌ Docker build failed!"
    if [[ "$VERBOSE" != "true" ]]; then
        echo "   Check the build_vamd.log for details."
        cat build_vamd.log
    fi
    exit 1
fi