#!/bin/bash

# Build comprehensive Sigyn development environment with workspace dependencies
# This image includes all packages discovered from workspace analysis

set -e

echo "Building comprehensive Sigyn development environment..."

# Get build metadata
BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ')
VCS_REF=$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")

echo "Build date: $BUILD_DATE"
echo "VCS ref: $VCS_REF"

# Build with legacy builder for better compatibility
DOCKER_BUILDKIT=0 docker build \
    -f Sigyn/Dockerfile.v3 \
    -t sigyn-dev-v3:comprehensive \
    --build-arg BUILD_DATE="$BUILD_DATE" \
    --build-arg VCS_REF="$VCS_REF" \
    .

echo "Build complete!"
echo "Image: sigyn-dev-v3:comprehensive"
echo "Features:"
echo "  ✓ ROS2 Jazzy with CycloneDDS"
echo "  ✓ All workspace ROS2 dependencies (nav2, cartographer, behavior trees, etc.)"
echo "  ✓ Gazebo Harmonic (gz-* packages)"
echo "  ✓ Comprehensive development aliases"
echo "  ✓ VS Code, development tools"
echo "  ✓ Workspace auto-detection"
echo "  ✓ User/group ID mapping support"
echo ""
echo "Usage:"
echo "  # Run with workspace mount:"
echo "  docker run --rm -it -v /home/ros/sigyn_ws:/workspace sigyn-dev-v3:comprehensive"
echo ""
echo "  # Run with full setup (networking, devices, etc.):"
echo "  cd ../runSigynV3Comprehensive.sh"
