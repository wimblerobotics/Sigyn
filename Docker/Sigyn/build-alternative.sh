#!/bin/bash
# Alternative build script for network connectivity issues
# This script tries different approaches to build the Docker image

set -e

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

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed or not in PATH"
    exit 1
fi

print_info "Attempting to build Sigyn Docker image with network-resilient approach..."

# Try building with minimal Dockerfile first
print_info "Step 1: Testing minimal build..."
if docker build -t sigyn-dev-minimal -f Dockerfile.minimal .; then
    print_success "Minimal build succeeded!"
    print_info "You can use this image with: ./run.sh --tag sigyn-dev-minimal"
    echo ""
    
    print_info "Step 2: Attempting full build..."
    if docker build -t sigyn-dev -f Dockerfile .; then
        print_success "Full build succeeded!"
        print_info "You can use the full image with: ./run.sh"
    else
        print_warning "Full build failed, but minimal image is available"
        print_info "Use: ./run.sh --tag sigyn-dev-minimal"
        print_info "You can install additional packages manually inside the container"
    fi
else
    print_error "Even minimal build failed. Checking network connectivity..."
    
    # Test network connectivity
    print_info "Testing network connectivity..."
    if ping -c 1 8.8.8.8 > /dev/null 2>&1; then
        print_info "Network connectivity: OK"
    else
        print_error "Network connectivity: FAILED"
        print_info "Please check your internet connection and try again"
        exit 1
    fi
    
    # Test Docker registry access
    print_info "Testing Docker registry access..."
    if docker pull hello-world > /dev/null 2>&1; then
        print_info "Docker registry access: OK"
        docker rmi hello-world > /dev/null 2>&1
    else
        print_error "Cannot access Docker registry"
        print_info "You may need to configure Docker proxy settings"
        exit 1
    fi
    
    print_error "Build failed despite network connectivity"
    print_info "This may be a temporary issue with package repositories"
    print_info "Try running the build again in a few minutes"
    exit 1
fi

print_success "Build process completed!"
print_info ""
print_info "Available images:"
docker images | grep sigyn-dev
print_info ""
print_info "To run the container:"
print_info "  ./run.sh --workspace-path ~/sigyn_ws --dev"
print_info ""
print_info "To run with minimal image:"
print_info "  ./run.sh --tag sigyn-dev-minimal --workspace-path ~/sigyn_ws --dev"
