#!/bin/bash
# Script to detect if we're running inside a Docker container
# Multiple methods are used for reliability

check_docker() {
    local in_docker=false
    local detection_methods=()
    
    echo "Checking if running inside Docker container..."
    echo "================================================="
    
    # Method 1: Check for .dockerenv file
    if [ -f /.dockerenv ]; then
        echo "✓ Found /.dockerenv file"
        detection_methods+=("dockerenv")
        in_docker=true
    else
        echo "✗ No /.dockerenv file found"
    fi
    
    # Method 2: Check cgroup for docker
    if [ -f /proc/1/cgroup ]; then
        if grep -q docker /proc/1/cgroup 2>/dev/null; then
            echo "✓ Found 'docker' in /proc/1/cgroup"
            detection_methods+=("cgroup")
            in_docker=true
        elif grep -q "/docker/" /proc/1/cgroup 2>/dev/null; then
            echo "✓ Found '/docker/' path in /proc/1/cgroup"
            detection_methods+=("cgroup-path")
            in_docker=true
        else
            echo "✗ No docker references in /proc/1/cgroup"
        fi
    else
        echo "✗ No /proc/1/cgroup file found"
    fi
    
    # Method 3: Check for container-specific environment variables
    if [ -n "$DOCKER_CONTAINER" ] || [ -n "$container" ]; then
        echo "✓ Found container environment variable"
        detection_methods+=("env-var")
        in_docker=true
    else
        echo "✗ No container environment variables found"
    fi
    
    # Method 4: Check init process (PID 1)
    if [ -f /proc/1/comm ]; then
        init_process=$(cat /proc/1/comm 2>/dev/null)
        if [ "$init_process" = "docker-init" ] || [ "$init_process" = "tini" ]; then
            echo "✓ Init process is container-related: $init_process"
            detection_methods+=("init-process")
            in_docker=true
        else
            echo "ℹ Init process: $init_process"
        fi
    fi
    
    # Method 5: Check hostname format (Docker containers often have hex hostnames)
    hostname=$(hostname)
    if [[ "$hostname" =~ ^[0-9a-f]{12}$ ]]; then
        echo "✓ Hostname looks like Docker container ID: $hostname"
        detection_methods+=("hostname")
        in_docker=true
    else
        echo "ℹ Hostname: $hostname"
    fi
    
    # Method 6: Check for Docker-specific mount points
    if mount | grep -q "on / type overlay"; then
        echo "✓ Found overlay filesystem on root (typical for Docker)"
        detection_methods+=("overlay-fs")
        in_docker=true
    else
        echo "ℹ Root filesystem is not overlay"
    fi
    
    echo "================================================="
    
    if [ "$in_docker" = true ]; then
        echo "🐳 RESULT: You are running inside a Docker container"
        echo "   Detection methods: ${detection_methods[*]}"
        echo ""
        echo "💡 To avoid running Docker-in-Docker, you should:"
        echo "   1. Exit this container first"
        echo "   2. Run the Docker commands from your host system"
        echo "   3. Or use Docker socket mounting if you need Docker inside Docker"
        return 0
    else
        echo "🖥️  RESULT: You are NOT running inside a Docker container"
        echo "   Safe to run Docker commands directly"
        return 1
    fi
}

# Function to show container info if inside Docker
show_container_info() {
    if [ -f /.dockerenv ]; then
        echo ""
        echo "📋 Container Information:"
        echo "========================"
        
        # Show container ID if available
        if [ -f /proc/self/cgroup ]; then
            container_id=$(grep docker /proc/self/cgroup | head -1 | sed 's/.*docker\/\([^\/]*\).*/\1/' | cut -c1-12)
            if [ -n "$container_id" ] && [ "$container_id" != "docker" ]; then
                echo "Container ID: $container_id"
            fi
        fi
        
        # Show hostname
        echo "Hostname: $(hostname)"
        
        # Show some environment variables
        echo "User: $(whoami)"
        echo "Working directory: $(pwd)"
        
        # Show if we have Docker socket access
        if [ -S /var/run/docker.sock ]; then
            echo "Docker socket: Available (Docker-in-Docker possible)"
        else
            echo "Docker socket: Not available"
        fi
        
        # Show mounted volumes
        echo ""
        echo "Mounted volumes:"
        mount | grep -E "(bind|volume)" | while read line; do
            echo "  $line"
        done
    fi
}

# Main execution
check_docker
exit_code=$?

if [ $exit_code -eq 0 ]; then
    show_container_info
fi

exit $exit_code
