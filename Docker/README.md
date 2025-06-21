# Sigyn Docker Setup

This directory contains Docker configuration and scripts for running the Sigyn robot simulation and development environment.

## Files

- `DockerSigynAmd` - Dockerfile for Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic
- `runSigynAmd.sh` - Main script to run the Docker container
- `buildSigynAmd.sh` - Script to build the Docker image
- `entrypoint.sh` - Container entry point script
- Other legacy files for different configurations

## Quick Start

1. **Build the Docker image:**
   ```bash
   ./buildSigynAmd.sh
   ```

2. **Run the container:**
   ```bash
   ./runSigynAmd.sh
   ```

## Run Script Options

The `runSigynAmd.sh` script supports several options:

- `--no-gpu` - Run without GPU support
- `--mount-workspace` - Mount the host workspace directory
- `--name NAME` - Set custom container name
- `--detached` - Run in detached mode
- `--privileged` - Run with privileged access (needed for some hardware)
- `--force` - Force run even if already inside a Docker container (NOT RECOMMENDED)
- `--help` - Show help message

## Docker-in-Docker Detection

The run script automatically detects if you're already running inside a Docker container to prevent Docker-in-Docker issues. If detected, it will:

1. Show a warning with explanation
2. Suggest running from the host system instead
3. Provide option to force execution with `--force` flag

For detailed container environment analysis, use:
```bash
/home/ros/sigyn_ws/src/Sigyn/scripts/check_docker.sh
```

## GUI Applications

The container is configured for X11 forwarding to support GUI applications like:
- Gazebo simulation
- RViz2 visualization
- VS Code (if needed)

The run script automatically enables X11 forwarding and sets up GPU acceleration.

## Common Issues

1. **GUI applications not displaying:** Ensure X11 forwarding is enabled on host
2. **Permission errors with mounted workspace:** Use `--mount-workspace` for proper permissions
3. **Docker-in-Docker warnings:** Exit current container and run from host system

## Environment

- Base OS: Ubuntu 24.04 LTS
- ROS Version: ROS 2 Jazzy
- Gazebo Version: Gazebo Harmonic
- GPU Support: NVIDIA (via nvidia-docker2)
