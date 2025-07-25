# Sigyn Docker Development Environment

This directory contains Docker configurations for the Sigyn robot development environment with comprehensive ROS2 Jazzy support.

## Available Configurations

### Comprehensive Development Environment (Recommended)
- **Image**: `sigyn-dev-v3:comprehensive`
- **Features**: Complete ROS2 Jazzy with all workspace dependencies
- **Size**: ~5.9GB
- **Includes**: Navigation, Cartographer, Behavior Trees, Gazebo Harmonic, ros2_control, VS Code

### Quick Start - Comprehensive Environment

1. **Build the comprehensive development environment:**
   ```bash
   ./buildSigynV3Comprehensive.sh
   ```

2. **Run with full hardware access:**
   ```bash
   ./runSigynV3Comprehensive.sh
   ```

### Legacy Configurations
- `buildSigynAmd.sh` / `runSigynAmd.sh` - Basic Ubuntu 24.04 setup
- Other build scripts for different hardware targets

## Comprehensive Environment Features

### ROS2 Packages Included (70+ packages)
- **Navigation**: nav2_*, cartographer, robot_localization
- **Control**: ros2_control, ros2_controllers, hardware_interface
- **Simulation**: Gazebo Harmonic (gz-* packages)
- **Behavior**: behaviortree_cpp with full nav2 integration
- **Perception**: cv_bridge, image_transport, vision_opencv
- **Communication**: CycloneDDS middleware configured

### Development Tools
- **IDE**: VS Code with extensions
- **Debugging**: gdb, valgrind, comprehensive toolchain
- **Build**: colcon with tab completion
- **Monitoring**: htop, system tools

### 30+ Sigyn-Specific Aliases
- `cb` - colcon build --symlink-install
- `nav` - launch navigation stack
- `sim` - launch simulation
- `gripper` - hardware interface controls
- `nodes`, `topics`, `services` - ROS2 introspection
- Plus debugging, Teensy development, and system shortcuts

## Configuration Details

### Workspace Auto-Detection
Automatically finds and sources workspaces from:
- `/workspace` (primary mount point)
- `/sigyn_ws`, `/home/ros/ws`, `/ws`, `/colcon_ws`

### CycloneDDS Configuration
- **RMW**: rmw_cyclonedds_cpp
- **Network**: enp37s0 interface configured
- **Multicast**: Enabled for multi-robot communication

### User Management
- **User**: ros (uid:1000) with sudo access
- **ID Mapping**: Automatic user/group ID mapping from host
- **Permissions**: Hardware device access configured

## Usage Examples

### Basic Development (Default Workspace)
```bash
# Start with default workspace (/home/ros/sigyn_ws)
./runSigynV3Comprehensive.sh

# Inside container - auto-detects workspace
cb  # Build workspace with symlink install
nav # Launch navigation stack
```

### Custom Workspace Support
```bash
# Mount any workspace directory
./runSigynV3Comprehensive.sh ~/my_other_ws
./runSigynV3Comprehensive.sh /path/to/any/ros2_workspace

# Show all options
./runSigynV3Comprehensive.sh --help
```

### Testing Your Build
```bash
# Quick validation test
docker run --rm -it -v /home/ros/sigyn_ws:/workspace \
  sigyn-dev-v3:comprehensive bash -c \
  "source ~/.bashrc && alias cb && echo 'RMW:' && echo \$RMW_IMPLEMENTATION"
```

## GUI Applications

The container supports X11 forwarding for GUI applications:
- **Gazebo Harmonic**: gz sim simulation
- **RViz2**: Robot visualization and debugging
- **VS Code**: Full development environment
- **rqt**: ROS2 debugging tools

## Troubleshooting

### Common Issues
- **No workspace found**: Ensure `/home/ros/sigyn_ws` exists and is built
- **Permission issues**: Check user/group ID mapping in run script
- **GUI not working**: Verify X11 forwarding and DISPLAY variable

### Docker Images Management
```bash
# List Sigyn images
docker images | grep sigyn

# Remove old images
docker image prune -f

# Rebuild comprehensive environment
./buildSigynV3Comprehensive.sh
```

## Legacy Support

For older or alternative configurations:
- `buildSigynAmd.sh` - Basic AMD64 setup
- `runSigynAmd.sh` - Simple container execution
- See individual scripts for specific options

## Development

The comprehensive environment is built from `Sigyn/Dockerfile.v3` which includes:
- Systematic workspace dependency analysis
- Gazebo Harmonic (not Classic) support
- CycloneDDS network configuration
- Complete development toolchain
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
