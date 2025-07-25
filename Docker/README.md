# Sigyn Docker Development Environment

This directory contains Docker configurations for the Sigyn robot development environment with comprehensive ROS2 Jazzy support.

## Available Configurations

### Comprehensive Development Environment (Recommended)
- **Image**: `sigyn-dev-v3:comprehensive`
- **Features**: Complete ROS2 Jazzy with all workspace dependencies
- **Size**: ~5.9GB
- **Includes**: Navigation, Cartographer, Behavior Trees, Gazebo Harmonic, ros2_control, VS Code, Arduino IDE, PlatformIO

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
- **Arduino/Teensy**: Arduino IDE and PlatformIO with Teensy platform support
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

### Arduino/Teensy Development
The environment includes full Arduino IDE and PlatformIO support for Teensy development:

```bash
# Create a new Teensy 4.1 project
mkdir my_teensy_project && cd my_teensy_project
pio project init --board teensy41

# Available tools
arduino-ide          # Launch Arduino IDE GUI
pio --version         # PlatformIO CLI
platformio --version  # PlatformIO full version

# Build and upload (with Teensy connected)
pio run              # Compile project
pio run --target upload  # Upload to Teensy

# Available Teensy boards in PlatformIO
pio boards teensy    # List all supported Teensy boards
```

**Installed Teensy Tools:**
- Arduino IDE with Teensy support
- PlatformIO with Teensy platform (5.0.0)
- Teensy toolchains (AVR and ARM)
- Framework Arduino for Teensy (1.159.0)

### Sigyn TeensyV2 Development
For working with the existing Sigyn TeensyV2 project:

```bash
# Navigate to TeensyV2 project directory
cd /workspace/TeensyV2

# Compile board1 (Navigation & Safety board)
pio run -e board1

# Compile board2 (Power & Sensors board)
pio run -e board2

# Compile and upload to connected Teensy
pio run -e board1 --target upload
pio run -e board2 --target upload

# Debug builds with additional logging
pio run -e board1_debug
pio run -e board2_debug

# Monitor serial output
pio device monitor --environment board1
pio device monitor --environment board2

# Clean and rebuild
pio run -e board1 --target clean
pio run -e board1

# Build all environments
pio run
```

**TeensyV2 Board Configurations:**
- **Board1**: Navigation & Safety (RoboClaw motor control, VL53L0X sensors, temperature monitoring)
- **Board2**: Power & Sensors (Battery monitoring, BNO055 IMU, power management)
- Both boards include performance monitoring and safety coordination

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
- **Arduino IDE**: Teensy and Arduino development
- **rqt**: ROS2 debugging tools

## Software Updates

### Docker Image Updates
The Docker image contains pre-installed system packages that are **not automatically updated**. To get software updates:

```bash
# Rebuild the image to get latest packages
./buildSigynV3Comprehensive.sh

# Or update an existing container (temporary until container restart)
sudo apt update && sudo apt upgrade -y
```

### Persistent vs Ephemeral Changes
- **Ephemeral**: Changes made inside a running container (like `apt install`, `pip install`) are **lost when container stops**
- **Persistent**: Only files in mounted volumes (`/workspace`, etc.) persist between container sessions
- **Permanent**: Changes must be added to `Dockerfile.v3` and the image rebuilt

### System Package Management
```bash
# Inside container - temporary until restart
sudo apt update
sudo apt upgrade -y
sudo apt install new-package

# For permanent changes - edit Dockerfile.v3 and rebuild
./buildSigynV3Comprehensive.sh
```

### Preserving Container Changes
You can save changes from a running container to a new image:

```bash
# Find your running container ID
docker ps

# Commit changes to a new image (from host, not inside container)
docker commit <container_id> sigyn-dev-v3:custom-$(date +%Y%m%d)
docker commit <container_id> sigyn-dev-v3:comprehensive  # Overwrite current

# Example with meaningful tag
docker commit abc123def456 sigyn-dev-v3:with-arduino-tools
```

**⚠️ Important Notes:**
- **Best Practice**: Add changes to `Dockerfile.v3` and rebuild for reproducibility
- **Container commits**: Create larger, less efficient images
- **Version control**: Dockerfile changes can be tracked in Git
- **Team sharing**: Dockerfile changes are easier to share than custom images

**Recommended Workflow:**
1. Test changes interactively in container
2. Document working commands
3. Add to `Dockerfile.v3` 
4. Rebuild image with `./buildSigynV3Comprehensive.sh`
5. Share Dockerfile changes via Git

### Recommended Update Strategy
1. **For development**: Use the container as-is, install temporary tools as needed
2. **For production**: Regularly rebuild images to get security updates
3. **For new tools**: Add to `Dockerfile.v3` for permanent installation

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

# Clean up custom/tagged images
docker images | grep sigyn-dev-v3
docker rmi sigyn-dev-v3:custom-20250124  # Remove specific custom image
docker rmi sigyn-dev-v3:with-arduino-tools  # Remove named custom image

# Save/load custom images for backup or sharing
docker save sigyn-dev-v3:custom-20250124 | gzip > sigyn-custom.tar.gz
docker load < sigyn-custom.tar.gz
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
