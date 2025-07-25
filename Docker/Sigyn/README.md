# Sigyn Docker Development Environment

This directory contains the comprehensive Docker development environment for the Sigyn house patroller robot.

## 🏗️ Current Status

**IMPORTANT**: This directory now contains only the working v3 comprehensive environment. All obsolete files have been cleaned up.

## 📁 Files

- `Dockerfile.v3` - The working comprehensive development environment with ROS2 Jazzy, CycloneDDS, Nav2, Gazebo Harmonic, and all workspace dependencies
- `README.md` - This file

## 🚀 Quick Start

Use the comprehensive v3 environment from the parent Docker directory:

### Build the Environment
```bash
cd /home/ros/sigyn_ws/src/Sigyn/Docker
./buildSigynV3Comprehensive.sh
```

### Run the Environment  
```bash
cd /home/ros/sigyn_ws/src/Sigyn/Docker
./runSigynV3Comprehensive.sh
```

## ✨ What's Included

- **ROS2 Jazzy Desktop Full** with CycloneDDS RMW
- **Navigation Stack**: Nav2, SLAM Toolbox, Cartographer
- **Simulation**: Gazebo Harmonic with physics simulation
- **Behavior Trees**: py_trees and nav2_behavior_tree
- **Control**: ros2_control, joint_state_publisher
- **Perception**: OpenCV, PCL, sensor packages
- **Development Tools**: 30+ aliases, enhanced tab completion
- **All Workspace Dependencies**: Systematically discovered from package.xml files

## 🔧 Configuration

The environment includes:
- CycloneDDS configured for `enp37s0` network interface
- Development aliases (`cb` for colcon build, `cs` for colcon source, etc.)
- Proper user ID mapping for file permissions
- Hardware device access for robot communication

## � Notes

- This replaces all previous Docker setups (v1, v2, minimal, runtime, test versions)
- The comprehensive environment includes all dependencies needed for Sigyn development
- Legacy documentation files may reference obsolete scripts - use the v3 comprehensive setup instead

For detailed documentation about the Sigyn robot itself, see the main project documentation in the `Documentation/` directory.

## 🚀 Quick Start

### 1. Build the Docker Image

```bash
cd Docker/Sigyn
./build.sh
```

Build options:
```bash
./build.sh --no-cache          # Build without cache
./build.sh --tag my-sigyn-dev  # Custom tag name
./build.sh --help              # Show all options
```

### 2. Run the Container

**Basic Usage (no workspace mounted):**
```bash
./run.sh
```

**Development Mode (with workspace):**
```bash
./run.sh --dev --workspace-path ~/sigyn_ws
```

**Simulation Mode:**
```bash
./run.sh --sim --workspace-path ~/sigyn_ws
```

**Hardware Development:**
```bash
./run.sh --hardware --workspace-path ~/sigyn_ws
```

## 📋 Usage Examples

### Sigyn Robot Development

```bash
# Full development environment with Sigyn workspace
./run.sh --dev --workspace-path ~/sigyn_ws

# Inside container:
cd /workspace  # Your sigyn_ws is mounted here
cb             # Build workspace (colcon build --symlink-install)
sim            # Launch simulation
nav            # Launch navigation
arduino        # Launch Arduino IDE
code .         # Launch VS Code
```

### Working with Different Workspaces

```bash
# Work with a different ROS2 workspace
./run.sh --mount-workspace --workspace-path ~/my_other_ws --gpu

# Work with Teensy development only
./run.sh --hardware --workspace-path ~/teensy_projects
```

### Background Development Server

```bash
# Start persistent container
./run.sh --detached --name sigyn-server --workspace-path ~/sigyn_ws

# Connect to it later
docker exec -it sigyn-server bash
```

## 🛠️ Run Script Options

```bash
Usage: ./run.sh [OPTIONS] [COMMAND]

OPTIONS:
  --gpu                   Enable GPU support (for Gazebo, RViz, etc.)
  --mount-workspace       Mount a workspace directory (requires --workspace-path)
  --workspace-path PATH   Host workspace path to mount
  --name NAME             Set container name (default: sigyn-dev-container)
  --tag TAG               Use specific image tag (default: sigyn-dev-v3)
  --detached              Run in detached mode (background)
  --privileged            Run with privileged access (for hardware devices)
  --no-remove             Don't remove container on exit (persistent)
  --network NETWORK       Set network mode (default: host)
  --device DEVICE         Add device access (e.g., /dev/ttyUSB0)
  --volume SRC:DST        Add volume mount
  --env VAR=VALUE         Set environment variable
  --force                 Force run even if already inside Docker
  --help                  Show help message

DEVELOPMENT SHORTCUTS:
  --dev                   Enable development mode (requires --workspace-path)
  --sim                   Enable simulation mode (GPU, no hardware devices)
  --hardware              Enable hardware mode (privileged, devices mounted)
```

## 🔧 Pre-configured Aliases

The container comes with numerous aliases for Sigyn development:

### Colcon Build
- `cb` - `colcon build --symlink-install`
- `cbt` - `colcon build --symlink-install --packages-select`
- `cbu` - `colcon build --symlink-install --packages-up-to`
- `cbf` - `colcon build --symlink-install --continue-on-error`

### ROS2 Utilities
- `rd` - `rosdep install --from-paths src --ignore-src -r -y`
- `fr` - `ros2 run tf2_tools view_frames`
- `tele` - Teleop with reduced speed
- `stele` - Teleop for keyboard input

### Sigyn Launch Commands
- `sim` - Launch Sigyn in simulation mode
- `nav` - Launch Sigyn navigation
- `map` - Launch mapping mode
- `sensor` - Launch sensor interface

### Development Tools
- `arduino` - Launch Arduino IDE
- `code` - Launch VS Code (with --no-sandbox)
- `rvs` - Launch RViz with Sigyn config (workspace-aware)

### Teensy Development
- `compileBoard1/2` - Compile Teensy firmware
- `buildBoard1/2` - Compile and flash Teensy firmware
- `setup_venv` - Activate Python virtual environment

## 🌐 Network Configuration

The container uses host networking by default for optimal ROS2 communication. DDS is configured for CycloneDX with automatic network interface detection.

## 🖥️ GUI Applications

GUI applications (RViz, Gazebo, Arduino IDE, VS Code) work through X11 forwarding:

1. X11 forwarding is automatically enabled by the run script
2. GPU acceleration is available with `--gpu` flag
3. Applications run natively with full performance

## 🔌 Hardware Access

### USB Devices (Teensy, Arduino)

```bash
# Automatic device mounting
./run.sh --hardware --workspace-path ~/sigyn_ws

# Manual device access
./run.sh --device /dev/ttyACM0 --workspace-path ~/sigyn_ws
```

### udev Rules

The container automatically copies udev rules from your workspace:
- Checks `/workspace/src/Sigyn/udev/` for `*.rules` files
- Copies them to `/etc/udev/rules.d/`
- Restarts udev service

## 🔄 Updates and Maintenance

### Updating the Container

```bash
# Rebuild with latest packages
./build.sh --no-cache

# Update specific workspace dependencies
./run.sh --workspace-path ~/sigyn_ws
# Inside container:
sudo apt update && sudo apt upgrade
pip install --upgrade <packages>
```

### Ubuntu Package Management

The container includes package management utilities:
- `apt` for system packages
- `pip` for Python packages  
- `rosdep` for ROS dependencies
- `arduino-cli` for Arduino libraries

### Persistent Changes

- **Temporary**: Changes made in `--rm` containers are lost on exit
- **Persistent**: Use `--no-remove` to keep containers between sessions
- **Workspace**: All workspace changes are persistent (mounted from host)

## 🐛 Troubleshooting

### Common Issues

1. **Permission Errors**: Ensure proper user mapping with workspace mounting
2. **GUI Not Working**: Check X11 forwarding and `xhost` permissions
3. **Device Access**: Use `--privileged` or specific `--device` flags
4. **Build Failures**: Try `--no-cache` to rebuild from scratch

### Debug Commands

```bash
# Check container status
docker ps -a

# View container logs
docker logs sigyn-dev-container

# Inspect image
docker inspect sigyn-dev

# Connect to running container
docker exec -it sigyn-dev-container bash
```

See `TROUBLESHOOTING.md` for detailed solutions.

## 📚 Additional Documentation

- `DEVELOPMENT.md` - Detailed development workflows and best practices
- `TROUBLESHOOTING.md` - Common issues and solutions
- Container logs and debugging information

## 🤝 Contributing

When adding new features or packages to the Docker environment:

1. Update the Dockerfile with new dependencies
2. Add relevant aliases to the bashrc section
3. Update this documentation
4. Test with `./build.sh --no-cache`

## 📝 Notes

- Container starts in `/home/ros` by default
- Workspace is mounted at `/workspace` when using `--mount-workspace`
- All GUI applications work with X11 forwarding
- Hardware devices are accessible with proper flags
- udev rules are automatically copied from workspace
