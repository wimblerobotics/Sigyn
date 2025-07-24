# Sigyn Robot Development Docker Environment

This directory contains a comprehensive Docker development environment for the Sigyn house patroller robot. The environment includes ROS2 Jazzy, Gazebo Harmonic, Arduino IDE with Teensyduino, Visual Studio Code, and all necessary dependencies for robot development, simulation, and Teensy programming.

## 🏗️ Architecture

The Docker setup is designed to be **workspace-agnostic**, meaning it doesn't assume any specific workspace structure inside the container. Instead, you mount your development workspace from the host system, allowing you to work with different projects and sync with GitHub as needed.

### Key Features

- **ROS2 Jazzy Desktop Full** - Complete ROS2 development environment
- **Gazebo Harmonic** - Robot simulation with GPU acceleration support
- **Arduino IDE 2.x + Teensyduino** - Teensy 4.1 development environment
- **Visual Studio Code** - Full IDE with extensions support
- **PlatformIO** - Advanced Arduino/Teensy development
- **Python Development** - Complete Python ecosystem with robotics packages
- **Hardware Access** - USB device support for Teensy programming and robot communication
- **X11 Forwarding** - GUI applications work seamlessly
- **User Mapping** - Proper file permissions when mounting host directories

## 📁 Directory Structure

```
Docker/Sigyn/
├── Dockerfile          # Main container definition
├── build.sh            # Build script with options
├── run.sh              # Comprehensive run script
├── README.md           # This documentation
├── DEVELOPMENT.md      # Development workflows and tips
└── TROUBLESHOOTING.md  # Common issues and solutions
```

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
