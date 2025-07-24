# Sigyn Development Workflows

This document provides detailed workflows for developing the Sigyn robot using the Docker environment.

## 🏗️ Development Setup

### Initial Setup

1. **Clone or sync your workspace:**
   ```bash
   git clone <your-sigyn-repo> ~/sigyn_ws
   # OR sync existing workspace
   cd ~/sigyn_ws && git pull
   ```

2. **Build the Docker image:**
   ```bash
   cd ~/sigyn_ws/src/Sigyn/Docker/Sigyn
   ./build.sh
   ```

3. **Start development environment:**
   ```bash
   ./run.sh --dev --workspace-path ~/sigyn_ws
   ```

### Daily Development Workflow

```bash
# Start container (from Docker/Sigyn directory)
./run.sh --dev --workspace-path ~/sigyn_ws

# Inside container - you're now in /workspace (your mounted sigyn_ws)
cb              # Build workspace
source install/setup.bash  # Source built packages
```

## 🤖 Robot Development Workflows

### 1. Simulation Development

```bash
# Start simulation-ready container
./run.sh --sim --workspace-path ~/sigyn_ws

# Inside container:
cb              # Build workspace
sim             # Launch simulation
# New terminal: docker exec -it sigyn-dev-container bash
tele            # Control robot with keyboard
rvs             # Launch RViz with Sigyn config
```

### 2. Hardware Development

```bash
# Start hardware-ready container
./run.sh --hardware --workspace-path ~/sigyn_ws

# Inside container:
cb              # Build workspace
nav             # Launch navigation stack
# Hardware should be automatically accessible
sensor          # Launch sensor interface
```

### 3. Navigation Development

```bash
# Start development container
./run.sh --dev --workspace-path ~/sigyn_ws

# Inside container:
cb              # Build workspace

# For mapping:
map             # Launch mapping mode
tele            # Drive robot to create map
savem           # Save the map

# For navigation:
nav             # Launch navigation
# Use RViz to set navigation goals
```

## 🔌 Teensy Development Workflows

### 1. Teensy Programming Setup

```bash
# Start container with hardware access
./run.sh --hardware --workspace-path ~/sigyn_ws

# Inside container:
cd src/Sigyn/Teensy  # Or wherever your Teensy code is

# Using PlatformIO (recommended):
compileBoard1    # Compile board 1 firmware
buildBoard1      # Compile and flash board 1

# Using Arduino IDE:
arduino          # Launch Arduino IDE GUI
```

### 2. Teensy Code Development

```bash
# Edit code on host system with your preferred editor
# OR use VS Code inside container:
code src/Sigyn/Teensy/teensy_board1/

# Compile and test:
compileBoard1    # Check for compilation errors
buildBoard1      # Flash to Teensy (requires physical connection)

# Test communication:
sensor           # Launch ROS2 sensor node
# New terminal:
ros2 topic echo /imu0  # Check IMU data
```

### 3. Multi-Board Development

```bash
# For projects with multiple Teensy boards:
compileBoard1 && compileBoard2  # Compile both
buildBoard1                     # Flash board 1
# Swap USB connection or use different port
buildBoard2                     # Flash board 2

# Test both boards:
sensor           # Launch sensor interface
ros2 topic list  # Verify both boards are communicating
```

## 🧪 Testing Workflows

### 1. Unit Testing

```bash
# Build with tests
cb --packages-select <package-name>

# Run tests
ros2 run <package-name> <test-executable>
# OR
colcon test --packages-select <package-name>
colcon test-result --verbose
```

### 2. Integration Testing

```bash
# Launch full system in simulation
sim

# In new terminal:
docker exec -it sigyn-dev-container bash
cd /workspace

# Run integration tests
ros2 run <test-package> <integration-test>
```

### 3. Hardware-in-the-Loop Testing

```bash
# Start with hardware access
./run.sh --hardware --workspace-path ~/sigyn_ws

# Launch sensor systems
sensor

# Run hardware tests
ros2 run <package-name> <hardware-test>
```

## 🔧 Debugging Workflows

### 1. ROS2 Node Debugging

```bash
# Launch with GDB
ros2 run --prefix 'gdb --args' <package> <node>

# OR with debug symbols
cb --cmake-args -DCMAKE_BUILD_TYPE=Debug
ros2 run <package> <node>
```

### 2. Network Debugging

```bash
# Check ROS2 discovery
ros2 node list
ros2 topic list
ros2 service list

# Monitor topic data
ros2 topic echo /cmd_vel
ros2 topic hz /scan

# Check TF tree
fr              # Generate and view TF frames
```

### 3. Hardware Debugging

```bash
# Check USB devices
lsusb

# Check serial ports
ls -la /dev/tty*

# Test serial communication
minicom -D /dev/ttyACM0  # Direct serial connection

# Check udev rules
cat /etc/udev/rules.d/*teensy*
redoudev        # Restart udev service
```

## 🎯 Specialized Workflows

### 1. Computer Vision Development

```bash
# Start with GPU support
./run.sh --dev --gpu --workspace-path ~/sigyn_ws

# Inside container:
cb --packages-select oakd_detector
ros2 launch oakd_detector detection.launch.py

# New terminal for visualization:
rvs             # View camera feeds and detections
```

### 2. Behavior Tree Development

```bash
# Edit behavior trees
code src/Sigyn/sigyn_behavior_trees/

# Test behavior trees
cb --packages-select sigyn_behavior_trees
ros2 launch sigyn_behavior_trees test_bt.launch.py

# Debug with Groot (if installed)
groot           # Behavior tree visualizer
```

### 3. Custom Message Development

```bash
# Edit message definitions
code src/Sigyn/sigyn_interfaces/

# Rebuild interface packages first
cb --packages-select sigyn_interfaces

# Rebuild dependent packages
cbu <dependent-package>  # Build up to dependent package
```

## 🔄 Continuous Development

### 1. Multi-Terminal Development

```bash
# Terminal 1: Main development
./run.sh --dev --workspace-path ~/sigyn_ws

# Terminal 2: Monitoring
docker exec -it sigyn-dev-container bash
htop            # Monitor system resources

# Terminal 3: Testing
docker exec -it sigyn-dev-container bash
# Run tests while developing

# Terminal 4: Hardware monitoring
docker exec -it sigyn-dev-container bash
ros2 topic echo /diagnostics
```

### 2. Iterative Development

```bash
# Make code changes on host
# Inside container:
cbt <package>   # Build only changed package
# Test immediately - no need to restart container
```

### 3. Git Workflow

```bash
# All git operations on host system (outside container)
cd ~/sigyn_ws
git add .
git commit -m "Description"
git push

# Container automatically sees changes through mount
```

## 📊 Performance Optimization

### 1. Build Optimization

```bash
# Parallel builds
cb --parallel-workers 4

# Symlink install (already default in aliases)
cb --symlink-install

# Build only what changed
cbt <specific-package>
```

### 2. Runtime Optimization

```bash
# Set CPU governor (on host)
sudo cpupower frequency-set --governor performance

# Monitor resource usage
htop            # Inside container
docker stats    # On host
```

### 3. Memory Management

```bash
# Increase shared memory
./run.sh --shm-size=2g --workspace-path ~/sigyn_ws

# Monitor memory usage
free -h
ros2 topic hz /rosout  # Check for memory warnings
```

## 🚀 Advanced Workflows

### 1. Multiple Workspace Development

```bash
# Work on different projects
./run.sh --workspace-path ~/ros2_ws --name ros2-dev
./run.sh --workspace-path ~/sigyn_ws --name sigyn-dev
./run.sh --workspace-path ~/research_ws --name research-dev
```

### 2. Cross-Compilation

```bash
# Build for different architectures (if needed)
./build.sh --platform linux/arm64
./run.sh --tag sigyn-dev-arm64 --workspace-path ~/sigyn_ws
```

### 3. Custom Environment Variables

```bash
# Development with custom settings
./run.sh --env ROS_DOMAIN_ID=42 \
         --env CYCLONEDDS_URI=/path/to/config.xml \
         --workspace-path ~/sigyn_ws
```

## 📝 Best Practices

1. **Always mount workspace**: Use `--workspace-path` for any real development
2. **Use symlink install**: Already configured in aliases, ensures fast rebuilds
3. **Leverage aliases**: Learn and use the pre-configured aliases for efficiency
4. **Multiple terminals**: Use `docker exec` for additional terminals in same container
5. **Git on host**: Do all version control operations on the host system
6. **Hardware testing**: Use `--hardware` mode when working with physical devices
7. **GPU for visualization**: Use `--gpu` when running Gazebo or RViz intensively
8. **Persistent containers**: Use `--no-remove` for long development sessions

## 🔧 Customization

### Adding Your Own Aliases

Edit the Dockerfile to add custom aliases, or add them to your host `.bashrc` and they'll be available when you shell into containers.

### Custom Package Installation

```bash
# Inside container (temporary):
sudo apt install <package>
pip install <package>

# Permanent: Edit Dockerfile and rebuild
```

### IDE Integration

```bash
# Use VS Code inside container
code .          # Launch from workspace root

# Or use host IDE with container as remote
# Configure your IDE to connect to running container
```
