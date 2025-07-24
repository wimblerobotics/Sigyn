# Troubleshooting Guide

This document covers common issues and solutions when using the Sigyn Docker development environment.

## 🔧 Build Issues

### Docker Build Fails

**Problem**: Build fails with package installation errors
```bash
E: Package 'ros-jazzy-*' has no installation candidate
```

**Solution**:
```bash
# Rebuild without cache to get latest packages
./build.sh --no-cache

# Check Docker daemon status
sudo systemctl status docker
sudo systemctl restart docker
```

**Problem**: Build fails with "no space left on device"
```bash
# Clean up Docker space
docker system prune -a
docker volume prune

# Check available space
df -h
```

### Arduino/Teensy Installation Issues

**Problem**: Arduino IDE download fails or Teensy support missing

**Solution**:
```bash
# Rebuild with different Arduino versions
# Edit Dockerfile and change version numbers:
ENV ARDUINO_IDE_VERSION=2.3.1
ENV ARDUINO_CLI_VERSION=0.35.2

./build.sh --no-cache
```

## 🖥️ GUI Issues

### X11 Forwarding Not Working

**Problem**: GUI applications don't display or show X11 errors

**Solution**:
```bash
# On host system:
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# If using SSH, enable X11 forwarding:
ssh -X username@hostname
```

**Problem**: "cannot connect to X server" error

**Solution**:
```bash
# Restart X11 server
sudo systemctl restart gdm  # or lightdm/xdm

# Allow X11 connections
xauth list
xhost +SI:localuser:ros
```

### GPU Acceleration Issues

**Problem**: Gazebo/RViz performance is poor

**Solution**:
```bash
# Ensure NVIDIA drivers are installed on host
nvidia-smi

# Install NVIDIA Docker runtime
sudo apt install nvidia-docker2
sudo systemctl restart docker

# Run with GPU support
./run.sh --gpu --workspace-path ~/sigyn_ws
```

**Problem**: "NVIDIA-SMI has failed" in container

**Solution**:
```bash
# Check Docker can access GPU
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# If fails, reinstall NVIDIA Docker support
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update && sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

## 🔌 Hardware Access Issues

### USB Device Access

**Problem**: Teensy/Arduino devices not visible in container

**Solution**:
```bash
# Check devices on host
lsusb
ls -la /dev/tty*

# Run with hardware mode
./run.sh --hardware --workspace-path ~/sigyn_ws

# Or add specific device
./run.sh --device /dev/ttyACM0 --workspace-path ~/sigyn_ws

# Check device permissions
sudo usermod -a -G dialout $USER
# Logout and login again
```

**Problem**: Permission denied when accessing /dev/ttyACM0

**Solution**:
```bash
# On host system:
sudo chmod 666 /dev/ttyACM0
# OR better, fix udev rules:
sudo cp ~/sigyn_ws/src/Sigyn/udev/*.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### udev Rules Not Working

**Problem**: Device names like /dev/teensy_sensor not appearing

**Solution**:
```bash
# Check if rules are copied
docker exec -it sigyn-dev-container bash
ls -la /etc/udev/rules.d/

# Manually copy rules if needed
sudo cp /workspace/src/Sigyn/udev/*.rules /etc/udev/rules.d/
redoudev  # Restart udev service

# Check rule syntax
udevadm test $(udevadm info -q path -n /dev/ttyACM0)
```

## 🌐 Network Issues

### ROS2 Discovery Problems

**Problem**: `ros2 node list` shows no nodes or incomplete list

**Solution**:
```bash
# Check ROS2 environment
echo $ROS_DOMAIN_ID
echo $RMW_IMPLEMENTATION

# Use host networking (default in run script)
./run.sh --network host --workspace-path ~/sigyn_ws

# Check firewall
sudo ufw status
sudo ufw allow from 224.0.0.0/4  # Allow multicast

# Test with simple publisher/subscriber
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
```

**Problem**: DDS communication slow or unreliable

**Solution**:
```bash
# Try different DDS implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# OR
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Configure Cyclone DDS
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General></Domain></CycloneDX>"
```

### Container Networking

**Problem**: Cannot reach external services from container

**Solution**:
```bash
# Test DNS resolution
docker exec -it sigyn-dev-container nslookup google.com

# Check network configuration
docker exec -it sigyn-dev-container ip route
docker exec -it sigyn-dev-container ip addr

# Use bridge networking if host networking fails
./run.sh --network bridge --workspace-path ~/sigyn_ws
```

## 📁 File System Issues

### Permission Problems

**Problem**: Cannot write to mounted workspace

**Solution**:
```bash
# Check user mapping
docker exec -it sigyn-dev-container id

# Ensure proper UID/GID mapping
./run.sh --mount-workspace --workspace-path ~/sigyn_ws
# Run script automatically sets USER_ID and GROUP_ID

# Manual fix if needed
sudo chown -R $(id -u):$(id -g) ~/sigyn_ws
```

**Problem**: Files created in container have wrong ownership

**Solution**:
```bash
# The entrypoint script should handle this automatically
# If not working, check entrypoint execution:
docker exec -it sigyn-dev-container env | grep USER_ID

# Manual fix:
docker exec -it sigyn-dev-container sudo chown -R ros:ros /workspace
```

### Workspace Mount Issues

**Problem**: Workspace not mounted or empty

**Solution**:
```bash
# Check mount point inside container
docker exec -it sigyn-dev-container ls -la /workspace

# Verify workspace path exists on host
ls -la ~/sigyn_ws

# Check run command
./run.sh --mount-workspace --workspace-path ~/sigyn_ws
# Note: --mount-workspace is required with --workspace-path
```

## 🏗️ Build and Development Issues

### Colcon Build Failures

**Problem**: Build fails with "package not found" errors

**Solution**:
```bash
# Install dependencies
rd  # rosdep install

# Check package.xml files
find src -name package.xml -exec xmllint {} \;

# Clean build
rm -rf build install log
cb  # Rebuild everything
```

**Problem**: Symlink install not working

**Solution**:
```bash
# Verify symlink option (should be default in aliases)
colcon build --symlink-install

# Check if symlinks are created
ls -la install/lib/python3.12/site-packages/
```

### ROS2 Package Issues

**Problem**: Package not found after building

**Solution**:
```bash
# Source the workspace
source install/setup.bash

# Check package is installed
ros2 pkg list | grep <package-name>

# Verify package.xml and CMakeLists.txt
cat src/<package>/package.xml
```

**Problem**: Python packages not importing

**Solution**:
```bash
# Check Python path
python3 -c "import sys; print(sys.path)"

# Verify setup.py or pyproject.toml
ls src/<package>/setup.py

# Rebuild with --symlink-install
cb --packages-select <package>
```

## 🤖 Arduino/Teensy Issues

### Compilation Failures

**Problem**: Arduino code won't compile

**Solution**:
```bash
# Check Arduino CLI installation
arduino-cli version
arduino-cli core list

# Update cores and libraries
arduino-cli core update-index
arduino-cli core upgrade

# Use PlatformIO instead
cd src/Sigyn/TeensyV2  # or your Teensy project
pio run -e board1
```

**Problem**: Teensy libraries missing

**Solution**:
```bash
# Install Teensy libraries manually
arduino-cli lib install "Teensy:4.1"

# Or use PlatformIO
pio lib install <library-name>
```

### Upload Issues

**Problem**: Cannot upload to Teensy

**Solution**:
```bash
# Check device visibility
lsusb | grep -i teensy

# Use privileged mode
./run.sh --privileged --workspace-path ~/sigyn_ws

# Try manual upload
teensy_loader_cli --mcu=TEENSY41 -w -v firmware.hex

# Check for programming button press
echo "Press and release the programming button on Teensy"
```

## 📊 Performance Issues

### Slow Container Performance

**Problem**: Container runs slowly

**Solution**:
```bash
# Increase shared memory
./run.sh --shm-size=2g --workspace-path ~/sigyn_ws

# Check CPU/memory usage
docker stats
htop  # inside container

# Use SSD storage if available
# Move workspace to SSD location
```

**Problem**: Gazebo/RViz lag

**Solution**:
```bash
# Enable GPU acceleration
./run.sh --gpu --workspace-path ~/sigyn_ws

# Reduce graphics quality in Gazebo
# Edit world files to reduce complexity

# Check graphics drivers
glxinfo | grep -i vendor  # inside container
```

## 🔍 Debug Commands

### Container Inspection

```bash
# Check container status
docker ps -a

# View container details
docker inspect sigyn-dev-container

# Check logs
docker logs sigyn-dev-container

# Get container IP
docker inspect sigyn-dev-container | grep IPAddress
```

### System Information

```bash
# Inside container diagnostics
uname -a
lsb_release -a
ros2 doctor  # ROS2 system check
env | grep ROS
```

### Network Diagnostics

```bash
# Test connectivity
ping google.com
nslookup github.com

# ROS2 network check
ros2 multicast receive
ros2 multicast send

# DDS discovery
ros2 daemon stop
ros2 daemon start
```

## 🆘 Emergency Procedures

### Complete Reset

**If everything is broken:**

```bash
# Stop all containers
docker stop $(docker ps -q)

# Remove all containers
docker rm $(docker ps -aq)

# Remove image
docker rmi sigyn-dev

# Clean up everything
docker system prune -a

# Rebuild from scratch
./build.sh --no-cache
```

### Recovery Commands

**If container won't start:**

```bash
# Try minimal container
docker run -it --rm ubuntu:24.04 bash

# Check Docker daemon
sudo systemctl status docker
sudo journalctl -u docker

# Restart Docker service
sudo systemctl restart docker
```

### Data Recovery

**If workspace appears corrupted:**

```bash
# Check file system
ls -la ~/sigyn_ws
du -sh ~/sigyn_ws

# Git status
cd ~/sigyn_ws
git status
git log --oneline -10

# Restore from backup or git
git checkout -- .
git clean -fd
```

## 📞 Getting Help

1. **Check Docker logs**: `docker logs sigyn-dev-container`
2. **Verify host system**: Ensure Docker, X11, and drivers work
3. **Test minimal case**: Try with basic `docker run ubuntu:24.04`
4. **Check documentation**: Review README.md and DEVELOPMENT.md
5. **Search issues**: Look for similar problems in Docker/ROS communities

## 📋 Diagnostic Checklist

Before reporting issues, gather this information:

```bash
# Host system info
uname -a
docker version
docker info

# Container info (if running)
docker exec -it sigyn-dev-container uname -a
docker exec -it sigyn-dev-container ros2 doctor

# Error messages
docker logs sigyn-dev-container 2>&1 | tail -50

# Environment
env | grep -E "(DISPLAY|ROS|DOCKER)"
```
