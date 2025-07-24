# Sigyn Comprehensive Development Environment

## Overview
The comprehensive Sigyn development environment provides a complete ROS2 Jazzy setup with all dependencies discovered from your workspace analysis.

## Features
- **Base**: Ubuntu 22.04 with ROS2 Jazzy Desktop Full
- **Middleware**: CycloneDDS configured for enp37s0 interface
- **Navigation**: Complete Nav2 stack with behavior trees
- **Mapping**: Cartographer and SLAM Toolbox
- **Simulation**: Gazebo Harmonic (gz-* packages) with ROS2 integration
- **Control**: ros2_control framework with all controllers
- **Development**: VS Code, comprehensive toolchain, debugging tools
- **Aliases**: 30+ Sigyn-specific development aliases
- **Auto-detection**: Automatic workspace sourcing from common locations

## ROS2 Packages Included
Based on workspace analysis, includes 70+ packages:
- behaviortree_cpp, nav2_*, cartographer_ros
- ros2_control, ros2_controllers, hardware_interface
- gz-ros2-control, ros-gz-* (Gazebo Harmonic)
- cv_bridge, image_transport, vision_opencv
- tf2, xacro, robot_state_publisher
- rmw_cyclonedds_cpp and CycloneDDS
- All message packages (geometry_msgs, sensor_msgs, etc.)

## Usage

### Build
```bash
cd /home/ros/sigyn_ws/src/Sigyn/Docker
./buildSigynV3Comprehensive.sh
```

### Run
```bash
# Full development environment
./runSigynV3Comprehensive.sh

# Simple workspace mount
docker run --rm -it -v /home/ros/sigyn_ws:/workspace sigyn-dev-v3:comprehensive
```

### Key Aliases Available
- `cb` - colcon build --symlink-install
- `nav` - launch navigation
- `sim` - launch simulation
- `gripper` - launch gripper interface
- `nodes` - list ROS2 nodes
- `topics` - list ROS2 topics
- Plus 25+ more development shortcuts

## Configuration
- **RMW**: rmw_cyclonedds_cpp
- **Network**: enp37s0 interface configured
- **User**: ros (uid:1000) with sudo access
- **Workspace**: Auto-detects /workspace, /sigyn_ws, etc.
- **Entrypoint**: Smart user/group ID mapping

## Size: 5.88GB
Comprehensive but optimized for development efficiency.

## Testing
```bash
# Verify aliases and workspace detection
docker run --rm -it -v /home/ros/sigyn_ws:/workspace sigyn-dev-v3:comprehensive \
  bash -c "source ~/.bashrc && alias cb && echo 'Workspace:' && ls /workspace | head -3"
```
