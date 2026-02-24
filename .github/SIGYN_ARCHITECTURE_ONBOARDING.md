# Sigyn Architecture Onboarding Guide

## 1. Project Overview
Sigyn is a ROS 2 Jazzy-based service robot designed for house patrolling, surveillance, and light manipulation tasks. It uses a differential drive base, an elevator mechanism, and a parallel gripper.

**Core Tech Stack:**
- **OS**: Linux (Ubuntu 24.04 recommended)
- **ROS Distro**: Jazzy Jalisco
- **Simulation**: Gazebo Fortress
- **Navigation**: Nav2
- **Behavior**: BehaviorTree.CPP (v4)

## 2. Key Operational Commands

### Build
Always use symlink install to allow script updates without rebuilding.
```bash
colcon build --symlink-install
```

### Simulation (Gazebo)
Launch the full robot simulation with Nav2, RViz, and Controllers:
```bash
ros2 launch sigyn_bringup sigyn.launch.py use_sim_time:=true
```

To run the specific "Can Do Challenge" (Coke fetching) simulation:
```bash
ros2 launch can_do_challenge can_do_sim_launch.py
```
*(requires `can_do_challenge` from `wimblerobotics/can_do_challenge` in your workspace)*

### Real Robot
Launch the physical driver stack:
```bash
ros2 launch sigyn_bringup sigyn.launch.py use_sim_time:=false
```

## 3. Package Structure Highlights

### Core System
- `sigyn_bringup` *(was `base`)*: Main entry point. Contains `sigyn.launch.py`, mapping launches, and global configs.
- `sigyn_description`: URDF/Xacro definitions. Critical for TF tree and physical parameters (friction, stiffness).
- `sigyn_interfaces`: Custom ROS 2 Actions/Services/Messages.

### Control & Navigation
- `wr_twist_multiplexer`: Arbitrates velocity commands between Nav2, Joystick, and Behavior Trees.
- `sigyn_bluetooth_joystick` *(separate repo: `wimblerobotics/sigyn_bluetooth_joystick`)*: Manual teleoperation.
- `sigyn_nav_goals`: Pre-defined waypoints for patrolling.

### AI & Behavior
- `sigyn_behavior_trees` *(separate repo: `wimblerobotics/sigyn_behavior_trees`)*: Core Behavior Tree nodes and XMLs for patrolling logic.
- `can_do_challenge` *(separate repo: `wimblerobotics/can_do_challenge`)*: Logic for the manipulation challenge (Visual servoing + Grasping).

### Hardware Bridges
- `sigyn_to_teensy` *(separate repo: `wimblerobotics/sigyn_to_teensy`)*: Teensy bridge for base motors, IMU, battery, elevator, and gripper. Replaces the old `sigyn_to_sensor_v2`.
- `sigyn_oakd_detection` *(separate workspace: `~/sigyn_oakd_detection_ws`)*: OAK-D Camera AI integration (YOLO). Publishes `OakdDetection.msg` (defined in `sigyn_interfaces`). The old `yolo_oakd_test` package has been **deleted** from this monorepo.

### Mapping
- `ros2 launch sigyn_bringup map_cartographer.launch.py` — build a new map with Cartographer SLAM
- `ros2 launch sigyn_bringup map_slam_toolbox.launch.py` — build a new map with SLAM Toolbox

## 4. Hardware Specifics

### Gripper Assembly
- **Type**: Parallel gripper on a prismatic elevator.
- **Controllers**: 
  - `forward_position_controller` (JointTrajectoryController) handles:
    - `parallel_gripper_base_plate_to_left_finger`
    - `parallel_gripper_base_plate_to_right_finger`
- **Tuning**: High stiffness (`kp=1e8`) required for grasping in simulation.

### Sensors
- **LIDAR**: LD06 for SLAM/Navigation.
- **Camera**: OAK-D Lite (Top mounted) for object detection.
- **Camera**: Raspberry Pi Camera (Gripper mounted) for close-range visual servoing.

## 5. Simulation Notes
- **Bridge**: `ros_gz_bridge` handles ROS2 <-> Gazebo communication.
- **Time**: Ensure `use_sim_time:=true` is set for all nodes in simulation.
- **Physics**: Uses ODE engine. Contact properties in `description/urdf` are critical for manipulation success.
