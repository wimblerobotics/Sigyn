# Sigyn Robot Operation Guide

## Overview

Sigyn is an autonomous mobile robot equipped with navigation, sensor fusion, and manipulation capabilities. The system uses ROS 2 Jazzy and integrates multiple TeensyV2 boards for low-level control, eliminating the need for micro-ROS agents.

## Quick Start

### Launch the Complete System (Real Robot)

```bash
ros2 launch base sigyn.launch.py
```

This launches the full navigation stack with default parameters. All launch options are described below.

### Launch with Simulation

```bash
ros2 launch base sigyn.launch.py use_sim_time:=true
```

## Launch File Options

The `sigyn.launch.py` file supports the following arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Set to `true` for simulation mode, `false` for real robot |
| `do_rviz` | `true` | Launch RViz visualization |
| `do_top_lidar` | `true` | Enable the top LiDAR (set `false` for single LiDAR at cup origin) |
| `do_oakd` | `false` | Launch OAK-D camera nodes |
| `do_joystick` | `false` | Launch Bluetooth joystick control |
| `make_map` | `false` | Enable SLAM Toolbox for mapping instead of localization |
| `urdf_file_name` | `sigyn.urdf.xacro` | URDF model file to use |
| `world` | `home.world` | Gazebo world file (simulation only) |
| `bt_xml` | (nav2 default) | Behavior tree XML file path |

### Example Launch Commands

```bash
# Real robot with RViz and joystick
ros2 launch base sigyn.launch.py do_joystick:=true

# Real robot with OAK-D camera
ros2 launch base sigyn.launch.py do_oakd:=true

# Create a new map
ros2 launch base sigyn.launch.py make_map:=true

# Simulation with RViz
ros2 launch base sigyn.launch.py use_sim_time:=true do_rviz:=true

# Real robot without top LiDAR
ros2 launch base sigyn.launch.py do_top_lidar:=false
```

## System Components

When launched, the system starts:

1. **Navigation Stack (Nav2)** - Path planning, costmap management, localization (AMCL)
2. **TeensyV2 Bridge** - Hardware interface for motors, sensors, and manipulator
3. **Robot State Publisher** - TF tree and joint states
4. **Sensor Processing** - LiDAR, IMU, range sensors, cameras
5. **EKF Localization** - Sensor fusion for odometry
6. **Velocity Multiplexer** - Command prioritization

## Topics

### Motion Control

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Final velocity commands to robot (output of multiplexer) |
| `/cmd_vel_keyboard` | `Twist` | Keyboard teleop input |
| `/cmd_vel_joystick` | `Twist` | Bluetooth joystick input |
| `/cmd_vel_nav` | `Twist` | Navigation stack commands |
| `/cmd_vel_teleop` | `Twist` | General teleop commands |
| `/cmd_vel_smoothed` | `Twist` | Smoothed velocity commands |
| `/cmd_vel_gripper` | `Twist` | Elevator/extender control (linear.x=elevator, angular.z=extender) |
| `/preempt_teleop` | `Bool` | Preempt navigation for manual control |

### Manipulator Control

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_gripper` | `Twist` | Incremental control: `linear.x` for elevator up/down, `angular.z` for extender in/out |

**Elevator/Extender Control Examples:**
```bash
# Move elevator up (small increment)
ros2 topic pub -1 /cmd_vel_gripper geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

# Move extender out (small increment)
ros2 topic pub -1 /cmd_vel_gripper geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"

# Move both simultaneously
ros2 topic pub -1 /cmd_vel_gripper geometry_msgs/msg/Twist "{linear: {x: -0.5}, angular: {z: 0.5}}"
```

### Sensor Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `LaserScan` | Processed LiDAR scan (filtered) |
| `/raw_scan` | `LaserScan` | Raw LiDAR data |
| `/scan_cup` | `LaserScan` | Cup LiDAR scan |
| `/odom` | `Odometry` | Filtered odometry from EKF |
| `/sigyn/wheel_odom` | `Odometry` | Raw wheel odometry from TeensyV2 |
| `/sigyn/teensy_bridge/imu/sensor_0` | `Imu` | Primary IMU data |
| `/sigyn/teensy_bridge/imu/sensor_1` | `Imu` | Secondary IMU data |
| `/sigyn/teensy_bridge/range/vl53l0x_[0-7]` | `Range` | VL53L0X ToF sensors (8 sensors) |
| `/sigyn/teensy_bridge/temperature/motor_[0-1]` | `Temperature` | Motor temperature |

### Battery & Diagnostics

| Topic | Type | Description |
|-------|------|-------------|
| `/sigyn/teensy_bridge/battery/status` | `BatteryState` | Multi-battery monitoring (36V LiPo, 5V, 24V, 12V, 3.3V) |
| `/battery_overlay_text` | `String` | Battery overlay for RViz |
| `/sigyn/teensy_bridge/diagnostics` | `DiagnosticArray` | System diagnostics from TeensyV2 |
| `/sigyn/teensy_bridge/safety/estop_status` | `DiagnosticArray` | Emergency stop status |
| `/diagnostics` | `DiagnosticArray` | System-wide diagnostics |

### Navigation Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `OccupancyGrid` | Current map |
| `/map_updates` | `OccupancyGridUpdate` | Map changes |
| `/amcl_pose` | `PoseWithCovarianceStamped` | Localized robot pose |
| `/particle_cloud` | `PoseArray` | AMCL particle cloud |
| `/goal_pose` | `PoseStamped` | Navigation goal |
| `/plan` | `Path` | Global path plan |
| `/plan_smoothed` | `Path` | Smoothed path |
| `/local_costmap/costmap` | `OccupancyGrid` | Local costmap for obstacle avoidance |
| `/global_costmap/costmap` | `OccupancyGrid` | Global costmap for planning |

### Camera Topics (when `do_oakd:=true`)

| Topic | Type | Description |
|-------|------|-------------|
| `/oakd_top/color/image/compressed` | `CompressedImage` | Compressed color image |
| `/stereo/points2` | `LaserScan` | Point cloud converted to laser scan |

## Services

### TeensyV2 Hardware Services

| Service | Type | Description |
|---------|------|-------------|
| `/sigyn/teensy_sensor_sd_getdir` | `TeensySdGetDir` | List SD card directory |
| `/sigyn/teensy_sensor_sd_getfile` | `TeensySdGetFile` | Retrieve file from SD card |
| `/sigyn/teensy_bridge/safety/reset_fault` | `ResetFault` | Reset safety faults |

### Navigation Services

| Service | Type | Description |
|---------|------|-------------|
| `/global_costmap/clear_entirely_global_costmap` | `ClearEntireCostmap` | Clear global costmap |
| `/local_costmap/clear_entirely_local_costmap` | `ClearEntireCostmap` | Clear local costmap |
| `/reinitialize_global_localization` | `Empty` | Reset AMCL |
| `/set_initial_pose` | `PoseWithCovarianceStamped` | Set robot initial pose |

## Action Servers

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_pose` | `NavigateToPose` | Navigate to a goal pose |
| `/navigate_through_poses` | `NavigateThroughPoses` | Navigate through waypoints |
| `/follow_path` | `FollowPath` | Follow a predefined path |
| `/spin` | `Spin` | Rotate in place |
| `/backup` | `BackUp` | Move backward |
| `/wait` | `Wait` | Wait for duration |
| `/assisted_teleop` | `AssistedTeleop` | Obstacle-aware teleop |
| `/say_something` | Custom | Text-to-speech action |
| `/move_a_short_distance_ahead` | Custom | Short forward movement |

## Teleoperation

### Keyboard Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_keyboard
```

Use `i/j/k/l` for movement, `q/z` for speed control.

### Bluetooth Joystick

Launch with the system:
```bash
ros2 launch base sigyn.launch.py do_joystick:=true
```

Or separately:
```bash
ros2 launch bluetooth_joystick bluetooth_joystick.launch.py
```

## Command Priority

The twist multiplexer prioritizes commands in this order:
1. Keyboard teleop (`cmd_vel_keyboard`)
2. Joystick (`cmd_vel_joystick`)
3. Manual teleop (`cmd_vel_teleop`)
4. Navigation (`cmd_vel_nav`)

## Useful Commands

### Building

```bash
cd ~/sigyn_ws
colcon build --symlink-install
```

### Mapping

```bash
# Start mapping mode
ros2 launch base sigyn.launch.py make_map:=true

# Save the map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Diagnostics

```bash
# View all topics
ros2 topic list

# Monitor battery
ros2 topic echo /sigyn/teensy_bridge/battery/status

# Check diagnostics
ros2 topic echo /sigyn/teensy_bridge/diagnostics

# View TF tree
ros2 run tf2_tools view_frames
```

### Clear Costmaps

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

## Troubleshooting

### No communication with TeensyV2 boards

Check device connections:
```bash
ls /dev/teensy_*
```

Should show: `/dev/teensy_sensor`, `/dev/teensy_sensor2`, `/dev/teensy_gripper`

Reload udev rules if needed:
```bash
sudo service udev restart
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Robot not moving

1. Check velocity multiplexer is receiving commands
2. Verify no emergency stop is active: `ros2 topic echo /sigyn/teensy_bridge/safety/estop_status`
3. Check battery voltage is adequate

### Navigation not working

1. Verify localization: `ros2 topic echo /amcl_pose`
2. Set initial pose in RViz (2D Pose Estimate)
3. Clear costmaps if obstacles are blocking paths

## System Architecture

- **Hardware Layer**: 3× Teensy 4.1 boards (motor control, sensors, manipulator)
- **Bridge Layer**: `sigyn_to_sensor_v2` package handles TeensyV2 communication
- **Navigation Layer**: Nav2 stack for autonomous navigation
- **Application Layer**: Behavior trees for task execution



