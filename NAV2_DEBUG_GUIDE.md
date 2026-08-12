# Nav2 & Docking Comprehensive Debug Guide

**Date:** 2026-08-03  
**Purpose:** Deep instrumentation of Nav2 navigation stack to diagnose docking failures

## Critical Changes Made

### 1. Nav2 DEBUG Logging Enabled
**File:** [navigation_launch.py](sigyn_bringup/launch/navigation_launch.py)

All critical Nav2 servers now run at **DEBUG** log level (overriding default "info"):
- ✅ `planner_server` - Shows planning requests, start/goal poses, path generation
- ✅ `controller_server` - Shows control loop execution, velocity commands, goal tracking
- ✅ `bt_navigator` - Shows behavior tree execution, navigation goals
- ✅ `docking_server` - Shows staging pose calculation, dock detection, visual servoing

**Effect:** Thousands of additional log messages showing internal Nav2 decision-making.

### 2. Nav2 Debug Monitor (NEW)
**File:** [nav2_debug_monitor.py](sigyn_bringup/scripts/nav2_debug_monitor.py)

A dedicated monitoring node that subscribes to all critical Nav2 topics and provides **human-readable analysis**:

**Monitors:**
- `/goal_pose` - Navigation goals sent to Nav2
- `/amcl_pose` - Current robot localization
- `/plan` - Global path from planner
- `/local_plan` - Local trajectory from controller
- `/staging_pose` - Docking staging pose (if published)

**Provides:**
- 🎯 **Goal analysis**: Position, orientation (with yaw in degrees)
- 📍 **Robot state**: Current pose when goal received
- 📐 **Relative geometry**: Distance to goal, bearing, rotation needed
- ⚠️ **Warnings**: Large rotations (>10°), backwards goals (>90°)
- 🗺️ **Path info**: Waypoint count, path length, start/end positions

### 3. Enhanced AprilTag Detection Logging
**Files:** 
- [apriltag_to_dock_pose.py](sigyn_bringup/scripts/apriltag_to_dock_pose.py)
- [docking_helper.py](sigyn_bringup/scripts/docking_helper.py)

Comprehensive AprilTag event tracking:
- 🎯 Detection acquisition/loss events
- 📍 Continuous pose tracking
- ⚠️ Low detection score warnings
- 📊 Docking state transitions

## How to Use

### Quick Start (Automated)

**Terminal 1:** Launch robot with debug logging
```bash
cd ~/sigyn_ws/src
./Sigyn/scripts/debug_docking.sh ~/my_test.txt
```

**Terminal 2:** Start Nav2 debug monitor
```bash
ros2 run sigyn_bringup nav2_debug_monitor.py
```

**Terminal 3:** Initiate docking
```bash
ros2 run sigyn_bringup docking_helper.py dock
```

### Manual Method

**Terminal 1:** Robot launch
```bash
ros2 launch sigyn_bringup sigyn.launch.py do_rviz:=false do_oakd:=true 2>&1 | tee ~/debug.txt
```

**Terminal 2:** Nav2 monitor
```bash
ros2 run sigyn_bringup nav2_debug_monitor.py
```

**Terminal 3:** Docking command
```bash
ros2 run sigyn_bringup docking_helper.py dock
```

**Terminal 4 (optional):** rviz2 on desktop
```bash
ros2 run rviz2 rviz2
```

## What You'll See

### Nav2 Debug Monitor Output

**When docking starts:**
```
================================================================================
🎯 GOAL #1 RECEIVED
   Frame: map
   Position: x=8.8800m, y=2.4400m, z=0.0000m
   Orientation: x=0.0000, y=0.0000, z=-0.9998, w=0.0175
   Yaw: -3.1066 rad (-178.00°)

📍 CURRENT ROBOT STATE:
   Position: x=7.2500m, y=3.1000m
   Yaw: 1.5708 rad (90.00°)

📐 GOAL RELATIVE TO ROBOT:
   Distance: 1.8547m
   Bearing: 0.3947 rad (22.62°)
   Rotation needed: -4.6774 rad (-268.00°)
   ⚠️  LARGE ROTATION REQUIRED: -268.0°
   ❌ GOAL IS BEHIND ROBOT (>90° rotation needed)!
================================================================================

🗺️  GLOBAL PLAN GENERATED
   Waypoints: 47
   Start: x=7.250, y=3.100
   End: x=8.880, y=2.440
   Path length: 2.145m
```

This tells you:
- **What goal was sent** (staging pose at 8.88, 2.44)
- **Where robot was** (7.25, 3.10)
- **Why it rotated** (goal required 268° rotation = goal is behind)
- **What path was planned** (2.14m path with 47 waypoints)

### DEBUG Log Output (in terminal 1)

**Planner server:**
```
[DEBUG] [planner_server]: Received planning request
[DEBUG] [planner_server]: Start: x=7.250 y=3.100 theta=1.571
[DEBUG] [planner_server]: Goal: x=8.880 y=2.440 theta=-3.107
[DEBUG] [planner_server]: Planning algorithm: GridBased
[DEBUG] [planner_server]: Path found with 47 waypoints
```

**Controller server:**
```
[DEBUG] [controller_server]: Control cycle started
[DEBUG] [controller_server]: Current pose: x=7.251 y=3.098 theta=1.569
[DEBUG] [controller_server]: Goal pose: x=8.880 y=2.440 theta=-3.107
[DEBUG] [controller_server]: Commanded velocities: linear=0.05 angular=-0.35
```

**Docking server:**
```
[DEBUG] [docking_server]: Dock pose: x=8.183 y=2.430 theta=3.118
[DEBUG] [docking_server]: Calculated staging pose: x=8.880 y=2.440 theta=-3.107
[DEBUG] [docking_server]: Staging offset: -0.7m in dock frame
[DEBUG] [docking_server]: Waiting for initial dock detection...
```

## Analyzing Navigation Failures

### Problem: Robot rotates immediately
**Look for in Nav2 monitor:**
- "LARGE ROTATION REQUIRED" warning
- Check if rotation_needed > 90° (goal is behind robot)
- Verify goal position matches expected staging pose

**Likely causes:**
1. Dock pose in database is wrong → staging pose calculation incorrect
2. Robot localization is poor → thinks it's somewhere else
3. Staging offset direction is backwards

### Problem: Robot heads in wrong direction
**Look for in Nav2 monitor:**
- Goal position coordinates
- Current robot position
- Bearing to goal (should point toward dock)
- Check global plan waypoints

**Likely causes:**
1. Goal sent to wrong coordinates
2. Map frame mismatch (odom vs map)
3. Planner couldn't find path to goal

### Problem: Robot can't make progress
**Look for in DEBUG logs:**
```
[ERROR] [controller_server]: Failed to make progress
```

**Check:**
- Is robot stuck in obstacle?
- Is path blocked by costmap inflation?
- Is goal in lethal obstacle?

## Verifying Dock Pose Database

The staging pose is calculated as **0.7m backward** from the dock pose.

**Current database:** [docking_stations.yaml](sigyn_bringup/config/docking_stations.yaml)
```yaml
home_charging_dock:
  pose: [8.1833, 2.4301, 3.118]  # [x, y, yaw]
```

**Staging pose calculation:**
```
staging_x = dock_x + 0.7 * cos(dock_yaw + π)
staging_y = dock_y + 0.7 * sin(dock_yaw + π)
staging_yaw = dock_yaw
```

With dock_yaw = 3.118 rad (178.7°):
```
staging_x = 8.1833 + 0.7 * cos(3.118 + 3.14159) = 8.1833 + 0.7 * (-0.999) ≈ 8.88
staging_y = 2.4301 + 0.7 * sin(3.118 + 3.14159) = 2.4301 + 0.7 * (0.035) ≈ 2.44
```

**To verify dock pose is correct:**
1. Manually drive robot to dock (physically touching)
2. Check AMCL pose:
   ```bash
   ros2 topic echo /amcl_pose --once
   ```
3. Record pose (x, y, yaw)
4. Update docking_stations.yaml

## Debug Topics Reference

| Topic | Type | Publisher | Purpose |
|-------|------|-----------|---------|
| `/goal_pose` | PoseStamped | bt_navigator | Navigation goals sent to planner |
| `/amcl_pose` | PoseWithCovarianceStamped | amcl | Robot localization |
| `/plan` | Path | planner_server | Global path from start to goal |
| `/local_plan` | Path | controller_server | Local trajectory (short-term) |
| `/staging_pose` | PoseStamped | docking_server | Calculated staging position |
| `/detected_dock_pose` | PoseStamped | apriltag_to_dock_pose.py | AprilTag-detected dock pose |
| `/cmd_vel_nav` | Twist | controller_server | Commanded velocities |

## Advanced Debugging

### Enable even MORE Nav2 verbosity

If DEBUG isn't enough, edit [navigation_launch.py](sigyn_bringup/launch/navigation_launch.py) and change:
```python
arguments=["--ros-args", "--log-level", "debug"],
```
to:
```python
arguments=["--ros-args", "--log-level", "debug", "--ros-args", "-p", "use_sim_time:=false"],
extra_arguments=[{"log-level": "debug", "console-output-format": "[{severity}] [{name}]: {message}"}],
```

### Monitor specific topics manually

**Watch goals being sent:**
```bash
ros2 topic echo /goal_pose
```

**Watch paths generated:**
```bash
ros2 topic echo /plan
```

**Watch velocities commanded:**
```bash
ros2 topic echo /cmd_vel_nav
```

**Check TF transforms:**
```bash
# Map to robot
ros2 run tf2_ros tf2_echo map base_link

# Map to dock (if AprilTag detected)
ros2 run tf2_ros tf2_echo map dock_frame
```

### Visualize in rviz2

Add these displays:
1. **Path** → `/plan` (global plan, green)
2. **Path** → `/local_plan` (local trajectory, red)
3. **PoseStamped** → `/goal_pose` (navigation goal, large arrow)
4. **PoseStamped** → `/staging_pose` (docking staging, yellow arrow)
5. **PoseWithCovariance** → `/amcl_pose` (robot localization)

## Importing Nav2 into Workspace

If you need to modify Nav2 source code for deeper instrumentation:

```bash
cd ~/sigyn_ws/src
git clone https://github.com/ros-planning/navigation2.git -b jazzy
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git -b v4.6.2

# Build specific packages
cd ~/sigyn_ws
colcon build --symlink-install --packages-select \
    nav2_planner \
    nav2_controller \
    nav2_bt_navigator \
    opennav_docking
```

Then you can add logging directly to the C++ source:
- `navigation2/nav2_planner/src/planner_server.cpp`
- `navigation2/nav2_controller/src/controller_server.cpp`
- `navigation2/nav2_docking/opennav_docking/src/docking_server.cpp`

## Troubleshooting

### Nav2 debug monitor not showing output
```bash
# Check if it's running
ros2 node list | grep nav2_debug_monitor

# Check topic connections
ros2 topic info /goal_pose -v
```

### No DEBUG messages in logs
```bash
# Verify log level was set
ros2 param get /planner_server use_sim_time  # (any param to test connection)
ros2 param get /planner_server log_level      # Should fail - log_level isn't a param

# Check node list
ros2 node list | grep -E "planner|controller|docking"
```

### Goals not appearing
```bash
# Check if docking action is being sent
ros2 action list
ros2 action info /dock_robot

# Monitor docking action directly
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{use_dock_id: true, dock_id: 'home_charging_dock'}"
```

## Summary

**Before this instrumentation:**
- ❌ No visibility into Nav2 decision-making
- ❌ Unknown what goal was sent or why
- ❌ No way to see if dock/staging pose was correct
- ❌ Debugging required guesswork

**After this instrumentation:**
- ✅ Complete visibility into goals, paths, and commands
- ✅ Human-readable analysis of navigation geometry
- ✅ Clear warnings when goals are problematic
- ✅ Staging pose calculations logged
- ✅ Can definitively determine if problem is dock pose, localization, or planning

**Next steps:**
1. Run debug session with new instrumentation
2. Share logs showing goal coordinates and rotation analysis
3. Verify dock pose in database matches physical location
4. Update staging offset direction if needed
