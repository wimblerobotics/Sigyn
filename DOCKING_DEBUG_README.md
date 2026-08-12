# Docking Debug Instrumentation

**Date:** 2026-08-03  
**Purpose:** Comprehensive debugging of AprilTag-based docking failures

## Changes Made

### 1. Navigation Launch - DEBUG Level Logging
**File:** `Sigyn/sigyn_bringup/launch/navigation_launch.py`

- **Change:** Force `docking_server` to DEBUG log level (overriding default "info")
- **Effect:** Enables verbose logging of:
  - Initial dock detection attempts
  - Staging pose calculations
  - Visual servoing control decisions
  - AprilTag tracking status
  - Docking state transitions
  - Collision detection triggers
  - Retry logic and failure reasons

### 2. Docking Helper - Enhanced Monitoring
**File:** `Sigyn/sigyn_bringup/scripts/docking_helper.py`

**AprilTag Detection Tracking:**
- Logs `🎯 APRILTAG ACQUIRED` when tag first detected
- Logs `❌ APRILTAG LOST` when detection drops
- Continuous pose logging: `[x, y, z]` and orientation quaternion
- Detection score monitoring

**Docking State Feedback:**
- State name decoding (INITIAL, WAIT_FOR_VALID_DOCK, DOCK_APPROACH, DOCKING, WAIT_FOR_CHARGE)
- `📍 DOCKING STATE` messages with:
  - Current state
  - Retry count
  - AprilTag visibility status
  - Live tag position during approach

### 3. Dock Pose Converter - Event Tracking
**File:** `Sigyn/sigyn_bringup/scripts/apriltag_to_dock_pose.py`

**Detection Events:**
- `🎯 DOCK TAG ACQUIRED` - First detection of dock tag
- `📍 Dock @ x=... y=... z=...` - Continuous pose tracking
- `❌ DOCK TAG LOST` - Detection lost (no array OR not in array)
- `⚠️  Dock tag score too low` - Score below threshold (50.0)

**Published Topics:**
- `/detected_dock_pose` - Filtered dock pose for docking server
- Logs full 3D position with score for every detection

## Usage

### Method 1: Automated Debug Launch
```bash
cd ~/sigyn_ws/src
./Sigyn/scripts/debug_docking.sh ~/my_docking_test.txt
```

This will:
1. Launch robot stack with DEBUG logging enabled
2. Capture ALL output to timestamped file
3. Display output to terminal simultaneously

### Method 2: Manual Launch with Redirection
```bash
# Terminal 1: Launch robot
ros2 launch sigyn_bringup sigyn.launch.py do_rviz:=false do_oakd:=true 2>&1 | tee ~/docking_debug.txt

# Terminal 2 (on amdc): Launch rviz2 for visualization
ros2 run rviz2 rviz2

# Terminal 3: Initiate docking
ros2 run sigyn_bringup docking_helper.py dock
```

### Method 3: Live Monitoring Only
```bash
# Terminal 1: Launch robot
ros2 launch sigyn_bringup sigyn.launch.py do_rviz:=false do_oakd:=true

# Terminal 2: Monitor AprilTag detections
ros2 topic echo /oakd_apriltag_node/detections

# Terminal 3: Monitor dock pose conversions
ros2 topic echo /detected_dock_pose

# Terminal 4: Initiate docking
ros2 run sigyn_bringup docking_helper.py dock
```

## What You'll See in the Logs

### Successful AprilTag Detection Sequence
```
[apriltag_dock_converter]: 🎯 DOCK TAG ACQUIRED - ID:1 pos=[...] score=85.0
[docking_helper]: 🎯 APRILTAG ACQUIRED - ID:1 @ x=... y=... z=... score=85.0
[apriltag_dock_converter]: 📍 Dock @ x=-0.079m y=0.017m z=0.364m score=85.3
[docking_helper]: AprilTag ID 1: pos=[...] orient=[...] score=85.3
```

### Docking State Progression
```
[docking_helper]: 📍 DOCKING STATE: INITIAL | Retries: 0 | AprilTag: NO DETECTION
[bt_navigator]: Begin navigating to staging pose (8.88, 2.41)
[docking_helper]: 📍 DOCKING STATE: DOCK_APPROACH | Retries: 0 | AprilTag: VISIBLE
[docking_helper]:    └─ Tag at: x=-0.081m y=0.015m z=0.368m
[docking_helper]: 📍 DOCKING STATE: DOCKING | Retries: 0 | AprilTag: VISIBLE
[docking_server]: [DEBUG] Visual servoing: linear=0.05 angular=0.02
```

### Tag Loss Events
```
[apriltag_dock_converter]: ❌ DOCK TAG LOST - No detections in array
[docking_helper]: ❌ APRILTAG LOST - No detection
[docking_server]: [WARN] Lost detection or did not detect: timeout exceeded
```

### Failure Diagnosis
```
[docking_server]: [ERROR] Failed initial dock detection
[docking_helper]: ✗ Docking failed: error_code=XXX
```

## Key Debug Topics to Monitor

| Topic | Type | Purpose |
|-------|------|---------|
| `/oakd_apriltag_node/detections` | Detection3DArray | Raw AprilTag detections |
| `/detected_dock_pose` | PoseStamped | Filtered dock pose for docking server |
| `/staging_pose` | PoseStamped | Calculated staging pose |
| `/docking_trajectory` | Path | Visual servoing trajectory |
| `/sigyn/power/charger` | BatteryState | Charging current detection |

## Analyzing Failures

### Problem: "Failed initial dock detection"
**Look for:**
- Is AprilTag being detected at all? Check for `🎯 APRILTAG ACQUIRED` messages
- Is tag score too low? Check for `⚠️ Dock tag score too low` warnings
- Is staging pose correct? Compare logged pose to expected (8.88, 2.41)

### Problem: Robot crashes into wall
**Look for:**
- Staging pose location in logs
- Visual servoing commands: `linear=X angular=Y`
- AprilTag pose relative to robot during approach
- Whether `use_collision_detection: false` is causing blind approach

### Problem: "Docking successful" but no charging
**Look for:**
- Final robot position vs dock pose
- Charging current in `/sigyn/power/charger` topic
- `charging_threshold: 0.01` comparison in logs
- Whether robot actually contacted dock

### Problem: AprilTag lost during approach
**Look for:**
- Distance when tag lost (z value in pose)
- `external_detection_timeout: 2.0` exceeded?
- Was tag occluded or outside camera FOV?
- Approach speed causing motion blur?

## Additional Configuration (Already Set)

### In navigation.yaml:
```yaml
docking_server:
  controller_frequency: 50.0
  dock_prestaging_tolerance: 0.08  # Tight tolerance
  simple_charging_dock:
    docking_threshold: 0.10  # Relaxed from 0.05m
    staging_x_offset: -0.7   # Robot frame backward
    external_detection_timeout: 2.0  # Increased from 1.0s
    v_linear_min/max: 0.05/0.08  # Slow approach
    use_collision_detection: false  # Disabled near wall
```

### In docking_stations.yaml:
```yaml
home_charging_dock:
  pose: [8.1833, 2.4301, 3.118]  # Measured dock pose
```

## Expected Behavior

1. **Navigation to Staging:** Robot navigates to (8.88, 2.41)
2. **AprilTag Acquisition:** Tag should be visible from staging pose at ~0.7m distance
3. **Visual Servoing:** Slow approach (0.05-0.08 m/s) toward tag
4. **Docking Completion:** Robot reaches docking_threshold (0.10m from target)
5. **Charging Validation:** Current > 0.01A on /sigyn/power/charger

## Troubleshooting Commands

```bash
# Check if AprilTag node is running
ros2 node list | grep oakd

# Check AprilTag detection rate
ros2 topic hz /oakd_apriltag_node/detections

# Check current robot pose
ros2 topic echo /amcl_pose --once

# Check TF from base_link to oakd camera
ros2 run tf2_ros tf2_echo base_link oakd_apriltag_optical_frame

# Check docking server parameters
ros2 param dump /docking_server

# Manual docking status check
ros2 run sigyn_bringup docking_helper.py status
```

## Next Steps After Data Collection

1. Share the full debug log file for analysis
2. Note the exact failure mode:
   - No tag detection?
   - Tag detected but wrong approach?
   - Tag lost during approach?
   - Crashes at specific distance?
   - Reports success incorrectly?
3. Check video/screencast showing robot behavior
4. Verify staging pose allows tag visibility from camera FOV
