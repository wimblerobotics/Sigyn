# Nav2 Docking System Setup for Sigyn Robot

**Date:** 2026-07-31  
**Status:** Configured and ready for testing

## Overview

The Sigyn robot uses Nav2's `opennav_docking` package for autonomous docking at the charging station. The system uses:
- **AprilTag ID 1** for visual dock detection (via OAK-D Lite camera)
- **Charging port current monitoring** for mate confirmation (via `/sigyn/power/charger` topic)

## Calibrated Docking Parameters

### AprilTag Detection When Docked
From captured data when robot is successfully docked:
- **Tag ID:** 1
- **Detection Score:** 76.71
- **Distance from camera:** 0.364m
- **Position (oakd_apriltag_optical_frame):**
  - x: -0.079m (lateral offset)
  - y: 0.017m (vertical offset)
  - z: 0.364m (forward distance)

### Charging Port Confirmation
From `/sigyn/power/charger` when charging port is mated:
- **Voltage:** 41.73V
- **Current:** 4.07A (positive = charging)
- **Status:** CHARGING (power_supply_status = 1)
- **Threshold:** Current > 0.01A indicates successful connection

## Configuration Files

### 1. Navigation Configuration
**File:** `sigyn_bringup/config/navigation.yaml`

Key docking parameters:
```yaml
docking_server:
  ros__parameters:
    controller_frequency: 50.0
    dock_approach_timeout: 30.0
    dock_backwards: false  # Approach dock driving forward
    
    dock_plugins: ['simple_charging_dock']
    simple_charging_dock:
      plugin: 'opennav_docking::SimpleChargingDock'
      docking_threshold: 0.05  # Final alignment tolerance (m)
      staging_x_offset: -0.7    # Pre-staging distance from dock (m)
      
      use_external_detection_pose: true  # Use AprilTag detection
      use_battery_status: true           # Monitor charging current
      
      # AprilTag transform (calibrated 2026-07-31)
      external_detection_translation_x: -0.079
      external_detection_translation_y: 0.017
      
      # Charging validation
      charging_threshold: 0.01  # Amps
```

### 2. Dock Database
**File:** `sigyn_bringup/config/docking_stations.yaml`

Defines dock instances (map locations to be filled in):
```yaml
docking_server:
  ros__parameters:
    docks: ['home_charging_dock']
    
    home_charging_dock:
      type: 'simple_charging_dock'
      frame: 'map'
      pose: [x, y, yaw]  # TODO: Set actual map coordinates
```

### 3. Launch Configuration
**File:** `sigyn_bringup/launch/navigation_launch.py`

Docking server is now enabled in the navigation stack lifecycle.

## Usage

### Command-Line Interface

#### 1. Dock the Robot
```bash
# Using dock ID from database (requires map coordinates set in docking_stations.yaml)
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: true, dock_id: 'home_charging_dock', navigate_to_staging_pose: true}"

# Using current pose (drive near dock first, requires AprilTag in view)
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot \
  "{use_dock_id: false, navigate_to_staging_pose: false, dock_type: 'simple_charging_dock'}"
```

#### 2. Undock the Robot
```bash
ros2 action send_goal /undock_robot nav2_msgs/action/UndockRobot "{}"
```

#### 3. Monitor Docking Detection
```bash
# Watch AprilTag detections
ros2 topic echo /oakd_apriltag_node/detections

# Watch charging status
ros2 topic echo /sigyn/power/charger
```

### Python Helper Script

**File:** `sigyn_bringup/scripts/docking_helper.py`

```bash
# Dock to the charging station
python3 ~/sigyn_ws/src/Sigyn/sigyn_bringup/scripts/docking_helper.py dock

# Undock from the charging station
python3 ~/sigyn_ws/src/Sigyn/sigyn_bringup/scripts/docking_helper.py undock

# Check detection and charging status
python3 ~/sigyn_ws/src/Sigyn/sigyn_bringup/scripts/docking_helper.py status
```

## Setup Checklist

- [x] AprilTag detector integrated into launch system
- [x] Charging port topic `/sigyn/power/charger` publishing
- [x] Docking server configuration updated with calibrated values
- [x] Docking server enabled in navigation launch
- [x] Helper scripts created
- [ ] **TODO:** Set actual map coordinates for `home_charging_dock` in `docking_stations.yaml`
- [ ] **TODO:** Test docking sequence end-to-end
- [ ] **TODO:** Tune staging_x_offset if needed (currently -0.7m)
- [ ] **TODO:** Build sigyn_bringup package with updated configuration

## Docking Sequence

1. **Navigate to Staging Pose**
   - Robot moves to pre-dock position (~0.7m from dock)
   - Uses map-based navigation

2. **Visual Alignment**
   - AprilTag detector acquires dock marker
   - Controller aligns robot based on tag pose
   - Approaches slowly toward docking threshold (0.05m)

3. **Mate Confirmation**
   - Monitors `/sigyn/power/charger` current
   - Charging current > 0.01A confirms electrical connection
   - Docking action completes successfully

4. **Charging Mode**
   - Robot remains docked
   - Battery monitoring continues
   - Ready for undock command

## Troubleshooting

### AprilTag Not Detected
- Check camera is running: `ros2 topic hz /oakd_apriltag_node/detections`
- Verify tag is visible and properly lit
- Check detection score (should be > 70)
- Ensure tag size parameter is 0.120m

### Charging Not Confirming
- Check charger topic: `ros2 topic echo /sigyn/power/charger`
- Verify current > 0.01A when manually docked
- Check voltage ~41-42V range
- Ensure physical connector alignment

### Docking Fails to Complete
- Check staging_x_offset (may need adjustment)
- Verify AprilTag transform values match actual mounting
- Review docking_threshold (0.05m may be too tight)
- Check controller_frequency (50Hz) is achievable

### Map Frame Issues
- **KNOWN ISSUE:** EKF not publishing odom frame (missing ODOM messages from Teensy)
- This must be fixed before autonomous docking can work
- Temporary workaround: Manually drive near dock, use `use_dock_id: false`

## Next Steps

1. **Resolve ODOM Message Issue**
   - Critical blocker for navigation stack
   - Investigate why Teensy firmware not sending ODOM messages
   
2. **Build and Deploy**
   ```bash
   cd ~/sigyn_ws
   colcon build --packages-select sigyn_bringup
   ```

3. **Manual Docking Test**
   - Drive robot near charging station
   - Ensure AprilTag visible
   - Test docking with `use_dock_id: false`

4. **Map Coordinate Measurement**
   - Once navigation is working, record dock position in map frame
   - Update `docking_stations.yaml` with actual coordinates

5. **Autonomous Docking Test**
   - Test full sequence with `navigate_to_staging_pose: true`
   - Verify charging confirmation
   - Test undocking

## References

- [Nav2 Docking Tutorial](https://navigation.ros.org/tutorials/docs/get_backtrace.html)
- [opennav_docking Documentation](https://github.com/open-navigation/opennav_docking)
- AprilTag tag36h11 family, 120mm physical size
- OAK-D Lite camera: MxID 1944301081303C1200
