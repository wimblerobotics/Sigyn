# Localization and Sensor Fusion Configuration Guide

**Last Updated:** October 10, 2025  
**Robot:** Sigyn House Patroller  
**ROS Version:** ROS 2 Jazzy

## Table of Contents
1. [Overview](#overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Sensor Fusion Strategy](#sensor-fusion-strategy)
4. [EKF Configuration](#ekf-configuration)
5. [AMCL Configuration](#amcl-configuration)
6. [Multi-Height Lidar Strategy](#multi-height-lidar-strategy)
7. [Lessons Learned](#lessons-learned)
8. [Troubleshooting](#troubleshooting)

---

## Overview

This document captures the design decisions, configuration choices, and lessons learned while implementing robust localization and sensor fusion for the Sigyn robot. The robot uses:
- Dual-height lidar sensors (30cm and 178cm)
- Dual BNO055 IMUs (different mounting orientations)
- Differential drive wheel odometry
- AMCL for map-based localization
- Extended Kalman Filter (EKF) for odometry fusion

---

## Hardware Configuration

### Sensors

#### Wheel Odometry
- **Source:** RoboClaw motor controller via Teensy 4.1
- **Topic:** `/sigyn/wheel_odom`
- **Frame:** `frame_id="odom"`, `child_frame_id="base_link"`
- **Update Rate:** ~30-50 Hz
- **Accuracy:** 
  - Linear: ±2% (tested over 0.5m)
  - Angular: ±5% (tested over 45° rotation)

#### IMU Sensors (BNO055)
This section is out of date as, though I have a pair of IMU sensors mounted upside down to each other with the aim of canceling out the rate gyro noise when the robot isn't moving, in the end I only use the data from one IMU.
- **IMU0 (sensor_0):** Mounted upside-down and backwards
  - Location: `xyz="-0.100 0.180 0.022"`, `rpy="0 π π"`
  - CCW rotation → **negative** gz (due to inverted mounting)
  - **Status:** Disabled in EKF (redundancy only)

- **IMU1 (sensor_1):** Mounted right-side up but backwards
  - Location: `xyz="0.085 -0.185 0.022"`, `rpy="0 0 π"`
  - CCW rotation → **positive** gz (correct per REP-103)
  - **Status:** Active in EKF
  - **Firmware:** Custom Teensy code with explicit UNIT_SEL=0x00 configuration

#### Lidar Sensors (LD06)
- **Top Lidar (`/scan`):**
  - Height: 0.30m above base_link
  - Orientation: -90° yaw (pointing left)
  - Purpose: Obstacle detection for navigation
  - Sees: Floor clutter, chair legs, low obstacles

- **Cup Lidar (`/scan_cup`):**
  - Height: 1.785m above base_link  
  - Orientation: 0° yaw (pointing forward)
  - Purpose: Localization (AMCL)
  - Sees: Clean wall surfaces, stable landmarks

---

## Sensor Fusion Strategy

### Why Multi-Sensor Fusion?

**Wheel Odometry Strengths:**
- ✅ High frequency updates (30-50 Hz)
- ✅ Accurate short-term velocity measurements
- ✅ Good position baseline

**Wheel Odometry Weaknesses:**
- ❌ Accumulates drift over distance
- ❌ Affected by wheel slip
- ❌ Yaw rate less accurate during fast rotation

**IMU Strengths:**
- ✅ Excellent yaw rate accuracy
- ✅ Stable absolute orientation (sensor fusion in BNO055)
- ✅ Not affected by wheel slip

**IMU Weaknesses:**
- ❌ Cannot provide position
- ❌ Linear acceleration not useful on flat ground (2D mode)

**AMCL Strengths:**
- ✅ Global position correction
- ✅ Eliminates long-term drift
- ✅ Robust to dynamic obstacles (with proper tuning)

**AMCL Weaknesses:**
- ❌ Lower update rate (1-5 Hz)
- ❌ Can fail in featureless environments
- ❌ Sensitive to sensor noise and clutter

### Fusion Architecture

```
┌─────────────────┐
│  Wheel Odometry │──────┐
│   (30-50 Hz)    │      │
└─────────────────┘      │
                         ├──► EKF ──► odom→base_link (50 Hz)
┌─────────────────┐      │      │
│   IMU sensor_1  │──────┘      │
│   (50 Hz)       │             │
└─────────────────┘             │
                                ├──► Combined Localization
┌─────────────────┐             │
│  AMCL (scan_cup)│─────────────┘
│   (1-5 Hz)      │  map→odom
└─────────────────┘
```

---

## EKF Configuration

### File: `base/config/ekf.yaml`

### Key Parameters

```yaml
frequency: 50.0           # Match sensor update rates
two_d_mode: true          # Planar operation only
publish_tf: true          # Publish odom→base_link transform
odom_frame: odom
base_link_frame: base_link
world_frame: odom         # EKF operates in odom frame
```

### Wheel Odometry Fusion

**Topic:** `/sigyn/wheel_odom`

**Fused states:**
```yaml
odom0_config: [false, false, false,   # No position (drifts)
               false, false, false,   # No orientation (use IMU)
               true,  true,  false,   # Fuse vx, vy velocities
               false, false, true,    # Fuse yaw rate wz
               false, false, false]   # No acceleration
```

**Rationale:**
- Position (`x`, `y`) not fused directly to avoid drift accumulation
- Velocities (`vx`, `vy`, `wz`) provide accurate short-term motion
- AMCL corrects position drift via `map→odom` transform

### IMU Fusion (sensor_1)

**Topic:** `/sigyn/teensy_bridge/imu/sensor_1`

**Fused states:**
```yaml
imu1_config: [false, false, false,
              false, false, true,     # Fuse absolute yaw orientation
              false, false, false,
              false, false, true,     # Fuse yaw rate gz
              false, false, false]
```

**Rationale:**
- **Absolute yaw:** Prevents gyro drift, stabilizes heading
- **Yaw rate:** More accurate than wheels during rotation
- **No linear acceleration:** Not useful in 2D mode on flat ground

### Why NOT Both IMUs?

**Decision:** Use only IMU1, disable IMU0

**Reasons:**
1. Gyro drift does NOT cancel between two IMUs
2. Conflicting absolute yaw readings cause EKF instability
3. Both IMUs would report slightly different orientations
4. EKF would average or jump between them, reducing accuracy
5. One well-calibrated IMU with absolute orientation is superior

**IMU0 Usage:** Keep for redundancy/failover, but not in EKF fusion

---

## AMCL Configuration

### File: `base/config/navigation_sim.yaml`

### Critical Decision: Which Lidar for Localization?

**Choice:** Use **upper lidar** (`/scan_cup` at 1.78m)

**Rationale:**

| Aspect | Lower Lidar (/scan @ 30cm) | Upper Lidar (/scan_cup @ 178cm) |
|--------|---------------------------|----------------------------------|
| **Wall visibility** | Blocked by furniture | Clear view of walls |
| **Floor clutter** | Sees chairs, boxes, cables | Above clutter |
| **Dynamic obstacles** | Many (people, chairs) | Few (stable environment) |
| **Landmark stability** | Poor (furniture moves) | Excellent (walls permanent) |
| **Doorways** | Clear passage | Sees door frames |
| **Localization quality** | ❌ Poor | ✅ Excellent |

**Configuration:**
```yaml
scan_topic: scan_cup  # Use upper lidar for clean wall features
```

### Key AMCL Parameters

```yaml
# Motion model - tuned for good wheel odometry
alpha1: 0.20       # Rotation noise from rotation
alpha2: 0.20       # Rotation noise from translation
alpha3: 0.02       # Translation noise from translation (trust wheels)
alpha4: 0.2        # Translation noise from rotation

# Update thresholds - more frequent updates for responsive localization
update_min_a: 0.005      # Update after ~0.29° rotation
update_min_d: 0.01       # Update after 1cm movement

# Particle filter
max_particles: 5000      # Large for robust localization
min_particles: 2000

# Transform tolerance
transform_tolerance: 0.2  # Allow 200ms latency
```

---

## Multi-Height Lidar Strategy

### Lidar Role Separation

**Upper Lidar (`/scan_cup` @ 178cm):**
- ✅ **Used by:** AMCL for localization
- ✅ **Advantage:** Clean wall features, no floor clutter
- ❌ **NOT used for:** Navigation costmap (sees doorframes as obstacles)

**Lower Lidar (`/scan` @ 30cm):**
- ✅ **Used by:** Voxel layer for obstacle detection
- ✅ **Advantage:** Sees floor obstacles, passes under doorframes
- ❌ **NOT used for:** AMCL (too much clutter)

### Local Costmap Configuration

```yaml
plugins: [
  "static_layer",
  "voxel_layer",      # Uses only lower lidar
  "oakd_top_layer",
  "inflation_layer"
]

voxel_layer:
  observation_sources: scan_low  # Only lower lidar
  scan_low:
    topic: /scan
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
    obstacle_max_range: 2.5
```

**Why not both lidars in costmap?**
- Upper lidar detects doorframes → blocks navigation
- Lower lidar at 30cm passes under doorways
- Trade-off: Won't detect overhead obstacles (acceptable for most environments)

---

## Lessons Learned

### 1. IMU Firmware Unit Configuration

**Problem:** IMU angular velocity magnitude was inconsistent.

**Solution:** Explicitly program BNO055 UNIT_SEL register:
```cpp
// In bno055_monitor.cpp initializeSensor()
writeRegister(kRegUnitSel, 0x00);  // deg/s, m/s², degrees, Celsius
```

**Lesson:** Never assume IMU default configuration. Always set units explicitly and verify via readback.

### 2. IMU Sign Conventions

**Problem:** IMU mounted backwards gave negative yaw rate for CCW rotation.

**Solution:** Use the correctly-oriented IMU (sensor_1) instead of trying to negate values.

**Lesson:** 
- URDF transformations handle orientation correctly
- Don't fight the TF tree with manual sign inversions
- Choose the sensor with the right mounting orientation

### 3. Wheel Odometry Validation

**Problem:** Initially suspected wheel odometry sign inversion.

**Finding:** Wheel odometry was actually correct!

**Validation Method:**
```python
# Created diagnose_localization.py
- Move forward 0.5m → wheel_odom reports +0.15 m/s ✓
- Rotate CCW 45° → wheel_odom reports +0.30 rad/s ✓
```

**Lesson:** Always validate sensor data with controlled movements before suspecting firmware bugs.

### 4. Lidar Height Matters

**Problem:** Robot couldn't navigate through doorways.

**Root Cause:** Upper lidar (178cm) detected doorframes as obstacles.

**Solution:** Use upper lidar ONLY for localization, lower lidar for navigation.

**Lesson:** Sensor height determines what it sees. Choose the right sensor for each task:
- High sensors → clean walls (localization)
- Low sensors → floor obstacles (navigation)

### 5. TF Frame Conflicts

**Problem:** Initially suspected TF conflicts between wheel_odom and EKF.

**Finding:** No conflict. Odometry messages don't auto-publish TF.

**Lesson:** 
- Odometry messages are just data
- Only nodes like `robot_localization` or TF broadcasters publish transforms
- Check for TF broadcasters with `ros2 run tf2_ros tf2_monitor`

### 6. AMCL Scan Topic Choice

**Initial thought:** Use lower lidar (more common approach).

**Better choice:** Use upper lidar for clean wall features.

**Impact:** 
- Localization stability improved significantly
- Particle filter converges faster
- Less sensitivity to dynamic obstacles

**Lesson:** Don't follow convention blindly. Analyze your specific environment and sensor placement.

### 7. Voxel Layer vs Multiple Obstacle Layers

**Tried:** Separate obstacle layers for each lidar.

**Issue:** Redundant processing, harder to tune.

**Better:** Single voxel layer with height-differentiated sources.

**Final:** Even simpler - just use the lower lidar.

**Lesson:** Start simple. Add complexity only when needed.

---

## Troubleshooting

### Localization Drifts During Rotation

**Symptom:** Robot appears to move backward/forward when rotating in place.

**Likely Causes:**
1. IMU yaw rate sign inverted
2. Wheel odometry angular velocity sign inverted
3. AMCL not updating fast enough

**Debug Steps:**
```bash
# 1. Check wheel odom during rotation
ros2 topic echo /sigyn/wheel_odom
# CCW rotation should show positive twist.angular.z

# 2. Check IMU during rotation  
ros2 topic echo /sigyn/teensy_bridge/imu/sensor_1
# CCW rotation should show positive angular_velocity.z

# 3. Run diagnostic script
python3 scripts/diagnose_localization.py
```

### Laser Scans Appear Rotated in RViz

**Symptom:** Laser scans don't align with robot body or map.

**Likely Causes:**
1. Lidar frame_id doesn't match TF tree
2. URDF lidar mounting incorrect
3. Laser filter not transforming frame

**Debug Steps:**
```bash
# Check scan frame_id
ros2 topic echo --once /scan | grep frame_id

# Check TF from base_link to lidar
ros2 run tf2_ros tf2_echo base_link lidar_frame_top_lidar

# Verify URDF
# Should show correct xyz and rpy for lidar mount
```

### Robot Can't Navigate Through Doorways

**Symptom:** Planner fails, "Optimizer fail to compute path", "Collision Ahead".

**Likely Causes:**
1. High-mounted lidar seeing doorframes as obstacles
2. Inflation radius too large
3. Costmap resolution too coarse

**Solutions:**
```yaml
# Use low lidar for navigation
voxel_layer:
  observation_sources: scan_low  # Don't use high lidar

# Reduce inflation if needed
inflation_layer:
  inflation_radius: 0.30  # Was 0.35
```

### IMU Not Publishing or Intermittent

**Symptom:** `/sigyn/teensy_bridge/imu/sensor_X` has gaps or stops.

**Likely Causes:**
1. I2C communication issues
2. BNO055 not initialized
3. TCA9548A mux not selecting IMU channel

**Debug:**
- Check Teensy diagnostics: `ros2 topic echo /sigyn/teensy_bridge/diagnostics`
- Look for "IMU_FW" version string in logs (confirms initialization)
- Check UNIT_SEL readback value (should be 0x00)

---

## Configuration Summary

### Quick Reference

**EKF (`base/config/ekf.yaml`):**
- Wheel odom: vx, vy, wz
- IMU1: absolute yaw + yaw rate
- IMU0: disabled (redundancy only)

**AMCL (`base/config/navigation_sim.yaml`):**
- Scan topic: `scan_cup` (upper lidar)
- Update thresholds: 0.01m / 0.005 rad

**Local Costmap:**
- Voxel layer: `/scan` (lower lidar) only
- No upper lidar in costmap (blocks doorways)

**Global Costmap:**
- Similar to local, adjusted ranges

---

## Future Improvements

### Potential Enhancements

1. **IMU Failover Logic:**
   - Monitor IMU1 health
   - Auto-switch to IMU0 if IMU1 fails
   - Requires custom node, not just EKF config

2. **Adaptive AMCL Parameters:**
   - Lower particle count in open areas
   - Increase in cluttered spaces
   - Use Nav2's lifecycle management

3. **Dual-Lidar Voxel Layer:**
   - Re-enable upper lidar with height limits
   - Only mark obstacles between 1.5-2.0m
   - Would catch overhead obstacles without blocking doorways

4. **Velocity Smoothing:**
   - Already have `velocity_smoother` in Nav2
   - Could tune for more aggressive motion

---

## References

- [REP-103: Standard Units of Measure and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
- [REP-105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [robot_localization Documentation](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html)
- [Nav2 AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)

---

**Document Maintainer:** AI Agent with Human Validation  
**Change Log:**
- 2025-10-10: Initial documentation based on localization debug session
