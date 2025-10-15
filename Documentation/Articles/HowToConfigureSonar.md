# How to Configure SONAR and Time-of-Flight Sensors for Nav2 Obstacle Avoidance

## Overview

This guide explains how to integrate short-range sensors (SONAR, VL53L0X, or similar time-of-flight sensors) into Nav2's local costmap for near-ground obstacle detection. These sensors are particularly useful for detecting:

- Low obstacles near the floor (shoes, cables, small boxes)
- Objects at corners that lidars miss due to their mounting height
- Dynamic obstacles that appear suddenly in the robot's path

**Key Learning**: Range sensors in Nav2 require careful tuning to avoid either missing obstacles or saturating the costmap with false detections.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Architecture Overview](#architecture-overview)
3. [URDF Configuration](#urdf-configuration)
4. [Nav2 Costmap Configuration](#nav2-costmap-configuration)
5. [Parameter Tuning Guide](#parameter-tuning-guide)
6. [Troubleshooting Common Issues](#troubleshooting-common-issues)
7. [Testing and Validation](#testing-and-validation)

---

## Prerequisites

### Hardware Requirements
- SONAR sensors (e.g., HC-SR04, MaxBotix MB1xxx series) or Time-of-Flight sensors (e.g., VL53L0X, VL53L1X)
- Sensors must publish ROS2 `sensor_msgs/Range` messages
- Mounting hardware to position sensors appropriately

### Software Requirements
- ROS2 (Humble or Jazzy)
- Nav2 navigation stack
- Working local costmap configuration
- Robot URDF with sensor frames defined

### Knowledge Prerequisites
- Basic understanding of Nav2 costmap layers
- Familiarity with URDF/xacro
- Understanding of ROS2 coordinate frames (TF)

---

## Architecture Overview

### How RangeSensorLayer Works

The `RangeSensorLayer` in Nav2 creates **conical projections** from each range reading:

```
        Sensor
          ↓
         / \     ← phi (cone angle)
        /   \
       /     \    
      /_______\   ← marking range
     
     Clear zone  ← clearing beyond threshold
```

**Key Concepts:**

1. **Marking Zone**: When a reading is below `mark_threshold * max_range`, the layer paints a cone of obstacles
2. **Clearing Zone**: When readings are far or at max range, it can optionally clear the costmap
3. **Persistence**: Marks remain until `no_readings_timeout` expires or clearing occurs
4. **Layer Ordering**: Range layer must run AFTER lidar layers to avoid being cleared immediately

---

## URDF Configuration

### Step 1: Define Sensor Frame

Add sensor frames to your robot's URDF. For floor-level detection, sensors should be tilted **upward** so the cone runs parallel to the floor.

#### Example: VL53L0X at Robot Corner

```xml
<!-- In your robot's URDF/xacro file -->

<!-- VL53L0X sensor at front-right corner, tilted up -->
<link name="vl53l0x_front_right_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.02 0.02 0.01"/>  <!-- Approximate sensor size -->
    </geometry>
    <material name="sensor_blue">
      <color rgba="0.2 0.4 0.8 1.0"/>
    </material>
  </visual>
</link>

<joint name="vl53l0x_front_right_joint" type="fixed">
  <parent link="base_link"/>
  <!-- Position at corner, low to ground -->
  <origin xyz="0.25 -0.20 0.05" rpy="0 0.436 0"/>  <!-- 25° upward tilt -->
  <child link="vl53l0x_front_right_link"/>
</joint>
```

#### Calculating the Tilt Angle

For floor-scanning applications:

**Goal**: Sensor cone should start detecting ~3-5cm above floor and sweep upward.

**Formula**:
```
tilt_angle (radians) = atan2(sensor_height, detection_start_distance)

Example:
- Sensor mounted 5cm above floor
- Want to detect objects at 10cm distance
- tilt_angle = atan2(0.05, 0.10) = 0.464 rad = 26.6°
```

**Common tilt angles**:
- 0.436 rad = 25° (gentle upward, good for 5-15cm range)
- 0.524 rad = 30° (steeper, catches closer objects)
- 0.349 rad = 20° (shallow, longer range detection)

#### Sensor Placement Strategy

**For 8-sensor corner coverage** (Sigyn configuration):
```
        Front
    [0]  🤖  [1]     ← Front-left, Front-right
   [2]     |     [3] ← Mid-left, Mid-right
 [4]      base    [5] ← Rear-left, Rear-right
    [6]       [7]     ← Back corners
```

Each sensor points outward from its corner with slight upward tilt.

---

## Nav2 Costmap Configuration

### Step 1: Understand Local Costmap Structure

Your `navigation.yaml` local costmap has a plugin pipeline:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: [
        "voxel_layer",        # Lidar clears/marks first
        "range_sensor_layer", # Range sensors mark AFTER lidar
        "inflation_layer"     # Inflate marked cells last
      ]
```

**Critical**: Range layer must come AFTER the lidar layer to prevent lidar raycasting from immediately clearing range detections.

### Step 2: Add RangeSensorLayer Configuration

Add the following to your `navigation.yaml` under `local_costmap.local_costmap.ros__parameters`:

```yaml
range_sensor_layer:
  enabled: True
  plugin: "nav2_costmap_2d::RangeSensorLayer"
  combination_method: 1   # Max combine - prevents clearing by other layers
  
  # List all your range sensor topics
  topics: [
    "/robot/range/sensor_0",
    "/robot/range/sensor_1",
    "/robot/range/sensor_2",
    # ... add all sensor topics
  ]
  
  # Cone geometry
  phi: 0.087                    # Cone half-angle in radians (~5°)
  inflate_cone: 0.0             # Additional cone inflation (usually 0)
  
  # Timing
  no_readings_timeout: 2.0      # How long marks persist without updates (seconds)
  
  # Marking thresholds (CRITICAL PARAMETERS)
  mark_threshold: 0.95          # Mark if reading < (mark_threshold * max_range)
  clear_threshold: 0.98         # Clear if reading > (clear_threshold * max_range)
  clear_on_max_reading: false   # Don't auto-clear on max readings
  
  # Range limits
  min_range: 0.03               # Ignore readings closer than this (meters)
  max_range: 0.12               # Ignore readings farther than this (meters)
```

### Step 3: Ensure Proper Layer Ordering

**Before** (WRONG - range marks get cleared):
```yaml
plugins: [
  "range_sensor_layer",  # ❌ Lidar clears these marks immediately
  "voxel_layer",
  "inflation_layer"
]
```

**After** (CORRECT - range marks persist):
```yaml
plugins: [
  "voxel_layer",         # ✅ Lidar clears/marks first
  "range_sensor_layer",  # ✅ Range marks added after, survive clearing
  "inflation_layer"      # ✅ Inflate all marked cells
]
```

### Step 4: Controller Configuration

Ensure your MPPI or other controller respects the costmap:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # ... other params ...
      
      CostCritic:
        enabled: true
        consider_footprint: false  # ⚠️ Use false to avoid segfaults with radius-only
        cost_weight: 3.81
        critical_cost: 300.0
        collision_cost: 1000000.0
```

**Note**: `consider_footprint: true` requires a footprint polygon specification. If you only have `robot_radius`, use `false`.

---

## Parameter Tuning Guide

### Understanding the Critical Parameters

#### 1. `phi` (Cone Half-Angle)

**What it does**: Controls the width of the obstacle cone projected from each reading.

**Formula**:
```
cone_width_at_distance = 2 * distance * tan(phi)

Example with phi=0.087 rad (5°):
- At 0.10m: width = 2 * 0.10 * tan(0.087) = 0.0175m (1.75cm)
- At 0.15m: width = 2 * 0.15 * tan(0.087) = 0.0262m (2.62cm)
```

**Tuning guide**:
| Value (rad) | Degrees | Width @ 10cm | Use Case |
|-------------|---------|--------------|----------|
| 0.052       | ~3°     | 1.0cm        | Pinpoint detection, may miss obstacles |
| 0.087       | ~5°     | 1.75cm       | **Recommended starting point** |
| 0.175       | ~10°    | 3.5cm        | Wider coverage, may over-mark |
| 0.262       | ~15°    | 5.3cm        | Very wide, risk of false positives |

**How to tune**:
1. Start with 0.087 (5°)
2. If robot nudges obstacles: increase by 0.02-0.03
3. If costmap turns red or robot won't pass through doors: decrease
4. Monitor in RViz - you should see small cones, not wedges covering half the map

#### 2. `mark_threshold` and `max_range` (The "Bumper" Pair)

These work together to define when obstacles are marked.

**Formula**:
```
Obstacle marked when: sensor_reading < (mark_threshold * max_range)

Example:
- max_range = 0.12m
- mark_threshold = 0.95
- Marks obstacles when reading < 0.114m (95% of 12cm)
```

**Tuning Strategy**:

| Detection Distance | max_range | mark_threshold | Marks when < |
|--------------------|-----------|----------------|--------------|
| Very close (3-8cm) | 0.10m     | 0.80           | 8cm          |
| Near (5-12cm)      | 0.12m     | 0.95           | 11.4cm       |
| Medium (10-20cm)   | 0.25m     | 0.80           | 20cm         |
| Far (20-50cm)      | 0.50m     | 0.90           | 45cm         |

**⚠️ Critical**: Keep `max_range` small to avoid false positives from walls/ceiling!

**For upward-tilted sensors**:
- Sensor sees floor at 10cm, walls at 2m+
- If `max_range = 2.0`, sensor marks walls as obstacles → costmap saturation
- **Solution**: Set `max_range = 0.15` → only near obstacles marked

#### 3. `no_readings_timeout` (Persistence)

**What it does**: How long obstacle marks remain after sensor stops seeing them.

**Tuning guide**:
- **0.5s**: Very aggressive clearing, good for fast-moving robots
- **2.0s**: **Recommended** - gives controller time to react
- **5.0s**: Conservative, marks linger even after obstacle removed
- **Never set < 0.3s** - controller may not have time to plan around obstacle

**When to increase**:
- Robot runs through detected obstacles
- Controller seems to ignore range detections
- Obstacle marks disappear before avoidance kicks in

**When to decrease**:
- Old obstacle marks block paths after object is removed
- Robot gets stuck with phantom obstacles

#### 4. `clear_threshold` and `clear_on_max_reading`

**Purpose**: Control when the layer actively clears marked cells.

**Recommended settings for floor sensors**:
```yaml
clear_threshold: 0.98           # Only clear if reading is very far
clear_on_max_reading: false     # Don't auto-clear on max range
```

**Why `false` for `clear_on_max_reading`**:
- Upward-tilted sensors often return max range (seeing ceiling/wall)
- If `true`, every max reading clears the costmap → obstacles disappear
- If `false`, marks persist until timeout → more stable detection

**When to use `true`**:
- Sensors point horizontally (not tilted)
- Max range genuinely means "no obstacle"
- You want aggressive clearing for dynamic environments

#### 5. `min_range`

**What it does**: Ignores readings closer than this value.

**Tuning**:
- Set to sensor's minimum reliable range (check datasheet)
- VL53L0X: typically 0.03m (3cm)
- SONAR HC-SR04: typically 0.02m (2cm)
- MaxBotix: typically 0.15m (15cm)

**If set too high**: Miss close obstacles  
**If set too low**: Noise and false detections

---

## Troubleshooting Common Issues

### Issue 1: Local Costmap Turns Completely Red

**Symptoms**:
- Entire local costmap filled with obstacles
- Robot refuses to move or plan paths
- RViz shows red everywhere except in small cones

**Cause**: Sensors seeing distant walls/ceiling, marking everything as obstacles.

**Solutions** (in order of likelihood):

1. **Reduce `max_range`**:
   ```yaml
   max_range: 0.12  # Was 0.50 → now only marks very close objects
   ```

2. **Check sensor tilt angle**:
   - Sensors must tilt UP if mounted low
   - Verify in URDF: `rpy="0 0.436 0"` (25° upward)

3. **Tighten `mark_threshold`**:
   ```yaml
   mark_threshold: 0.98  # Only mark when reading is 98% of max_range
   ```

4. **Ensure correct layer order**:
   ```yaml
   plugins: ["voxel_layer", "range_sensor_layer", "inflation_layer"]
   # Range must be AFTER voxel!
   ```

### Issue 2: Robot Runs Through Detected Obstacles

**Symptoms**:
- Sensors show reduced cone in RViz
- Costmap briefly shows red
- Robot continues moving, ignores obstacle

**Solutions**:

1. **Increase `no_readings_timeout`**:
   ```yaml
   no_readings_timeout: 3.0  # Was 0.5 → marks now persist longer
   ```

2. **Widen the cone**:
   ```yaml
   phi: 0.15  # Was 0.087 → wider detection area
   ```

3. **Move range layer earlier** (but after lidar):
   ```yaml
   plugins: ["voxel_layer", "range_sensor_layer", "inflation_layer"]
   ```

4. **Increase inflation radius**:
   ```yaml
   inflation_layer:
     inflation_radius: 0.45  # Was 0.35 → larger keep-out zone
   ```

5. **Check controller is respecting costmap**:
   ```bash
   ros2 topic echo /local_costmap/costmap_updates --once
   # Should show non-zero obstacle cells
   ```

### Issue 3: Range Layer Marks Disappear Immediately

**Symptoms**:
- Sensor detects obstacle
- Costmap briefly shows red
- Mark vanishes before robot reacts

**Cause**: Lidar or other layer is clearing the marks.

**Solutions**:

1. **Verify layer ordering** (range AFTER lidar):
   ```yaml
   plugins: [
     "voxel_layer",        # Lidar first
     "range_sensor_layer", # Range second
     "inflation_layer"
   ]
   ```

2. **Set `combination_method` to Max**:
   ```yaml
   range_sensor_layer:
     combination_method: 1  # Max - marks survive other layers
   ```

3. **Disable aggressive clearing**:
   ```yaml
   clear_on_max_reading: false
   clear_threshold: 0.99
   ```

### Issue 4: YAML Parsing Errors

**Symptoms**:
```
mapping values are not allowed here
```

**Cause**: Incorrect indentation in YAML.

**Check**:
- All `range_sensor_layer` parameters must be indented under `range_sensor_layer:`
- Use spaces, not tabs
- Match indentation of other layers exactly

**Correct**:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      range_sensor_layer:    # Layer name
        enabled: True        # ← 2 spaces indent
        plugin: "..."        # ← 2 spaces indent
        phi: 0.087           # ← 2 spaces indent
```

**Wrong**:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      range_sensor_layer:
    enabled: True            # ❌ Should be indented 2 more spaces
```

### Issue 5: Segmentation Fault on Launch

**Symptoms**:
```
Magick: abort due to signal 11 (SIGSEGV) "Segmentation Fault"
```

**Cause**: MPPI controller's `CostCritic` has `consider_footprint: true` but costmap only defines `robot_radius`.

**Solution**:
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      CostCritic:
        consider_footprint: false  # Use point-based collision checking
```

---

## Testing and Validation

### Step 1: Verify Sensor Data

```bash
# Check sensor topics are publishing
ros2 topic list | grep range

# Verify Range message format
ros2 topic echo /robot/range/sensor_0 --once

# Check publishing rate (should be 5-20 Hz)
ros2 topic hz /robot/range/sensor_0
```

**Expected `sensor_msgs/Range` format**:
```yaml
header:
  frame_id: "vl53l0x_0_link"
radiation_type: 1  # INFRARED (SONAR would be 0)
field_of_view: 0.436  # Should match your URDF
min_range: 0.03
max_range: 2.0  # Device max, not layer max!
range: 0.085  # Current reading in meters
```

### Step 2: Visualize in RViz

Add these displays:

1. **Range visualization**:
   - Add → By Topic → Select range topics
   - Should show cones emanating from sensor frames

2. **Local Costmap**:
   - Add → Map → `/local_costmap/costmap`
   - Should show red cells when obstacle detected

3. **TF frames**:
   - Verify sensor frames exist and are positioned correctly

**What to look for**:
- Range cones point in expected directions
- Cones tilt upward for floor-mounted sensors
- When object placed in cone, costmap shows red cell
- Red cells appear inside the cone projection

### Step 3: Physical Testing

1. **Place test obstacle** (box, shoe, etc.) 10cm in front of a sensor
2. **Observe in RViz**:
   - Range cone should shorten
   - Costmap should show red cell
3. **Send navigation goal** through the obstacle
4. **Expected behavior**:
   - Robot should plan around the obstacle
   - Or stop if no clear path

**If robot ignores obstacle**:
- Increase `no_readings_timeout` to 3.0
- Widen `phi` to 0.12
- Check controller logs for costmap warnings

### Step 4: Parameter Optimization Workflow

```bash
# 1. Start with conservative (narrow, near) values
max_range: 0.10
phi: 0.087
mark_threshold: 0.98

# 2. Test obstacle detection
# Place object, verify costmap marks it

# 3. If marks too small → widen
phi: 0.12

# 4. If costmap saturates → tighten
max_range: 0.08

# 5. If robot ignores obstacles → persist longer
no_readings_timeout: 3.0

# 6. Iterate until balance achieved
```

### Step 5: Validation Checklist

- [ ] All sensor topics publishing at 5+ Hz
- [ ] Sensor frames visible in TF tree
- [ ] Range cones visible in RViz
- [ ] Placing obstacle creates costmap marks
- [ ] Marks persist for at least 1 second
- [ ] Robot plans around detected obstacles
- [ ] Costmap doesn't saturate (turn red everywhere)
- [ ] Robot can still traverse doorways
- [ ] No YAML parsing errors on launch
- [ ] No segmentation faults

---

## Example: Complete Configuration

Here's a complete working example for 8x VL53L0X sensors on a differential drive robot:

### URDF Snippet

```xml
<xacro:macro name="vl53l0x_sensor" params="name parent x y z yaw">
  <link name="${name}_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.01"/>
      </geometry>
      <material name="sensor_blue"/>
    </visual>
  </link>
  
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent}"/>
    <!-- 25° upward tilt for floor scanning -->
    <origin xyz="${x} ${y} ${z}" rpy="0 0.436 ${yaw}"/>
    <child link="${name}_link"/>
  </joint>
</xacro:macro>

<!-- Instantiate 8 sensors at corners -->
<xacro:vl53l0x_sensor name="vl53l0x_0" parent="base_link" 
                       x="0.25" y="0.20" z="0.05" yaw="0.785"/>
<xacro:vl53l0x_sensor name="vl53l0x_1" parent="base_link"
                       x="0.25" y="-0.20" z="0.05" yaw="-0.785"/>
<!-- ... 6 more sensors ... -->
```

### Navigation YAML Snippet

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: True
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.28
      
      plugins: [
        "voxel_layer",
        "range_sensor_layer",
        "inflation_layer"
      ]
      
      voxel_layer:
        enabled: True
        plugin: "nav2_costmap_2d::VoxelLayer"
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: True
          clearing: True
          max_obstacle_height: 2.0
          obstacle_max_range: 2.5
          raytrace_max_range: 3.0
      
      range_sensor_layer:
        enabled: True
        plugin: "nav2_costmap_2d::RangeSensorLayer"
        combination_method: 1
        topics: [
          "/robot/range/vl53l0x_0",
          "/robot/range/vl53l0x_1",
          "/robot/range/vl53l0x_2",
          "/robot/range/vl53l0x_3",
          "/robot/range/vl53l0x_4",
          "/robot/range/vl53l0x_5",
          "/robot/range/vl53l0x_6",
          "/robot/range/vl53l0x_7"
        ]
        phi: 0.087
        inflate_cone: 0.0
        no_readings_timeout: 2.0
        mark_threshold: 0.95
        clear_threshold: 0.98
        clear_on_max_reading: false
        min_range: 0.03
        max_range: 0.12
      
      inflation_layer:
        enabled: True
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.35
        cost_scaling_factor: 8.0
```

---

## Advanced Topics

### Multi-Height Sensor Arrays

For robots that need to detect obstacles at different heights:

```yaml
# Low sensors (0-15cm) - floor obstacles
range_sensor_layer_low:
  topics: ["/range/low_0", "/range/low_1"]
  max_range: 0.15
  
# Mid sensors (15-60cm) - table legs, furniture
range_sensor_layer_mid:
  topics: ["/range/mid_0", "/range/mid_1"]
  max_range: 0.60
  
# High sensors (60-120cm) - counters, shelves
range_sensor_layer_high:
  topics: ["/range/high_0", "/range/high_1"]
  max_range: 1.20
```

### Sensor Fusion with Multiple Lidar Heights

If you have both low and high lidars:

```yaml
plugins: [
  "voxel_layer_low",      # Low lidar
  "voxel_layer_high",     # High lidar
  "range_sensor_layer",   # Fill gaps
  "inflation_layer"
]
```

### Dynamic Reconfiguration

To tune parameters at runtime:

```bash
# List all costmap parameters
ros2 param list /controller_server

# Change a parameter live
ros2 param set /controller_server \
  local_costmap.range_sensor_layer.phi 0.12

# Dump current config
ros2 param dump /controller_server > current_config.yaml
```

---

## References

- [Nav2 Costmap Documentation](https://navigation.ros.org/configuration/packages/costmap-plugins/range.html)
- [RangeSensorLayer Source Code](https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/plugins/range_sensor_layer.cpp)
- [sensor_msgs/Range Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/Range.html)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

---

## Changelog

- **2025-10**: Initial version based on Sigyn robot VL53L0X integration
- Lessons learned from troubleshooting costmap saturation and layer ordering

---

## License

This document is licensed under Apache 2.0. Feel free to use and adapt for your robot projects.
