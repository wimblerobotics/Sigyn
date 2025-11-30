# Sigyn ros2_control Migration Guide

This guide provides a step-by-step plan to convert Sigyn from its current custom control system to ros2_control.

## Table of Contents

1. [Migration Overview](#migration-overview)
2. [Current System Analysis](#current-system-analysis)
3. [Target Architecture](#target-architecture)
4. [Migration Phases](#migration-phases)
5. [Testing Strategy](#testing-strategy)
6. [Rollback Plan](#rollback-plan)

---

## Migration Overview

### Goals

- ✅ Maintain or improve current performance (67 Hz odometry, smooth control)
- ✅ Standardize interface for Nav2 and future controllers
- ✅ Add safety features (cmd_vel timeout, emergency stop)
- ✅ Enable easy addition of new actuators (gripper, elevator fully integrated)
- ✅ Improve sensor integration (VL53L0X, temperature sensors)

### Non-Goals

- ❌ Rewrite TeensyV2 firmware completely (minimal changes)
- ❌ Replace IMU integration (keep separate, it works well)
- ❌ Replace LIDAR drivers (keep separate, they work well)
- ❌ Change robot physical parameters or tuning

### Timeline Estimate

| Phase | Duration | Description |
|-------|----------|-------------|
| 1. Analysis & Planning | 1-2 days | Document current system, plan changes |
| 2. Hardware Interface Development | 3-5 days | Implement SigynSystem, test communication |
| 3. Controller Integration | 2-3 days | Configure diff_drive_controller, test |
| 4. Sensor Addition | 2-3 days | Add VL53L0X, temperature to hardware interface |
| 5. Actuator Addition | 3-5 days | Add gripper, elevator control |
| 6. Integration Testing | 3-5 days | Full system test with Nav2 |
| 7. Tuning & Optimization | 2-3 days | Performance tuning, parameter adjustment |
| **Total** | **16-26 days** | **Approximately 3-5 weeks** |

---

## Current System Analysis

### Odometry Flow (As-Is)

```
┌─────────────────────────────────────────────────────────────────┐
│ TeensyV2 (roboclaw_monitor.cpp)                                 │
│                                                                  │
│ 1. Read encoders from RoboClaw (67 Hz)                         │
│ 2. Calculate odometry locally                                   │
│ 3. Publish JSON to serial:                                      │
│    {                                                             │
│      "type": "wheel_odom",                                      │
│      "x": 1.234,                                                │
│      "y": 0.567,                                                │
│      "theta": 0.123,                                            │
│      "vx": 0.2,                                                 │
│      "vtheta": 0.1                                              │
│    }                                                             │
│ 4. Receive cmd_vel from serial:                                │
│    {                                                             │
│      "type": "cmd_vel",                                         │
│      "linear_x": 0.5,                                           │
│      "angular_z": 0.2                                           │
│    }                                                             │
│ 5. Convert to motor QPPS and send to RoboClaw                  │
└─────────────────────────────────────────────────────────────────┘
                     │                            ▲
                     │ Serial (921600 baud)       │
                     ▼                            │
┌─────────────────────────────────────────────────────────────────┐
│ teensy_bridge.cpp                                                │
│                                                                  │
│ 1. Parse wheel_odom JSON                                        │
│ 2. Publish to /sigyn/wheel_odom (nav_msgs/Odometry)            │
│ 3. Subscribe to /cmd_vel                                        │
│ 4. Send cmd_vel JSON to Teensy                                  │
└─────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│ EKF (robot_localization)                                         │
│                                                                  │
│ Fuses:                                                           │
│ - /sigyn/wheel_odom (velocities only)                          │
│ - /sigyn/teensy_bridge/imu/sensor_1 (orientation)              │
│                                                                  │
│ Publishes:                                                       │
│ - /odometry/filtered                                            │
│ - TF: odom → base_link                                          │
└─────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│ Nav2                                                             │
│                                                                  │
│ Uses /odometry/filtered for localization                        │
│ Publishes /cmd_vel for motion                                   │
└─────────────────────────────────────────────────────────────────┘
```

### What Works Well

1. **High-frequency odometry:** TeensyV2 runs at 67 Hz, faster than most systems
2. **RoboClaw PID:** Closed-loop motor control at 1 kHz
3. **Sensor fusion:** EKF combines wheel odometry + IMU effectively
4. **Communication:** Serial at 921600 baud is fast and reliable

### What Could Be Improved

1. **Non-standard interface:** Nav2 expects `/cmd_vel` subscribed by standard controller
2. **No cmd_vel timeout:** If Nav2 crashes, robot keeps last command
3. **Odometry calculated in two places:** TeensyV2 AND potentially EKF (redundant)
4. **Actuator control scattered:** Gripper/elevator use separate topics, not coordinated
5. **Sensor integration ad-hoc:** VL53L0X published separately, hard to synchronize

---

## Target Architecture

### Odometry Flow (To-Be)

```
┌─────────────────────────────────────────────────────────────────┐
│ TeensyV2 (roboclaw_monitor.cpp) - MINIMAL CHANGES               │
│                                                                  │
│ 1. Read encoders from RoboClaw (67 Hz)                         │
│ 2. Publish JSON to serial:                                      │
│    {                                                             │
│      "type": "encoder_data",                                    │
│      "left_encoder": 123456,     ← Raw counts                   │
│      "right_encoder": 234567,                                   │
│      "left_velocity_qpps": 1200, ← Optional, for validation     │
│      "right_velocity_qpps": 1180,                               │
│      "timestamp_ms": 123456789                                  │
│    }                                                             │
│ 3. Receive motor commands from serial:                          │
│    {                                                             │
│      "type": "motor_command",                                   │
│      "left_qpps": 1200,          ← QPPS units                   │
│      "right_qpps": 1180                                         │
│    }                                                             │
│ 4. Send to RoboClaw                                             │
│                                                                  │
│ NO MORE ODOMETRY CALCULATION - That's now ros2_control's job!   │
└─────────────────────────────────────────────────────────────────┘
                     │                            ▲
                     │ Serial (921600 baud)       │
                     ▼                            │
┌─────────────────────────────────────────────────────────────────┐
│ sigyn_hardware_interface (SigynSystem)                          │
│                                                                  │
│ read():                                                          │
│ 1. Parse encoder_data JSON                                      │
│ 2. Convert encoder counts → wheel positions (radians)           │
│ 3. Calculate velocities from position deltas                    │
│ 4. Update state interfaces                                      │
│                                                                  │
│ write():                                                         │
│ 1. Get commanded velocities from command interfaces             │
│ 2. Convert rad/s → QPPS                                         │
│ 3. Send motor_command JSON                                      │
│                                                                  │
│ Also handles:                                                    │
│ - VL53L0X sensor reading                                        │
│ - Temperature sensor reading                                    │
│ - Gripper/elevator joint control                                │
└─────────────────────────────────────────────────────────────────┘
                     │                            ▲
                     ▼                            │
┌─────────────────────────────────────────────────────────────────┐
│ ros2_control (controller_manager)                               │
│                                                                  │
│ ┌───────────────────────────────────────────────────────────┐  │
│ │ diff_drive_controller                                      │  │
│ │ - Subscribes: /cmd_vel                                     │  │
│ │ - Commands: left/right wheel velocities                    │  │
│ │ - Publishes: /diff_drive_controller/odom                   │  │
│ │ - Publishes TF: odom → base_link                          │  │
│ │ - Safety: cmd_vel timeout (0.5s default)                   │  │
│ └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│ ┌───────────────────────────────────────────────────────────┐  │
│ │ joint_state_broadcaster                                    │  │
│ │ - Publishes: /joint_states (all joints + wheels)          │  │
│ └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│ ┌───────────────────────────────────────────────────────────┐  │
│ │ gripper_controller (joint_trajectory_controller)           │  │
│ │ - Controls: gripper joint                                  │  │
│ └───────────────────────────────────────────────────────────┘  │
│                                                                  │
│ ┌───────────────────────────────────────────────────────────┐  │
│ │ elevator_controller (joint_trajectory_controller)          │  │
│ │ - Controls: elevator joint                                 │  │
│ └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│ EKF (robot_localization) - OPTIONAL                             │
│                                                                  │
│ Fuses:                                                           │
│ - /diff_drive_controller/odom (velocities only)                │
│ - /sigyn/teensy_bridge/imu/sensor_1 (orientation)              │
│                                                                  │
│ Publishes:                                                       │
│ - /odometry/filtered                                            │
│ - TF: odom → base_link (or use diff_drive's TF directly)       │
└─────────────────────────────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────────┐
│ Nav2                                                             │
│                                                                  │
│ Uses /odometry/filtered (or /diff_drive_controller/odom)       │
│ Publishes /cmd_vel                                              │
└─────────────────────────────────────────────────────────────────┘
```

### Key Changes

1. **TeensyV2:** Reports raw encoder data, doesn't calculate odometry
2. **Hardware Interface:** Does unit conversion, minimal processing
3. **diff_drive_controller:** Calculates odometry from wheel states
4. **EKF:** Optional (may use diff_drive odometry directly for simplicity)
5. **All actuators:** Controlled through ros2_control

---

## Migration Phases

### Phase 0: Preparation (1-2 days)

**Goal:** Ensure we can safely revert if something goes wrong.

#### Tasks

1. **Create migration branch**
   ```bash
   cd ~/sigyn_ws/src/Sigyn
   git checkout -b ros2_control_migration
   git push -u origin ros2_control_migration
   ```

2. **Document baseline performance**
   ```bash
   # Run current system
   ros2 launch base sigyn.launch.py
   
   # In separate terminals, capture:
   # - Odometry accuracy (drive 1m straight, measure error)
   # - Rotation accuracy (rotate 360°, measure drift)
   # - Response latency (time from cmd_vel to motion)
   # - CPU usage (top, monitor controller_manager process)
   
   # Save results to baseline_performance.md
   ```

3. **Create test scenarios**
   ```markdown
   # tests/test_scenarios.md
   
   ## Test 1: Straight Line
   - Command: 0.5 m/s for 10 seconds
   - Expected: 5.0m ± 0.05m forward motion
   
   ## Test 2: Rotation
   - Command: 1.0 rad/s for 6.28 seconds (2*pi)
   - Expected: Return to starting orientation ± 5°
   
   ## Test 3: Square Pattern
   - Drive 1m, turn 90°, repeat 4 times
   - Expected: Return to start position ± 0.1m
   
   ## Test 4: cmd_vel Timeout
   - Send cmd_vel, then stop publishing
   - Expected: Robot stops within 0.5 seconds
   
   ## Test 5: Nav2 Integration
   - Use Nav2 to navigate to goal 5m away
   - Expected: Reach goal within tolerance
   ```

4. **Backup configuration**
   ```bash
   # Save current working configs
   cp -r base/config base/config_backup
   cp -r base/launch base/launch_backup
   ```

---

### Phase 1: TeensyV2 Modifications (1-2 days)

**Goal:** Modify TeensyV2 to publish raw encoder data instead of calculated odometry.

#### Changes to `roboclaw_monitor.cpp`

**Current code** (roboclaw_monitor.cpp):
```cpp
void RoboClawMonitor::updateOdometry() {
    // Calculate pose
    current_pose_.x += delta_distance * cos(current_pose_.theta + delta_theta / 2.0f);
    current_pose_.y += delta_distance * sin(current_pose_.theta + delta_theta / 2.0f);
    current_pose_.theta += delta_theta;
    
    // Publish odometry
    publishOdometry();
}
```

**New code** (roboclaw_monitor.cpp):
```cpp
void RoboClawMonitor::publishEncoderData() {
    // Just report raw data, let ros2_control calculate odometry
    StaticJsonDocument<256> doc;
    doc["type"] = "encoder_data";
    doc["left_encoder"] = motor1_status_.encoder_count;
    doc["right_encoder"] = motor2_status_.encoder_count;
    doc["left_velocity_qpps"] = motor1_status_.speed_qpps;
    doc["right_velocity_qpps"] = motor2_status_.speed_qpps;
    doc["timestamp_ms"] = millis();
    
    std::string message;
    serializeJson(doc, message);
    serial_manager_.queueMessage("encoder_data", message);
}

// In loop():
if (now - last_reading_time_ms_ >= 15) {  // Still 67 Hz
    updateCriticalMotorStatus();
    publishEncoderData();  // Changed from updateOdometry()
    last_reading_time_ms_ = now;
}
```

**Motor command handling:**
```cpp
void RoboClawMonitor::processMotorCommand(const std::string& message) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, message);
    
    if (doc["type"] == "motor_command") {
        int32_t left_qpps = doc["left_qpps"];
        int32_t right_qpps = doc["right_qpps"];
        
        // Send to RoboClaw (existing code)
        setM1Velocity(address_, left_qpps);
        setM2Velocity(address_, right_qpps);
    }
}
```

#### Testing TeensyV2 Changes

```bash
# Build TeensyV2 firmware
cd TeensyV2
platformio run --environment teensy41

# Upload to Teensy (use your upload script)
./scripts/ty_upload.sh

# Monitor serial output
platformio device monitor --baud 921600

# Should see:
# {"type":"encoder_data","left_encoder":12345,"right_encoder":23456,...}
```

**Rollback:** Keep old firmware binary, can flash back if needed.

---

### Phase 2: Hardware Interface Implementation (3-5 days)

**Goal:** Implement `SigynSystem` hardware interface that communicates with TeensyV2.

#### File: `sigyn_hardware_interface/src/sigyn_system.cpp`

**Key methods to implement:**

1. **on_init():** Parse URDF parameters
2. **on_configure():** Open serial port
3. **on_activate():** Start communication
4. **read():** Parse encoder data, update wheel states
5. **write():** Send motor commands

**Critical: Unit Conversions**

```cpp
// Constants from TeensyV2
constexpr uint32_t QUADRATURE_PULSES_PER_REV = 1000;
constexpr uint32_t COUNTS_PER_REV = QUADRATURE_PULSES_PER_REV * 4;  // 4000
constexpr double WHEEL_DIAMETER_M = 0.102224144529039;
constexpr double WHEEL_CIRCUMFERENCE_M = M_PI * WHEEL_DIAMETER_M;

// Encoder counts → Wheel radians
double countsToRadians(int32_t counts) {
    return (2.0 * M_PI * counts) / COUNTS_PER_REV;
}

// Wheel rad/s → Motor QPPS (quadrature pulses per second)
int32_t radiansPerSecToQPPS(double rad_per_sec) {
    // rad/s → rev/s → pulses/s
    double rev_per_sec = rad_per_sec / (2.0 * M_PI);
    return static_cast<int32_t>(rev_per_sec * QUADRATURE_PULSES_PER_REV);
}
```

#### Implementation Example

```cpp
return_type SigynSystem::read(const Time& time, const Duration& period) {
    // Read from serial (non-blocking)
    std::string message = readFromSerial();
    
    if (message.empty()) {
        // No new data, check timeout
        if ((time - last_read_time_).seconds() > config_.read_timeout) {
            RCLCPP_ERROR(getLogger(), "Read timeout!");
            return return_type::ERROR;
        }
        // Use previous values
        return return_type::OK;
    }
    
    // Parse JSON
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, message);
    if (error) {
        RCLCPP_WARN(getLogger(), "JSON parse error: %s", error.c_str());
        return return_type::OK;  // Non-fatal
    }
    
    if (doc["type"] == "encoder_data") {
        // Get encoder counts
        int32_t left_counts = doc["left_encoder"];
        int32_t right_counts = doc["right_encoder"];
        
        // Convert to radians
        hw_positions_[0] = countsToRadians(left_counts);
        hw_positions_[1] = countsToRadians(right_counts);
        
        // Calculate velocities from position delta
        double dt = period.seconds();
        if (dt > 0.0) {
            hw_velocities_[0] = (hw_positions_[0] - prev_positions_[0]) / dt;
            hw_velocities_[1] = (hw_positions_[1] - prev_positions_[1]) / dt;
        }
        
        prev_positions_[0] = hw_positions_[0];
        prev_positions_[1] = hw_positions_[1];
        last_read_time_ = time;
    }
    
    return return_type::OK;
}

return_type SigynSystem::write(const Time&, const Duration&) {
    // Convert commanded velocities (rad/s) to QPPS
    int32_t left_qpps = radiansPerSecToQPPS(hw_commands_[0]);
    int32_t right_qpps = radiansPerSecToQPPS(hw_commands_[1]);
    
    // Build JSON command
    StaticJsonDocument<128> doc;
    doc["type"] = "motor_command";
    doc["left_qpps"] = left_qpps;
    doc["right_qpps"] = right_qpps;
    
    std::string cmd;
    serializeJson(doc, cmd);
    cmd += "\n";  // Newline delimiter
    
    // Send to serial
    ssize_t bytes_written = ::write(teensy_fd_, cmd.c_str(), cmd.length());
    if (bytes_written < 0) {
        RCLCPP_ERROR(getLogger(), "Serial write failed!");
        return return_type::ERROR;
    }
    
    return return_type::OK;
}
```

#### Testing Hardware Interface

```bash
# Build
cd ~/sigyn_ws
colcon build --packages-select sigyn_hardware_interface --symlink-install
source install/setup.bash

# Test with fake hardware first (no Teensy)
ros2 launch sigyn_hardware_interface test_fake_hardware.launch.py

# Check interfaces are exposed
ros2 control list_hardware_interfaces

# Should see:
# left_wheel_joint/position [available] [claimed]
# left_wheel_joint/velocity [available] [claimed]
# right_wheel_joint/position [available] [claimed]
# right_wheel_joint/velocity [available] [claimed]

# Test commanding
ros2 topic pub /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}" --rate 10
```

---

### Phase 3: Controller Configuration (2-3 days)

**Goal:** Configure and test diff_drive_controller with hardware interface.

#### File: `sigyn_hardware_interface/config/controllers.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Match or slightly lower than TeensyV2's 67 Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    
    # FROM TEENSY V2 CONSTANTS:
    wheel_separation: 0.3906           # WHEEL_BASE_M
    wheel_radius: 0.051112072264520    # WHEEL_DIAMETER_M / 2
    
    # Tuning (start with 1.0, adjust based on testing)
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    
    publish_rate: 50.0
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    
    # Conservative odometry covariance (tune based on testing)
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    
    open_loop: false
    enable_odom_tf: true  # Or false if using EKF for TF
    
    cmd_vel_timeout: 0.5  # SAFETY FEATURE - stop if no cmd_vel
    
    # Velocity limits (from navigation_sim.yaml)
    linear.x.has_velocity_limits: true
    linear.x.max_velocity: 1.0
    linear.x.min_velocity: -1.0
    linear.x.has_acceleration_limits: true
    linear.x.max_acceleration: 2.0
    linear.x.min_acceleration: -2.0
    
    angular.z.has_velocity_limits: true
    angular.z.max_velocity: 2.0
    angular.z.min_velocity: -2.0
    angular.z.has_acceleration_limits: true
    angular.z.max_acceleration: 4.0
    angular.z.min_acceleration: -4.0
```

#### Testing Controllers

```bash
# Launch with real Teensy
ros2 launch sigyn_hardware_interface sigyn_control.launch.py

# Verify controllers loaded
ros2 control list_controllers
# Should show:
# diff_drive_controller[diff_drive_controller/DiffDriveController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# Check topics
ros2 topic list
# Should include:
# /cmd_vel (subscribed by diff_drive_controller)
# /diff_drive_controller/odom (published by diff_drive_controller)
# /joint_states (published by joint_state_broadcaster)

# Test driving
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Monitor odometry
ros2 topic echo /diff_drive_controller/odom

# Run test scenarios from Phase 0
```

#### Odometry Calibration

If odometry doesn't match baseline:

```bash
# Drive robot 1.0 meter forward
# Measure actual distance traveled: e.g., 0.95 meters

# Correction factor: 0.95 / 1.0 = 0.95
# Update wheel_radius_multiplier in controllers.yaml:
wheel_radius_multiplier: 0.95

# Or if turning is off, adjust wheel_separation_multiplier
```

---

### Phase 4: Nav2 Integration (2-3 days)

**Goal:** Integrate ros2_control with Nav2 stack.

#### Update `base/launch/sigyn.launch.py`

```python
# OLD CODE (remove or comment out):
# teensy_bridge_node = Node(
#     package='sigyn_to_sensor_v2',
#     executable='teensy_bridge',
#     ...
# )

# NEW CODE (add):
sigyn_control_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('sigyn_hardware_interface'),
            'launch',
            'sigyn_control.launch.py'
        )
    )
)

return LaunchDescription([
    # ... other nodes ...
    sigyn_control_launch,  # Replace teensy_bridge
    # ... rest of nodes ...
])
```

#### Update EKF Configuration

**Option A: Use diff_drive odometry directly (simpler)**

```python
# Don't launch EKF, use diff_drive_controller's TF directly
# Update Nav2 to use /diff_drive_controller/odom
```

**Option B: Keep EKF for sensor fusion (recommended)**

```yaml
# base/config/ekf.yaml

ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    
    # Change odom0 source
    odom0: /diff_drive_controller/odom  # Changed from /sigyn/wheel_odom
    odom0_config: [false, false, false,  # Don't fuse position
                   false, false, false,  # Don't fuse orientation
                   true,  true,  false,  # Fuse vx, vy
                   false, false, true,   # Fuse vtheta
                   false, false, false]
    
    # Keep IMU unchanged
    imu1: /sigyn/teensy_bridge/imu/sensor_1
    # ... rest unchanged ...
```

#### Testing Nav2

```bash
# Launch full Sigyn stack
ros2 launch base sigyn.launch.py

# Verify TF tree
ros2 run tf2_tools view_frames
# Should show: map -> odom -> base_link -> ... (sensors)

# Test Nav2 navigation
ros2 launch base sigyn.launch.py
# In RViz:
# 1. Set initial pose (2D Pose Estimate)
# 2. Set goal (2D Goal Pose)
# 3. Watch robot navigate

# Run full test scenarios from Phase 0
# Compare performance to baseline
```

---

### Phase 5: Sensor Integration (2-3 days)

**Goal:** Add VL53L0X and temperature sensors to ros2_control.

#### Update URDF

```xml
<!-- base/description/urdf/ros2_control.xacro -->

<ros2_control name="sigyn_system" type="system">
  <!-- ... existing wheel joints ... -->
  
  <!-- VL53L0X Distance Sensors -->
  <sensor name="vl53l0x_front">
    <state_interface name="range"/>
    <param name="frame_id">vl53l0x_front</param>
    <param name="min_range">0.03</param>
    <param name="max_range">2.0</param>
  </sensor>
  
  <sensor name="vl53l0x_left">
    <state_interface name="range"/>
    <param name="frame_id">vl53l0x_left</param>
  </sensor>
  
  <!-- Add more VL53L0X sensors as needed -->
  
  <!-- Temperature Sensors -->
  <sensor name="temp_board">
    <state_interface name="temperature"/>
    <param name="frame_id">temp_board</param>
  </sensor>
</ros2_control>
```

#### Update Hardware Interface

```cpp
// In SigynSystem class:

// Add to export_state_interfaces():
state_interfaces.emplace_back("vl53l0x_front", "range", &sensor_range_front_);
state_interfaces.emplace_back("temp_board", "temperature", &sensor_temp_board_);

// Add to read():
if (doc["type"] == "sensor_data") {
    sensor_range_front_ = doc["vl53l0x_front"]["range"];
    sensor_temp_board_ = doc["temp_board"]["temperature"];
    
    // Also publish as ROS messages for non-control consumers
    publishRangeSensor("vl53l0x_front", sensor_range_front_);
    publishTemperature("temp_board", sensor_temp_board_);
}
```

#### Update TeensyV2

Add sensor data to JSON output:

```cpp
void publishSensorData() {
    StaticJsonDocument<512> doc;
    doc["type"] = "sensor_data";
    
    // VL53L0X readings
    doc["vl53l0x_front"]["range"] = readVL53L0X(FRONT_SENSOR);
    doc["vl53l0x_left"]["range"] = readVL53L0X(LEFT_SENSOR);
    
    // Temperature readings
    doc["temp_board"]["temperature"] = readTemperature(BOARD_TEMP_SENSOR);
    
    std::string message;
    serializeJson(doc, message);
    serial_manager_.queueMessage("sensor_data", message);
}

// Call at appropriate rate (e.g., 10 Hz for sensors)
if (now - last_sensor_time_ms_ >= 100) {
    publishSensorData();
    last_sensor_time_ms_ = now;
}
```

---

### Phase 6: Actuator Integration (3-5 days)

**Goal:** Add gripper and elevator control through ros2_control.

#### Update URDF

```xml
<!-- Gripper joint -->
<joint name="gripper_joint">
  <command_interface name="position">
    <param name="min">0.0</param>
    <param name="max">0.05</param>  <!-- Max opening in meters -->
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>  <!-- Current sensing -->
</joint>

<!-- Elevator joint -->
<joint name="elevator_joint">
  <command_interface name="position">
    <param name="min">0.0</param>
    <param name="max">1.2</param>  <!-- Max height in meters -->
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

#### Configure Controllers

```yaml
# Add to controllers.yaml

gripper_controller:
  ros__parameters:
    joints: [gripper_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity, effort]
    
    constraints:
      stopped_velocity_tolerance: 0.01
      gripper_joint:
        trajectory: 0.01
        goal: 0.005

elevator_controller:
  ros__parameters:
    joints: [elevator_joint]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
    
    constraints:
      stopped_velocity_tolerance: 0.01
      elevator_joint:
        trajectory: 0.02
        goal: 0.01
```

#### Control Examples

```bash
# Open gripper
ros2 topic pub /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['gripper_joint'], points: [{positions: [0.05], time_from_start: {sec: 1}}]}" \
  --once

# Close gripper
ros2 topic pub /gripper_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['gripper_joint'], points: [{positions: [0.0], time_from_start: {sec: 1}}]}" \
  --once

# Move elevator to height
ros2 topic pub /elevator_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['elevator_joint'], points: [{positions: [0.8], time_from_start: {sec: 3}}]}" \
  --once
```

---

## Testing Strategy

### Unit Tests

Create test suite in `sigyn_hardware_interface/test/`:

```cpp
// test_unit_conversions.cpp
TEST(UnitConversions, EncoderCountsToRadians) {
  EXPECT_NEAR(countsToRadians(4000), 2*M_PI, 0.001);  // 1 revolution
  EXPECT_NEAR(countsToRadians(2000), M_PI, 0.001);    // Half revolution
}

TEST(UnitConversions, RadiansPerSecToQPPS) {
  EXPECT_EQ(radiansPerSecToQPPS(2*M_PI), 1000);  // 1 rev/s = 1000 QPPS
}
```

### Integration Tests

```bash
# test_communication.launch.py - Test serial communication
ros2 launch sigyn_hardware_interface test_communication.launch.py

# Verify:
# - Can open serial port
# - Can parse encoder data
# - Can send motor commands
# - Latency < 10ms
```

### System Tests

Run all test scenarios from Phase 0 and compare results.

**Acceptance Criteria:**
- ✅ Odometry accuracy within 5% of baseline
- ✅ Response latency within 10% of baseline
- ✅ CPU usage < 20% for controller_manager
- ✅ No missed controller update cycles
- ✅ cmd_vel timeout works (robot stops within 0.5s)
- ✅ Nav2 can navigate successfully
- ✅ All sensors publish data correctly
- ✅ Actuators respond to commands smoothly

---

## Rollback Plan

If migration fails, rollback steps:

1. **Revert TeensyV2 firmware**
   ```bash
   cd TeensyV2
   # Flash backup firmware
   platformio run --environment teensy41 --target upload
   ```

2. **Revert ROS2 code**
   ```bash
   cd ~/sigyn_ws/src/Sigyn
   git checkout main  # Or previous working branch
   colcon build --packages-select base sigyn_to_sensor_v2
   ```

3. **Restore configuration**
   ```bash
   cp -r base/config_backup/* base/config/
   cp -r base/launch_backup/* base/launch/
   ```

4. **Verify system works**
   ```bash
   ros2 launch base sigyn.launch.py
   # Run basic tests
   ```

---

## Success Metrics

Migration is successful when:

- ✅ All Phase 0 test scenarios pass
- ✅ Performance meets or exceeds baseline
- ✅ No regression in existing functionality
- ✅ New features work (cmd_vel timeout, actuator control)
- ✅ Code is cleaner and more maintainable
- ✅ Documentation is updated

---

## Post-Migration Tasks

1. **Update documentation**
   - Update README.md
   - Update architecture diagrams
   - Document new launch files

2. **Create tutorial videos/docs**
   - How to use new actuator controllers
   - How to add new sensors
   - Troubleshooting guide

3. **Clean up old code**
   - Remove unused teensy_bridge code
   - Archive old configurations
   - Tag release: `v2.0.0-ros2-control`

4. **Plan future enhancements**
   - Custom safety controller for VL53L0X
   - Advanced gripper control (force feedback)
   - Additional sensor integration

---

**This migration transforms Sigyn into a ros2_control-based system while preserving the high-performance characteristics that make it special. The phased approach ensures you can validate each step before proceeding, with clear rollback options if needed.**

**Good luck with your migration! 🤖**
