# Advanced Topics in ros2_control

This document covers advanced concepts you'll encounter when working with ros2_control in production systems like Sigyn.

## Table of Contents

1. [Real-Time Performance](#real-time-performance)
2. [Custom Controllers](#custom-controllers)
3. [Sensor Integration Patterns](#sensor-integration-patterns)
4. [Multi-Interface Hardware](#multi-interface-hardware)
5. [Error Handling and Recovery](#error-handling-and-recovery)
6. [Performance Tuning](#performance-tuning)

---

## Real-Time Performance

### Understanding Real-Time in ros2_control

**Key Principle:** Controller update loop must complete within deadline (e.g., 20ms for 50Hz).

```
Ideal 50Hz cycle (20ms):
├─ read()  : 2ms  ← Hardware communication
├─ controllers: 5ms  ← Computation
├─ write() : 2ms  ← Hardware communication
└─ slack   : 11ms ← Safety margin
```

**What breaks real-time:**
- Blocking I/O (slow serial reads)
- Memory allocations in hot path
- Complex computations
- Lock contention

### Sigyn's Real-Time Strategy

**TeensyV2 Side:**
```cpp
// High-frequency operations (67 Hz)
if (now - last_reading_time_ms_ >= 15) {
    updateCriticalMotorStatus();  // Non-blocking
    updateOdometry();              // Fast math
    last_reading_time_ms_ = now;
}

// Low-frequency operations (3 Hz) - Don't block critical path
if (now - last_status_report_time_ms_ >= 333) {
    updateSystemStatus();  // Slow I2C reads
    last_status_report_time_ms_ = now;
}
```

**ros2_control Hardware Interface:**
```cpp
return_type SigynSystem::read(const Time&, const Duration& period) {
    // Use non-blocking serial read with timeout
    std::string msg = readFromSerial();  // Returns immediately if no data
    
    if (!msg.empty()) {
        parseEncoderMessage(msg);  // Fast parsing
    }
    // If no new data, use previous values - don't wait!
    
    encoderPositionsToWheelPositions();  // Pure math, very fast
    
    return return_type::OK;
}
```

**Best Practices:**
1. **Non-blocking I/O:** Never wait for hardware in read()/write()
2. **Preallocate:** No `new` or `malloc()` in hot path
3. **Defer:** Move slow operations to separate threads
4. **Profile:** Measure and optimize bottlenecks

---

## Custom Controllers

### When to Create Custom Controllers

**Use standard controllers when:**
- ✅ Standard diff_drive works for your robot
- ✅ Joint trajectory controller meets needs
- ✅ Effort controllers work for force control

**Create custom when:**
- ❌ Need special kinematics (e.g., mecanum wheels)
- ❌ Safety logic specific to your robot
- ❌ Sensor-based control (e.g., visual servoing)
- ❌ Complex state machines

### Example: Sigyn Safety Controller

**Scenario:** Stop robot if front VL53L0X detects obstacle < 10cm while moving forward.

**Implementation:**

```cpp
// sigyn_controllers/safety_controller.hpp
class SigynSafetyController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {"left_wheel_joint/velocity", "right_wheel_joint/velocity"}
    };
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override {
    return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::INDIVIDUAL,
      {"left_wheel_joint/velocity", "right_wheel_joint/velocity",
       "vl53l0x_front/range"}
    };
  }

  controller_interface::return_type update(
    const rclcpp::Time&, const rclcpp::Duration&) override {
    
    // Read current state
    double left_vel = state_interfaces_[0].get_value();
    double right_vel = state_interfaces_[1].get_value();
    double range_front = state_interfaces_[2].get_value();
    
    // Check if moving forward
    bool moving_forward = (left_vel > 0.01 && right_vel > 0.01);
    
    // Emergency stop if too close
    if (moving_forward && range_front < 0.10) {
      RCLCPP_WARN(get_node()->get_logger(), "Obstacle detected! Emergency stop.");
      command_interfaces_[0].set_value(0.0);  // Stop left wheel
      command_interfaces_[1].set_value(0.0);  // Stop right wheel
      
      // Publish event
      publishSafetyEvent("obstacle_detected", range_front);
    }
    
    return controller_interface::return_type::OK;
  }
};
```

**Activation:**
```yaml
# controllers.yaml
safety_controller:
  ros__parameters:
    update_rate: 50.0
    emergency_stop_distance: 0.10  # meters
```

**Run alongside diff_drive:**
```bash
ros2 control load_controller safety_controller
ros2 control set_controller_state safety_controller active
# Now both diff_drive_controller AND safety_controller are active
```

---

## Sensor Integration Patterns

### Pattern 1: State Interface Only (Read-Only Sensors)

**Best for:** Temperature, distance, IMU raw data

```xml
<!-- URDF -->
<sensor name="front_distance">
  <state_interface name="range"/>
</sensor>
```

```cpp
// Hardware interface
std::vector<StateInterface> export_state_interfaces() {
  interfaces.emplace_back("front_distance", "range", &range_front_);
  return interfaces;
}

return_type read(const Time&, const Duration&) {
  range_front_ = readDistanceSensor();  // Update shared variable
  
  // Also publish as ROS message for non-control consumers
  range_msg_.range = range_front_;
  range_pub_->publish(range_msg_);
}
```

**Usage:**
- Controllers can read `front_distance/range` state interface
- Other nodes subscribe to `/front_distance` topic
- Best of both worlds!

### Pattern 2: Command + State Interface (Actuated Sensors)

**Best for:** Pan-tilt units, active sensors with control

```xml
<joint name="camera_pan">
  <command_interface name="position"/>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

### Pattern 3: GPIO/Digital Outputs

**Best for:** LEDs, relays, digital sensors

```xml
<gpio name="status_led">
  <command_interface name="digital_output"/>
  <state_interface name="digital_output"/>
</gpio>
```

```cpp
// In hardware interface
std::vector<CommandInterface> export_command_interfaces() {
  interfaces.emplace_back("status_led", "digital_output", &led_state_);
  return interfaces;
}

return_type write(const Time&, const Duration&) {
  // led_state_ is controlled by some controller
  setGPIOPin(LED_PIN, led_state_ > 0.5);  // HIGH if > 0.5, LOW otherwise
}
```

### Pattern 4: Sensor Fusion (IMU Example)

**Best for:** Complex sensors that need processing before use

**Recommendation for Sigyn:** Keep IMUs separate, not in ros2_control

**Why?**
- IMU fusion is complex (orientation, bias correction, calibration)
- Dedicated `robot_localization` (EKF) node is mature and tested
- ros2_control hardware interface should be simple and fast

**Architecture:**
```
TeensyV2 → publishes /imu/data (sensor_msgs/Imu)
           ↓
robot_localization (EKF) ← also takes /wheel_odom
           ↓
     /odometry/filtered
           ↓
        Nav2
```

---

## Multi-Interface Hardware

### Scenario: Sigyn's Gripper with Position AND Effort Control

```xml
<joint name="gripper_joint">
  <!-- Can command either position OR effort, not both simultaneously -->
  <command_interface name="position"/>
  <command_interface name="effort"/>
  
  <state_interface name="position"/>
  <state_interface name="velocity"/>
  <state_interface name="effort"/>  <!-- Current sensing -->
</joint>
```

**Controller A: Position Control (Normal Operation)**
```yaml
gripper_position_controller:
  type: position_controllers/JointTrajectoryController
  joints: [gripper_joint]
  command_interfaces: [position]
```

**Controller B: Force Control (Gentle Grasping)**
```yaml
gripper_force_controller:
  type: effort_controllers/JointGroupEffortController
  joints: [gripper_joint]
  command_interfaces: [effort]
```

**Switching:**
```bash
# For delicate objects - use force control
ros2 control switch_controllers \
  --deactivate gripper_position_controller \
  --activate gripper_force_controller

# For normal objects - use position control
ros2 control switch_controllers \
  --deactivate gripper_force_controller \
  --activate gripper_position_controller
```

---

## Error Handling and Recovery

### Hardware Interface Error Handling

```cpp
return_type SigynSystem::read(const Time& time, const Duration&) {
  // Check for stale data
  if ((time - last_read_time_).seconds() > config_.read_timeout) {
    RCLCPP_ERROR(getLogger(), "Read timeout! No data from Teensy.");
    
    // Return error - controller manager will deactivate controllers
    return return_type::ERROR;
  }
  
  // Check for invalid data
  if (!validateEncoderData(encoder_left_, encoder_right_)) {
    RCLCPP_WARN_THROTTLE(getLogger(), clock_, 1000, 
                         "Invalid encoder data received");
    // Don't update positions - use previous values
    return return_type::OK;  // Non-fatal, continue
  }
  
  // Normal operation
  updateWheelStates();
  last_read_time_ = time;
  return return_type::OK;
}

return_type SigynSystem::write(const Time&, const Duration&) {
  // Check for NaN commands
  if (std::isnan(hw_commands_[0]) || std::isnan(hw_commands_[1])) {
    RCLCPP_ERROR(getLogger(), "NaN command received! Sending zero.");
    return sendZeroVelocity();
  }
  
  // Check for communication error
  if (!sendVelocityCommand(hw_commands_[0], hw_commands_[1])) {
    comm_error_count_++;
    if (comm_error_count_ > MAX_COMM_ERRORS) {
      RCLCPP_ERROR(getLogger(), "Too many communication errors!");
      return return_type::ERROR;  // Fatal, stop robot
    }
  } else {
    comm_error_count_ = 0;  // Reset on success
  }
  
  return return_type::OK;
}
```

### Controller-Level Safety

```cpp
// In diff_drive_controller or custom controller
controller_interface::return_type update(const Time& time, const Duration& period) {
  // Check for command timeout
  auto cmd_age = time - last_cmd_vel_time_;
  if (cmd_age.seconds() > cmd_vel_timeout_) {
    // No recent cmd_vel - stop robot
    writeZeroVelocity();
    return return_type::OK;
  }
  
  // Clamp velocities to limits
  double left_vel = std::clamp(desired_left_, -max_vel_, max_vel_);
  double right_vel = std::clamp(desired_right_, -max_vel_, max_vel_);
  
  // Apply acceleration limits
  left_vel = applyAccelLimit(left_vel, prev_left_, max_accel_, period);
  right_vel = applyAccelLimit(right_vel, prev_right_, max_accel_, period);
  
  // Write to hardware
  command_interfaces_[0].set_value(left_vel);
  command_interfaces_[1].set_value(right_vel);
  
  return return_type::OK;
}
```

### Recovery Behaviors

**Automatic Recovery:**
```cpp
// In on_error() callback
CallbackReturn on_error(const State& previous_state) override {
  // Try to recover communication
  if (reconnectToHardware()) {
    RCLCPP_INFO(getLogger(), "Hardware reconnected!");
    return CallbackReturn::SUCCESS;  // Continue
  }
  
  // Can't recover
  return CallbackReturn::ERROR;  // Stay in error state
}
```

**Manual Recovery:**
```bash
# Check controller states
ros2 control list_controllers

# If in error state, try reactivating
ros2 control set_controller_state diff_drive_controller inactive
ros2 control set_controller_state diff_drive_controller active

# If hardware interface crashed, restart controller manager node
ros2 lifecycle set /controller_manager shutdown
# Then relaunch
```

---

## Performance Tuning

### Profiling the Update Loop

```cpp
// Add to hardware interface
#include <chrono>

return_type read(const Time&, const Duration&) {
  auto start = std::chrono::high_resolution_clock::now();
  
  // Your read code
  
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  
  // Log every second
  RCLCPP_INFO_THROTTLE(getLogger(), *clock_, 1000, 
                       "read() took %ld µs", duration.count());
  
  // Warn if too slow (target < 1ms for 50Hz)
  if (duration.count() > 1000) {
    RCLCPP_WARN(getLogger(), "read() SLOW: %ld µs", duration.count());
  }
}
```

### Optimizing Serial Communication

**Problem:** Serial reads can block for 10-100ms

**Solution 1: Non-blocking reads**
```cpp
// Set serial port to non-blocking mode
fcntl(teensy_fd_, F_SETFL, O_NONBLOCK);

std::string readFromSerial() {
  char buffer[1024];
  int bytes = read(teensy_fd_, buffer, sizeof(buffer));
  
  if (bytes > 0) {
    return std::string(buffer, bytes);
  }
  // No data available - return immediately
  return "";
}
```

**Solution 2: Separate read thread**
```cpp
class SigynSystem : public SystemInterface {
private:
  std::thread read_thread_;
  std::mutex data_mutex_;
  std::string latest_message_;
  std::atomic<bool> thread_active_{false};
  
  void readThreadLoop() {
    while (thread_active_) {
      std::string msg = blockingReadFromSerial();  // OK to block here
      
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_message_ = msg;
    }
  }

public:
  CallbackReturn on_activate(const State&) override {
    thread_active_ = true;
    read_thread_ = std::thread(&SigynSystem::readThreadLoop, this);
  }
  
  return_type read(const Time&, const Duration&) override {
    std::string msg;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      msg = latest_message_;  // Copy latest data - very fast
    }
    
    if (!msg.empty()) {
      parseEncoderMessage(msg);
    }
    
    return return_type::OK;
  }
};
```

### Reducing Controller Computation

**Problem:** Complex controller logic takes too long

**Solution: Simplify or offload**
```yaml
# Reduce update rate if possible
controller_manager:
  ros__parameters:
    update_rate: 30  # Instead of 50, if robot dynamics allow

# Or split controllers
# Fast controller: Critical safety (50 Hz)
# Slow controller: Advanced features (10 Hz)
```

### Memory Optimization

```cpp
// Pre-allocate everything in on_init()
CallbackReturn on_init(const HardwareInfo& info) override {
  // Allocate all vectors once
  hw_commands_.resize(info.joints.size(), 0.0);
  hw_positions_.resize(info.joints.size(), 0.0);
  hw_velocities_.resize(info.joints.size(), 0.0);
  
  // Pre-allocate message buffers
  serial_buffer_.reserve(1024);
  
  // No more allocations after this!
  return CallbackReturn::SUCCESS;
}

// Never do this in read()/write():
// ❌ std::vector<double> temp_data;  // Allocates memory!
// ❌ std::string message = "...";    // Allocates!
// ✅ Use pre-allocated member variables instead
```

---

## Summary

**Key Takeaways:**

1. **Real-Time:** Keep read()/write() fast (<1ms), defer slow operations
2. **Custom Controllers:** Create when standard controllers don't fit your needs
3. **Sensor Integration:** Choose pattern based on sensor type and usage
4. **Multi-Interface:** Support multiple control modes, switch at runtime
5. **Error Handling:** Graceful degradation, automatic recovery where possible
6. **Performance:** Profile, optimize, use non-blocking I/O

**For Sigyn specifically:**
- TeensyV2 already handles real-time well (67 Hz odometry)
- Hardware interface should be minimal: serial I/O + parsing
- Keep IMU/LIDAR separate from ros2_control
- Consider custom safety controller for VL53L0X obstacle detection
- Profile serial communication - likely bottleneck

**Next:** See `MIGRATION_GUIDE.md` for step-by-step Sigyn conversion.
