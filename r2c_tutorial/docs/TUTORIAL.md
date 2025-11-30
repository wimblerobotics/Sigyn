# ROS2 Control Tutorial - Complete Guide

## Table of Contents

1. [Overview](#overview)
2. [What is ros2_control?](#what-is-ros2_control)
3. [Quick Start](#quick-start)
4. [Phase 1: Understanding the Basics](#phase-1-understanding-the-basics)
5. [Phase 2: Exploring Controllers](#phase-2-exploring-controllers)
6. [Phase 3: Creating Custom Hardware Interfaces](#phase-3-creating-custom-hardware-interfaces)
7. [Phase 4: Integration with Sigyn](#phase-4-integration-with-sigyn)
8. [Troubleshooting](#troubleshooting)
9. [Resources](#resources)

---

## Overview

This tutorial package teaches you ros2_control through hands-on experience with a simple robot. You'll learn:

- ✅ How ros2_control architecture works
- ✅ How to configure controllers for different joint types
- ✅ How differential drive control works
- ✅ How to integrate sensors (like temperature sensors)
- ✅ How to create custom hardware interfaces
- ✅ How to migrate existing systems to ros2_control

**Learning Approach:** Start simple, build understanding, then tackle complex real-world integration with Sigyn.

---

## What is ros2_control?

### The Problem It Solves

When building robots, you face challenges:
- **Hardware Abstraction:** Need to swap between simulation and real hardware without changing control code
- **Controller Reusability:** Want to use standard controllers (diff_drive, joint_trajectory, etc.) without rewriting them
- **Safety & Real-time:** Need guarantees about update rates, command timeouts, emergency stops
- **Multi-hardware Integration:** Motors, servos, sensors all need unified interface

### The Solution: ros2_control Framework

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Application Layer                   │
│         (Navigation, MoveIt, your custom nodes)             │
└────────────────────┬────────────────────────────────────────┘
                     │ cmd_vel, joint commands, etc.
┌────────────────────▼────────────────────────────────────────┐
│                  ros2_control Layer                         │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Controller Manager                            │  │
│  │  ┌────────────────┐  ┌──────────────────────────┐   │  │
│  │  │ Diff Drive     │  │ Joint Trajectory         │   │  │
│  │  │ Controller     │  │ Controller               │   │  │
│  │  └────────┬───────┘  └────────┬─────────────────┘   │  │
│  └───────────┼──────────────────┼─────────────────────┘  │
│              │                  │                         │
│  ┌───────────▼──────────────────▼─────────────────────┐  │
│  │         Hardware Interface Layer                     │  │
│  │    (Your custom code: SigynSystem, etc.)            │  │
│  │  - read() : Get data from hardware                  │  │
│  │  - write(): Send commands to hardware               │  │
│  └───────────┬──────────────────────────────────────────┘  │
└──────────────┼─────────────────────────────────────────────┘
               │
┌──────────────▼─────────────────────────────────────────────┐
│              Actual Hardware                                │
│   (RoboClaw, Teensy, motors, sensors, etc.)                │
└────────────────────────────────────────────────────────────┘
```

### Key Concepts

1. **Hardware Interface**: Your code that talks to actual hardware (or simulation)
2. **Controller**: Standard or custom logic that converts high-level commands to joint commands
3. **Controller Manager**: Orchestrates controllers, manages lifecycle, enforces timing
4. **Command Interface**: What you can SEND to hardware (velocity, position, effort)
5. **State Interface**: What you can READ from hardware (position, velocity, temperature, etc.)

---

## Quick Start

### Build the Package

```bash
cd ~/sigyn_ws
colcon build --packages-select r2c_tutorial --symlink-install
source install/setup.bash
```

### Run the Simulation

```bash
# Terminal 1: Launch Gazebo with robot and controllers
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# Terminal 2: Run the Twist Stamper (Required for Jazzy/Rolling)
# This converts teleop Twist messages to TwistStamped for the controller
chmod +x src/Sigyn/r2c_tutorial/scripts/twist_stamper.py
python3 src/Sigyn/r2c_tutorial/scripts/twist_stamper.py --ros-args -r cmd_vel_in:=/cmd_vel -r cmd_vel_out:=/diff_drive_controller/cmd_vel

# Terminal 3: Drive the robot with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 4: Monitor robot state
ros2 run r2c_tutorial monitor_robot.py

# Terminal 5: Control the arm
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['arm_joint'], points: [{positions: [1.57], time_from_start: {sec: 2}}]}" \
  --once
```

### What You Should See

- **Foxglove Studio**: Connect to `ws://localhost:8765` to see robot model and topics
- **teleop_twist_keyboard**: Use WASD keys to drive
- **monitor_robot.py**: Real-time display of joint positions, velocities, temperature, odometry

### Note on TwistStamped

In newer ROS2 versions (Jazzy/Rolling), `diff_drive_controller` requires `TwistStamped` messages. The `teleop_twist_keyboard` node publishes `Twist` messages. We use the `twist_stamper.py` script to bridge this gap.

---

## Phase 1: Understanding the Basics

### Exercise 1.1: Explore the URDF Structure

**File:** `urdf/r2c_test.xacro`

Open this file and identify:

1. **Links**: Physical components (chassis, wheels, arm, sensor)
2. **Joints**: How links connect and move
   - `continuous` joints: Wheels (unlimited rotation)
   - `revolute` joints: Arm (limited rotation)
   - `fixed` joints: Caster, sensor mount
3. **Inertial properties**: Mass, inertia matrices (required for simulation)

**Action:** Modify the arm length from 0.25m to 0.30m
```bash
# In urdf/r2c_test.xacro, change:
<xacro:property name="arm_length" value="0.30"/>

# Rebuild and visualize:
ros2 launch r2c_tutorial manual_control.launch.py
```

**Key Learning:** URDF defines the robot's **physical structure**. ros2_control builds on this.

---

### Exercise 1.2: Understanding ros2_control Hardware Interface

**File:** `urdf/r2c_test.ros2_control.xacro`

This file defines the **control interface** for your robot. Key sections:

#### Wheel Joints (Velocity Control)
```xml
<joint name="left_wheel_joint">
  <command_interface name="velocity">  <!-- We SEND velocity commands -->
    <param name="min">-10.0</param>    <!-- rad/s limits -->
    <param name="max">10.0</param>
  </command_interface>
  <state_interface name="position"/>   <!-- We READ position -->
  <state_interface name="velocity"/>   <!-- We READ velocity -->
</joint>
```

**Why velocity control?** Differential drive robots control wheel speeds, not positions.

#### Arm Joint (Position Control)
```xml
<joint name="arm_joint">
  <command_interface name="position">  <!-- We SEND position commands -->
    <param name="min">-3.14159</param> <!-- -pi to +pi radians -->
    <param name="max">3.14159</param>
  </command_interface>
  <state_interface name="position"/>
  <state_interface name="velocity"/>
</joint>
```

**Why position control?** Arms typically move to desired positions, not continuous spinning.

#### Temperature Sensor (Read-Only)
```xml
<sensor name="temperature_sensor">
  <state_interface name="temperature"/>  <!-- Read-only, no command interface -->
  <param name="initial_value">25.0</param>
</sensor>
```

**Key Learning:** 
- **Command interfaces** = What you control
- **State interfaces** = What you measure
- Different joints need different control modes (velocity vs. position vs. effort)

---

### Exercise 1.3: Controller Configuration

**File:** `config/controllers.yaml`

This file configures the **controllers** that use your hardware interfaces.

#### Differential Drive Controller
```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    
    # Robot geometry - CRITICAL for accurate odometry!
    wheel_separation: 0.40   # Distance between wheels (m)
    wheel_radius: 0.05       # Wheel radius (m)
```

**What it does:**
1. Subscribes to `/cmd_vel` (linear + angular velocity)
2. Converts to left/right wheel velocities using **differential drive kinematics**
3. Publishes `/odom` topic with estimated position
4. Publishes `odom→base_link` TF transform

**The Math:**
```
Given cmd_vel (linear_x, angular_z):

left_wheel_velocity = (linear_x - angular_z * wheel_separation/2) / wheel_radius
right_wheel_velocity = (linear_x + angular_z * wheel_separation/2) / wheel_radius
```

**Action:** Change `wheel_separation` to 0.50 and observe how the robot turns differently.

```bash
# Edit config/controllers.yaml
wheel_separation: 0.50

# Restart simulation
ros2 launch r2c_tutorial gazebo_sim.launch.py
```

**Key Learning:** Controller parameters **must match your physical robot** or you'll have incorrect odometry!

---

### Exercise 1.4: Controller Lifecycle

Controllers in ros2_control have a lifecycle:

```
 unconfigured
      ↓
  configure()
      ↓
   inactive
      ↓
   activate()
      ↓
    active  ← This is when the controller runs
      ↓
  deactivate()
      ↓
   inactive
```

**Try it:**

```bash
# List active controllers
ros2 control list_controllers

# You should see:
# diff_drive_controller[diff_drive_controller/DiffDriveController] active
# arm_position_controller[joint_trajectory_controller/JointTrajectoryController] active
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active

# Deactivate diff drive controller
ros2 control set_controller_state diff_drive_controller inactive

# Try driving with teleop - nothing happens!

# Reactivate
ros2 control set_controller_state diff_drive_controller active

# Driving works again
```

**Key Learning:** Controllers can be dynamically loaded, started, stopped without restarting the robot.

---

### Exercise 1.5: Understanding Joint State Broadcaster

**Special controller:** `joint_state_broadcaster`

This doesn't control anything - it just **publishes joint states** to `/joint_states` topic.

**Why needed?**
- `robot_state_publisher` needs joint states to compute TF transforms
- RViz needs joint states to animate the robot model
- Your monitoring tools need to know joint positions

**Verify:**
```bash
# Echo joint states
ros2 topic echo /joint_states

# You'll see:
# name: [left_wheel_joint, right_wheel_joint, arm_joint]
# position: [0.0, 0.0, 0.0]
# velocity: [0.0, 0.0, 0.0]
```

**Key Learning:** Some "controllers" are really just data publishers.

---

## Phase 2: Exploring Controllers

### Exercise 2.1: Differential Drive Controller Deep Dive

The diff_drive_controller is doing **a lot** behind the scenes:

1. **Kinematics:** Converts cmd_vel → wheel velocities
2. **Odometry:** Integrates wheel encoder data → robot pose
3. **TF Publishing:** Broadcasts `odom→base_link` transform
4. **Safety:** Stops robot if cmd_vel times out

**Understanding Odometry:**

```python
# Pseudocode for odometry integration (runs at 50 Hz):
delta_left = (current_left_encoder - previous_left_encoder) * wheel_circumference / encoder_resolution
delta_right = (current_right_encoder - previous_right_encoder) * wheel_circumference / encoder_resolution

delta_distance = (delta_left + delta_right) / 2
delta_angle = (delta_right - delta_left) / wheel_separation

# Update robot pose
x += delta_distance * cos(theta + delta_angle/2)
y += delta_distance * sin(theta + delta_angle/2)
theta += delta_angle
```

**Test odometry accuracy:**

```bash
# Drive forward 1 meter
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}}" --rate 10

# Watch odometry
ros2 topic echo /diff_drive_controller/odom

# After ~10 seconds, stop (Ctrl+C the pub command)
# Check if reported position ≈ 1.0 meter
```

**Expected Result:** Should be close to x=1.0, but slight error due to wheel slippage in simulation.

---

### Exercise 2.2: Arm Position Controller

The `arm_position_controller` is a `JointTrajectoryController` - much more sophisticated than simple position control.

**Features:**
- **Trajectory following:** Smooth motion from current → goal position
- **Time constraints:** Reach goal by specified time
- **Velocity/acceleration limits:** Enforced in software
- **Goal tolerance:** Success threshold

**Simple Position Command:**

```bash
# Move arm to 90 degrees (1.57 radians) in 2 seconds
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{
    joint_names: ['arm_joint'],
    points: [
      {positions: [1.57], time_from_start: {sec: 2}}
    ]
  }" --once

# Move to -90 degrees
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{
    joint_names: ['arm_joint'],
    points: [
      {positions: [-1.57], time_from_start: {sec: 2}}
    ]
  }" --once
```

**Multi-Point Trajectory:**

```bash
# Move through multiple waypoints
ros2 topic pub /arm_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{
    joint_names: ['arm_joint'],
    points: [
      {positions: [0.5], velocities: [0.0], time_from_start: {sec: 1}},
      {positions: [1.0], velocities: [0.0], time_from_start: {sec: 2}},
      {positions: [0.0], velocities: [0.0], time_from_start: {sec: 3}}
    ]
  }" --once
```

**Key Learning:** Trajectory controllers provide **smooth, time-coordinated motion** vs. raw position commands.

---

### Exercise 2.3: Controller Switching

You can have multiple controllers for the same hardware, but only one active at a time.

**Scenario:** Switch between trajectory control and direct control.

```bash
# Currently: arm_position_controller is active

# Load forward_position_controller (already configured)
ros2 control load_controller forward_position_controller

# Switch controllers
ros2 control switch_controllers \
  --deactivate arm_position_controller \
  --activate forward_position_controller

# Now you can send direct position commands
ros2 topic pub /forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.0, 1.57]}" --rate 10
# This sends: [left_wheel, right_wheel, arm]

# Switch back
ros2 control switch_controllers \
  --deactivate forward_position_controller \
  --activate arm_position_controller
```

**Use Case:** Testing, debugging, or different operation modes.

**Key Learning:** Controller hot-swapping allows flexible robot behavior.

---

### Exercise 2.4: Temperature Sensor Integration

Unlike joints, the temperature sensor is **read-only**. No controller controls it.

**In Simulation:**
The launch file starts a simple temperature sensor simulator. It publishes
`sensor_msgs/msg/Temperature` messages on `/temperature_sensor/raw` so you can
work with the data just like a real sensor.

**Monitoring:**

```bash
# View temperature data (Celsius)
ros2 topic echo /temperature_sensor/raw

# Monitor script displays temperature alongside joints/odom
ros2 run r2c_tutorial monitor_robot.py

# Change the temperature target (affects simulator response)
ros2 run r2c_tutorial temperature_heater.py --temperature 45.0
```

**Note:** The temperature sensor in this tutorial is primarily for demonstrating **read-only sensor interfaces**. In real hardware, you'd:

1. Define sensor in ros2_control URDF
2. Read sensor in your hardware interface's `read()` method
3. Publish to sensor-specific topics or expose via state interfaces

**For Sigyn:** Your VL53L0X distance sensors and temperature sensors would work similarly.

---

## Phase 3: Creating Custom Hardware Interfaces

Now we get to the **real power** of ros2_control: creating custom hardware interfaces.

### Understanding Hardware Interface Structure

Every hardware interface must implement:

```cpp
class MyHardwareInterface : public hardware_interface::SystemInterface {
  // Lifecycle callbacks
  CallbackReturn on_init(const HardwareInfo & info);
  CallbackReturn on_configure(const State & previous_state);
  CallbackReturn on_activate(const State & previous_state);
  CallbackReturn on_deactivate(const State & previous_state);
  
  // Runtime methods (called at update_rate frequency)
  return_type read(const Time & time, const Duration & period);
  return_type write(const Time & time, const Duration & period);
  
  // Interface export
  std::vector<StateInterface> export_state_interfaces();
  std::vector<CommandInterface> export_command_interfaces();
};
```

### The Read-Write Loop

```
Controller Manager (50 Hz):
  ┌──────────────────────────────────────┐
  │  Loop forever:                       │
  │    1. read() from hardware           │ ← Get encoder counts, sensor data
  │    2. Update controller states       │
  │    3. Run all active controllers     │ ← diff_drive_controller, etc.
  │    4. write() to hardware            │ ← Send motor commands
  │    5. Sleep until next cycle         │
  └──────────────────────────────────────┘
```

**Timing is critical:**
- If `read()` is slow: Controllers get stale data
- If `write()` is slow: Motors respond late
- Target: Each cycle should complete in < 20ms for 50 Hz

---

### Exercise 3.1: Examine Gazebo Sim Hardware Interface

When `sim_mode:=true`, we use `gz_ros2_control/GazeboSimSystem`.

This is a **provided** hardware interface that:
- `read()`: Gets joint positions/velocities from Gazebo physics engine
- `write()`: Sends commands to Gazebo joint controllers

**File:** `/opt/ros/jazzy/share/gz_ros2_control/` (if installed)

**Key Point:** You don't write this - Gazebo provides it. But you **will** write the equivalent for your real hardware.

---

### Exercise 3.2: Create a Simple Mock Hardware Interface

Let's create a minimal hardware interface to understand the structure.

**Create:** `src/mock_hardware.cpp` (educational example, not for actual use)

```cpp
#include "hardware_interface/system_interface.hpp"

namespace r2c_tutorial {

class MockHardware : public hardware_interface::SystemInterface {
private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

public:
  CallbackReturn on_init(const HardwareInfo & info) override {
    // Called once at startup
    // Allocate storage for each joint
    hw_commands_.resize(info.joints.size(), 0.0);
    hw_states_position_.resize(info.joints.size(), 0.0);
    hw_states_velocity_.resize(info.joints.size(), 0.0);
    
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const State &) override {
    // Called when controllers are activated
    // Initialize hardware, open serial ports, etc.
    RCLCPP_INFO(rclcpp::get_logger("MockHardware"), "Activating...");
    return CallbackReturn::SUCCESS;
  }

  return_type read(const rclcpp::Time &, const rclcpp::Duration & period) override {
    // Called at update_rate (50 Hz)
    // READ from hardware: encoders, sensors, etc.
    
    // For mock: just integrate commanded velocity
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_states_velocity_[i] = hw_commands_[i];  // Assume perfect tracking
      hw_states_position_[i] += hw_commands_[i] * period.seconds();
    }
    
    return return_type::OK;
  }

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Called at update_rate (50 Hz)
    // WRITE to hardware: motor commands, actuator positions, etc.
    
    // For mock: nothing to do (we're not controlling real hardware)
    
    return return_type::OK;
  }

  std::vector<StateInterface> export_state_interfaces() override {
    std::vector<StateInterface> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
      state_interfaces.emplace_back(
        info_.joints[i].name, "position", &hw_states_position_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_states_velocity_[i]);
    }
    
    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override {
    std::vector<CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
      command_interfaces.emplace_back(
        info_.joints[i].name, "velocity", &hw_commands_[i]);
    }
    
    return command_interfaces;
  }
};

}  // namespace r2c_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(r2c_tutorial::MockHardware, hardware_interface::SystemInterface)
```

**Key Concepts:**

1. **Storage:** `hw_commands_`, `hw_states_*` - These hold the actual data
2. **Interfaces:** `export_*_interfaces()` gives pointers to controllers
3. **Read/Write:** Where you interact with hardware
4. **Plugin:** `PLUGINLIB_EXPORT_CLASS` makes it loadable

---

### Exercise 3.3: Understanding Sigyn's Hardware Interface

**File:** `sigyn_hardware_interface/src/sigyn_system.cpp`

This is your **real** hardware interface for Sigyn. Key sections:

#### Initialization
```cpp
CallbackReturn SigynSystem::on_init(const HardwareInfo & info) {
  // Parse parameters from URDF
  config_.teensy_port = info_.hardware_parameters["port"];
  config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  config_.wheel_diameter = std::stod(info_.hardware_parameters["wheel_diameter"]);
  // ...
  
  // Allocate storage
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
}
```

#### Reading from Teensy
```cpp
return_type SigynSystem::read(const Time &, const Duration &) {
  // 1. Read from serial port (Teensy)
  std::string message = readFromSerial();
  
  // 2. Parse encoder data
  parseEncoderMessage(message);
  
  // 3. Convert encoder counts to wheel positions/velocities
  encoderPositionsToWheelPositions();
  
  return return_type::OK;
}
```

#### Writing to Teensy
```cpp
return_type SigynSystem::write(const Time &, const Duration &) {
  // 1. Convert wheel velocities to motor speeds (QPPS)
  int32_t left_qpps, right_qpps;
  wheelVelocitiesToMotorSpeeds(hw_commands_[0], hw_commands_[1], 
                                left_qpps, right_qpps);
  
  // 2. Send command to Teensy over serial
  sendVelocityCommand(left_qpps, right_qpps);
  
  return return_type::OK;
}
```

**Key Differences from Mock:**
- Opens real serial port (`/dev/teensy_sensor`)
- Parses JSON messages from TeensyV2
- Converts units (rad/s ↔ QPPS)
- Handles timeouts and errors

---

## Phase 4: Integration with Sigyn

Now let's plan how to convert Sigyn to fully use ros2_control.

### Current Sigyn Architecture

```
TeensyV2 (roboclaw_monitor.cpp)
  ├─ Reads encoders from RoboClaw
  ├─ Calculates odometry
  ├─ Publishes /wheel_odom
  └─ Subscribes to /cmd_vel
       └─ Sends motor commands to RoboClaw

ROS2 Side
  ├─ teensy_bridge: Forwards /cmd_vel
  ├─ EKF: Fuses /wheel_odom + IMU
  └─ Nav2: Uses EKF odometry
```

### Target ros2_control Architecture

```
TeensyV2 (roboclaw_monitor.cpp)
  ├─ Reads encoders from RoboClaw
  └─ Reports via serial (EXISTING - just needs JSON format)

sigyn_hardware_interface (NEW)
  ├─ read(): Parses Teensy encoder data
  ├─ write(): Sends motor commands to Teensy
  └─ Exposes standard ros2_control interfaces

ros2_control Layer
  ├─ diff_drive_controller
  │   ├─ Subscribes to /cmd_vel
  │   ├─ Publishes /diff_drive_controller/odom
  │   └─ Publishes odom→base_link TF
  └─ joint_state_broadcaster

Nav2
  └─ Uses /diff_drive_controller/odom (or EKF fusion)
```

**Benefits:**
- ✅ Standard interface for Nav2
- ✅ Built-in safety (cmd_vel timeout)
- ✅ Cleaner architecture
- ✅ Easy to add more controllers later

---

### Migration Steps for Sigyn

#### Step 1: Verify Current System Works

```bash
# Test current Sigyn with existing code
ros2 launch base sigyn.launch.py
```

Make sure everything works before changes.

#### Step 2: Minimal Teensy Changes

The TeensyV2 already publishes encoder data. Just ensure format is parseable:

**Current:** `roboclaw_monitor.cpp` publishes to `serial_manager`
**Needed:** Ensure JSON format includes:
- `left_encoder`: int32
- `right_encoder`: int32
- `left_velocity_qpps`: int32 (optional, can calculate)
- `right_velocity_qpps`: int32 (optional)

**Action:** Review `TeensyV2/modules/roboclaw/roboclaw_monitor.cpp` - this already exists!

#### Step 3: Test Hardware Interface in Isolation

```bash
# Use fake hardware mode first
ros2 launch sigyn_hardware_interface test_hardware.launch.py

# This should:
# 1. Load sigyn_system hardware interface
# 2. Spawn diff_drive_controller
# 3. Allow you to test without Teensy connected
```

Create `test_hardware.launch.py` (similar to this tutorial's launch files).

#### Step 4: Test with Real Teensy

```bash
# With Teensy connected
ros2 launch sigyn_hardware_interface sigyn_control.launch.py

# Verify:
ros2 topic echo /joint_states  # Should show wheel positions
ros2 topic echo /diff_drive_controller/odom  # Should show odometry

# Test driving:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Step 5: Integrate with Navigation

Update `base/launch/sigyn.launch.py`:

```python
# OLD:
# teensy_bridge launches and publishes /wheel_odom

# NEW:
# Include sigyn_hardware_interface launch
# diff_drive_controller publishes /diff_drive_controller/odom

# Update EKF to use /diff_drive_controller/odom instead of /wheel_odom
```

#### Step 6: Add Sensors to ros2_control

**VL53L0X Distance Sensors:**

```xml
<!-- In ros2_control.xacro -->
<sensor name="vl53l0x_front">
  <state_interface name="range"/>  <!-- meters -->
  <param name="frame_id">vl53l0x_front</param>
</sensor>
```

**Temperature Sensors:**

```xml
<sensor name="temp_sensor_board">
  <state_interface name="temperature"/>  <!-- Celsius -->
</sensor>
```

**In Hardware Interface:**

```cpp
// In read():
// Parse sensor data from Teensy
double range_front = parseRangeSensor(message, "vl53l0x_front");
double temp_board = parseTemperature(message, "temp_board");

// Update state interfaces (already exported in export_state_interfaces)
```

**Publish as ROS topics:**

```cpp
// Create publishers in on_activate():
range_pub_ = node_->create_publisher<sensor_msgs::msg::Range>("/vl53l0x_front", 10);

// In read():
sensor_msgs::msg::Range range_msg;
range_msg.range = range_front;
range_pub_->publish(range_msg);
```

---

### Exercise 4.1: Plan Your Sensor Integration

**For Sigyn, you have:**
- 2x IMUs (already integrated separately)
- 2x LIDARs (already integrated separately)
- Nx VL53L0X distance sensors
- Nx Temperature sensors
- Gripper servo
- Elevator servo

**Decision Matrix:**

| Sensor | Integrate via ros2_control? | Why? |
|--------|---------------------------|------|
| IMUs | No | Complex fusion, separate node better |
| LIDARs | No | Already have drivers, Nav2 uses directly |
| VL53L0X | **Yes** | Simple sensors, Teensy provides data |
| Temperature | **Yes** | Simple sensors, Teensy provides data |
| Gripper | **Yes** | Actuator, benefits from trajectory control |
| Elevator | **Yes** | Actuator, benefits from trajectory control |
| Wheels | **Yes** | Core of ros2_control integration |

**Guideline:** 
- Simple sensors that Teensy already reads → ros2_control
- Complex sensors with dedicated drivers → Separate nodes
- All actuators → ros2_control

---

### Exercise 4.2: Create Migration Checklist

Create a file `sigyn_hardware_interface/docs/migration_checklist.md`:

```markdown
# Sigyn ros2_control Migration Checklist

## Phase 1: Preparation
- [ ] Document current system behavior (odometry accuracy, response time)
- [ ] Create test scenarios for validation
- [ ] Backup current working code (git branch)

## Phase 2: Hardware Interface Development
- [ ] Implement SigynSystem::read() for encoder parsing
- [ ] Implement SigynSystem::write() for motor commands
- [ ] Test serial communication in isolation
- [ ] Verify encoder data parsing
- [ ] Verify motor command formatting

## Phase 3: Controller Integration
- [ ] Configure diff_drive_controller with correct parameters
- [ ] Test odometry accuracy vs. old system
- [ ] Tune controller parameters if needed
- [ ] Verify cmd_vel timeout safety feature

## Phase 4: Sensor Addition
- [ ] Add VL53L0X sensors to hardware interface
- [ ] Test sensor reading and publishing
- [ ] Add temperature sensors
- [ ] Verify sensor data quality

## Phase 5: Actuator Addition
- [ ] Add gripper joint to ros2_control
- [ ] Configure joint_trajectory_controller for gripper
- [ ] Test gripper motion
- [ ] Add elevator joint
- [ ] Test elevator motion

## Phase 6: Integration Testing
- [ ] Test with Nav2 stack
- [ ] Compare localization accuracy to old system
- [ ] Test recovery behaviors
- [ ] Stress test (long duration runs)

## Phase 7: Cleanup
- [ ] Remove old teensy_bridge code (if fully replaced)
- [ ] Update documentation
- [ ] Update launch files
- [ ] Tag release
```

---

## Troubleshooting

### Common Issues

#### 1. "No controller manager node found"

**Symptom:** Controllers don't start, no joint_states published

**Solution:**
```bash
# Check if controller_manager is running
ros2 node list | grep controller_manager

# If not, check your launch file includes ros2_control plugin
# In Gazebo mode: Check gazebo.xacro has gz_ros2_control plugin
```

#### 2. "Controller failed to load"

**Symptom:** Error messages about missing controller

**Solution:**
```bash
# List available controller types
ros2 pkg list | grep controller

# Install missing controllers
sudo apt install ros-jazzy-diff-drive-controller
sudo apt install ros-jazzy-joint-trajectory-controller
```

#### 3. Robot drifts in circles

**Symptom:** When driving straight, robot curves

**Causes:**
- Incorrect `wheel_separation` parameter
- Wheel radius mismatch between left/right
- Encoder direction reversed

**Solution:**
```yaml
# In controllers.yaml, tune:
wheel_separation_multiplier: 1.02  # Increase if turning too much
left_wheel_radius_multiplier: 0.98  # Adjust individual wheels
right_wheel_radius_multiplier: 1.02
```

#### 4. Odometry doesn't match reality

**Symptom:** Robot thinks it moved 1m but actually moved 0.8m

**Causes:**
- Wrong `wheel_radius`
- Wrong `pulses_per_revolution` (encoder resolution)
- Wheel slippage

**Solution:**
```bash
# Calibration procedure:
# 1. Command robot to drive 1.0 meter
# 2. Physically measure actual distance
# 3. Calculate correction factor:
#    correction = actual_distance / reported_distance
# 4. Apply to wheel_radius:
#    new_radius = old_radius * correction
```

#### 5. High CPU usage / missed controller cycles

**Symptom:** Jerky motion, "Controller update loop took too long" warnings

**Causes:**
- `read()` or `write()` too slow
- Update rate too high
- Serial communication blocking

**Solution:**
```cpp
// In hardware interface:
// - Use non-blocking serial I/O
// - Cache data, don't compute in read()/write()
// - Profile with timing measurements:

auto start = std::chrono::high_resolution_clock::now();
// Your code here
auto end = std::chrono::high_resolution_clock::now();
auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
RCLCPP_INFO_THROTTLE(logger, clock, 1000, "read() took %ld us", duration.count());
// Target: < 1000 us (1 ms) for 50 Hz update rate
```

---

## Resources

### Official Documentation
- [ros2_control Overview](https://control.ros.org/)
- [Controller Types](https://github.com/ros-controls/ros2_controllers)
- [Hardware Interface Guide](https://control.ros.org/master/doc/getting_started/getting_started.html)

### Example Implementations
- [DiffBot Tutorial](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2)
- [Articulated Robots](https://control.ros.org/master/doc/ros2_control_demos/doc/index.html)

### Sigyn-Specific
- `TeensyV2/modules/roboclaw/` - Current motor control code
- `sigyn_hardware_interface/` - ros2_control implementation
- `base/config/navigation_sim.yaml` - Controller parameters

### Comparison: Sigyn vs. Typical Systems

**What makes Sigyn special:**
- ✅ RoboClaw closed-loop control (most hobbyist robots use open-loop)
- ✅ Teensy 4.1 high-speed processing
- ✅ 67 Hz odometry (vs. typical 20-30 Hz)
- ✅ Multiple sensor fusion

**How ros2_control helps Sigyn:**
- ✅ Standardized interface to Nav2
- ✅ Built-in safety features
- ✅ Easy to add new actuators (gripper, elevator already planned)
- ✅ Testing with fake hardware before deploying
- ✅ Community controllers (no need to write diff_drive from scratch)

---

## Summary

You've learned:

1. **Architecture:** ros2_control layers (hardware interface, controller, controller manager)
2. **Configuration:** URDF ros2_control tags, controller YAML files
3. **Controllers:** diff_drive, joint_trajectory, their parameters and tuning
4. **Hardware Interfaces:** Structure, read/write loops, custom implementation
5. **Migration Strategy:** How to move Sigyn from custom control to ros2_control

**Next Steps:**

1. **Practice with this tutorial robot**
   - Drive it around in Gazebo
   - Change parameters and observe effects
   - Try creating a simple trajectory for the arm

2. **Study sigyn_hardware_interface code**
   - Compare to this tutorial's simplified approach
   - Understand serial communication with Teensy
   - Review encoder conversion math

3. **Plan Sigyn migration**
   - Use the checklist in Exercise 4.2
   - Start with simulation (fake hardware)
   - Gradually migrate real hardware

4. **Experiment with sensors**
   - Add a new sensor to this tutorial robot
   - Implement reading it in the hardware interface
   - Publish the data to a ROS topic

**You're now ready to confidently work with ros2_control and migrate Sigyn!**

---

**Questions?** Review the troubleshooting section or check the official ros2_control documentation.

**Found issues?** Check your build with:
```bash
colcon build --packages-select r2c_tutorial --symlink-install
source install/setup.bash
ros2 launch r2c_tutorial gazebo_sim.launch.py
```
