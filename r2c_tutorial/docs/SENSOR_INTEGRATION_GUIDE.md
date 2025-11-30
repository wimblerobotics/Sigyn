# Complete Sensor Integration with ros2_control

This document explains the complete architecture for integrating custom sensors (temperature, voltage, current, VL53L0X range) into ros2_control, including hardware interfaces, controllers, and client code.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         APPLICATION LAYER                            │
│  (Your safety systems, navigation, monitoring, etc.)                │
│                                                                       │
│  sensor_reader.py          safety_monitor.py       nav_system.cpp   │
│  └─ Subscribes to:         └─ Checks limits       └─ Uses range     │
│     sensor topics             Shuts down on          for obstacles  │
│                               thermal/voltage                        │
└───────────────────────────────┬─────────────────────────────────────┘
                                │
                    ROS Topics (sensor_msgs)
                                │
┌───────────────────────────────┴─────────────────────────────────────┐
│                        CONTROLLER LAYER                              │
│                                                                       │
│  ┌──────────────────────┐          ┌──────────────────────┐        │
│  │ SensorStateBroadcaster│         │ SensorCommandController│       │
│  │ (Read + Publish)      │         │ (Subscribe + Write)   │       │
│  │                       │         │ [SIMULATION ONLY]     │       │
│  │ Reads:                │         │ Subscribes:           │       │
│  │  - temperature/temp   │         │  - ~/temperature_cmd  │       │
│  │  - voltage/voltage    │         │  - ~/voltage_cmd      │       │
│  │  - current/current    │         │  - ~/current_cmd      │       │
│  │  - range/range        │         │  - ~/range_cmd        │       │
│  │                       │         │                        │       │
│  │ Publishes:            │         │ Writes:               │       │
│  │  - ~/temperature      │         │  - temperature/temp   │       │
│  │  - ~/battery          │         │  - voltage/voltage    │       │
│  │  - ~/range            │         │  - current/current    │       │
│  └───────┬───────────────┘         │  - range/range        │       │
│          │                         └───────┬────────────────┘       │
│          │                                 │                         │
│       Reads State                       Writes Command              │
│       Interfaces                        Interfaces                  │
└──────────┴────────────────────────────────┴─────────────────────────┘
           │                                 │
    State Interfaces                  Command Interfaces
    (Always available)           (Simulation mode only)
           │                                 │
┌──────────┴────────────────────────────────┴─────────────────────────┐
│                    HARDWARE INTERFACE LAYER                          │
│                                                                       │
│                   SensorHardwareInterface                            │
│                   (hardware_interface::SystemInterface)              │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ export_state_interfaces()                                    │   │
│  │  - temperature_sensor/temperature                            │   │
│  │  - voltage_sensor/voltage                                    │   │
│  │  - current_sensor/current                                    │   │
│  │  - vl53l0x_sensor/range                                      │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ export_command_interfaces() [if sim_mode]                    │   │
│  │  - temperature_sensor/temperature                            │   │
│  │  - voltage_sensor/voltage                                    │   │
│  │  - current_sensor/current                                    │   │
│  │  - vl53l0x_sensor/range                                      │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ read()                                                        │   │
│  │  - In real hardware: Updates state from topic callbacks      │   │
│  │  - In simulation: No-op (values come from write())           │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │ write()                                                       │   │
│  │  - In real hardware: No-op (read-only sensors)               │   │
│  │  - In simulation: Copies command values to state values      │   │
│  └─────────────────────────────────────────────────────────────┘   │
└───────────────────────────────┬───────────────────────────────────┘
                                │
                        ROS Topics (subscriptions)
                                │
┌───────────────────────────────┴─────────────────────────────────────┐
│                          DATA SOURCE                                 │
│                                                                       │
│  REAL HARDWARE:                  SIMULATION:                         │
│  sigyn_to_sensor_v2              sensor_simulator.py                 │
│  (Teensy → ROS)                  (Fake data generator)               │
│                                                                       │
│  Publishes:                      Publishes:                          │
│   - /sigyn/teensy_bridge/        - /sensors/temperature             │
│     battery_state                - /sensors/voltage                  │
│   - /sensors/temperature         - /sensors/current                  │
│   - /sensors/range               - /sensors/range                    │
└─────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Hardware Interface: `SensorHardwareInterface`

**Purpose**: Bridge between physical/simulated sensors and ros2_control

**Files**:
- `include/r2c_tutorial/sensor_hardware_interface.hpp`
- `src/sensor_hardware_interface.cpp`
- `sensor_hardware_interface_plugin.xml`

**Key Methods**:

```cpp
// Called once during initialization
CallbackReturn on_init(const HardwareInfo & info) {
  // Read initial values from URDF parameters
  // Set sim_mode flag from hardware parameters
}

// Export sensor state interfaces (always available)
std::vector<StateInterface> export_state_interfaces() {
  // temperature_sensor/temperature
  // voltage_sensor/voltage
  // current_sensor/current
  // vl53l0x_sensor/range
}

// Export command interfaces (simulation mode only)
std::vector<CommandInterface> export_command_interfaces() {
  if (sim_mode_) {
    // Same interfaces as state, but writable
  }
  return {};  // Empty in real hardware mode
}

// Called when hardware is activated
CallbackReturn on_activate(const State & previous_state) {
  // Create ROS node
  // Subscribe to sensor topics
  // Start executor thread
}

// Called periodically to read sensor data
return_type read(const Time & time, const Duration & period) {
  // In real hardware: Data arrives via topic callbacks
  // In simulation: No-op (data comes from write())
  return return_type::OK;
}

// Called periodically to write commands
return_type write(const Time & time, const Duration & period) {
  if (sim_mode_) {
    // Copy command interface values to state interface values
    temperature_ = temperature_cmd_;
    voltage_ = voltage_cmd_;
    // etc.
  }
  // In real hardware mode: No-op (read-only)
  return return_type::OK;
}
```

**Configuration (URDF)**:
```xml
<ros2_control name="sensor_system" type="system">
  <hardware>
    <plugin>r2c_tutorial/SensorHardwareInterface</plugin>
    <param name="sim_mode">true</param>  <!-- or false for real hardware -->
  </hardware>
  
  <sensor name="temperature_sensor">
    <state_interface name="temperature"/>
    <xacro:if value="$(arg sim_mode)">
      <command_interface name="temperature"/>
    </xacro:if>
    <param name="initial_value">25.0</param>
  </sensor>
  <!-- voltage, current, range sensors similarly -->
</ros2_control>
```

### 2. State Broadcaster: `SensorStateBroadcaster`

**Purpose**: Read sensor state interfaces and publish as ROS messages

**Files**:
- `include/r2c_tutorial/sensor_state_broadcaster.hpp`
- `src/sensor_state_broadcaster.cpp`
- `sensor_state_broadcaster_plugin.xml`

**Key Methods**:

```cpp
// Specify which state interfaces to read
InterfaceConfiguration state_interface_configuration() const {
  return {
    interface_configuration_type::INDIVIDUAL,
    {
      "temperature_sensor/temperature",
      "voltage_sensor/voltage",
      "current_sensor/current",
      "vl53l0x_sensor/range"
    }
  };
}

// Called periodically to publish sensor data
return_type update(const Time & time, const Duration & period) {
  // Read values from state interfaces
  double temp = state_interfaces_[0].get_value();
  double voltage = state_interfaces_[1].get_value();
  double current = state_interfaces_[2].get_value();
  double range = state_interfaces_[3].get_value();
  
  // Publish as ROS messages
  temperature_pub_->publish(temp_msg);
  battery_pub_->publish(battery_msg);
  range_pub_->publish(range_msg);
}
```

**Configuration (controllers.yaml)**:
```yaml
sensor_state_broadcaster:
  ros__parameters:
    temperature_sensor_name: "temperature_sensor"
    voltage_sensor_name: "voltage_sensor"
    current_sensor_name: "current_sensor"
    range_sensor_name: "vl53l0x_sensor"
    
    temperature_frame_id: "temperature_sensor"
    battery_frame_id: "base_link"
    range_frame_id: "vl53l0x_sensor"
    
    publish_rate: 10.0  # Hz
```

**Published Topics**:
- `/sensor_state_broadcaster/temperature` - `sensor_msgs/msg/Temperature`
- `/sensor_state_broadcaster/battery` - `sensor_msgs/msg/BatteryState`
- `/sensor_state_broadcaster/range` - `sensor_msgs/msg/Range`

### 3. Command Controller: `SensorCommandController` (Simulation Only)

**Purpose**: Subscribe to command topics and write to command interfaces

**Files**:
- `include/r2c_tutorial/sensor_command_controller.hpp`
- `src/sensor_command_controller.cpp`

**Key Methods**:

```cpp
// Specify which command interfaces to write
InterfaceConfiguration command_interface_configuration() const {
  return {
    interface_configuration_type::INDIVIDUAL,
    {
      "temperature_sensor/temperature",
      "voltage_sensor/voltage",
      "current_sensor/current",
      "vl53l0x_sensor/range"
    }
  };
}

// Called periodically to write commands
return_type update(const Time & time, const Duration & period) {
  if (temperature_cmd_received_) {
    command_interfaces_[0].set_value(temperature_cmd_);
  }
  // Similar for other sensors
}

// Callback when command received
void temperature_cmd_callback(const Float64::SharedPtr msg) {
  temperature_cmd_ = msg->data;
  temperature_cmd_received_ = true;
}
```

**Subscribed Topics**:
- `/sensor_command_controller/temperature_cmd` - `std_msgs/msg/Float64`
- `/sensor_command_controller/voltage_cmd` - `std_msgs/msg/Float64`
- `/sensor_command_controller/current_cmd` - `std_msgs/msg/Float64`
- `/sensor_command_controller/range_cmd` - `std_msgs/msg/Float64`

## Usage Examples

### Reading Sensor Data (Application Code)

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, BatteryState, Range

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Subscribe to sensor data from controller
        self.create_subscription(
            Temperature,
            '/sensor_state_broadcaster/temperature',
            self.check_temperature,
            10
        )
        
        self.create_subscription(
            BatteryState,
            '/sensor_state_broadcaster/battery',
            self.check_battery,
            10
        )
        
        self.create_subscription(
            Range,
            '/sensor_state_broadcaster/range',
            self.check_obstacles,
            10
        )
    
    def check_temperature(self, msg):
        if msg.temperature > 80.0:
            self.get_logger().error('THERMAL SHUTDOWN!')
            # Trigger emergency stop
    
    def check_battery(self, msg):
        if msg.voltage < 10.0:
            self.get_logger().warn('Low battery - return to dock')
    
    def check_obstacles(self, msg):
        if msg.range < 0.1:
            self.get_logger().warn('Obstacle detected!')
            # Slow down or stop
```

### Writing Commands (Simulation Testing)

```bash
# Test thermal protection
ros2 topic pub --once /sensor_command_controller/temperature_cmd \
  std_msgs/msg/Float64 "{data: 85.0}"

# Test low battery protection
ros2 topic pub --once /sensor_command_controller/voltage_cmd \
  std_msgs/msg/Float64 "{data: 9.5}"

# Test overcurrent protection
ros2 topic pub --once /sensor_command_controller/current_cmd \
  std_msgs/msg/Float64 "{data: 9.0}"

# Test collision avoidance
ros2 topic pub --once /sensor_command_controller/range_cmd \
  std_msgs/msg/Float64 "{data: 0.05}"
```

### Running the Complete System

```bash
# Terminal 1: Launch simulation with sensors
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# Terminal 2: Monitor sensor data
ros2 run r2c_tutorial sensor_reader.py

# Terminal 3: (Optional) Inject test commands
ros2 topic pub /sensor_command_controller/temperature_cmd \
  std_msgs/msg/Float64 "{data: 85.0}"
```

## Key Concepts

### 1. Hardware Interfaces vs Controllers

- **Hardware Interface**: Low-level driver that communicates with hardware (or simulation)
  - Exports state/command interfaces
  - Handles read/write cycles
  - One per physical hardware system

- **Controller**: High-level logic that uses interfaces
  - Reads state interfaces, writes command interfaces
  - Implements control algorithms
  - Publishes/subscribes to ROS topics
  - Multiple controllers can use same hardware

### 2. State vs Command Interfaces

- **State Interface**: Read-only data from hardware
  - Current temperature, voltage, position, etc.
  - Always available
  - Multiple controllers can read simultaneously

- **Command Interface**: Write commands to hardware
  - Target temperature, target position, etc.
  - Only one controller can claim at a time
  - Only available if hardware supports it (sim mode for sensors)

### 3. Simulation vs Real Hardware

**Simulation Mode** (`sim_mode: true`):
- Command interfaces exposed
- write() method copies commands to states
- Enables testing without physical hardware
- Can inject fault conditions safely

**Real Hardware Mode** (`sim_mode: false`):
- No command interfaces (read-only sensors)
- State values come from topic subscriptions
- write() is no-op
- Safe for production use

## Testing Safety Systems

The primary purpose of writable sensor interfaces in simulation is to test safety systems:

```bash
# 1. Start simulation
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# 2. In another terminal, start safety monitor
ros2 run r2c_tutorial sensor_reader.py

# 3. Inject high temperature
ros2 topic pub --once /sensor_command_controller/temperature_cmd \
  std_msgs/msg/Float64 "{data: 85.0}"
# → Should see: "⚠️  TEMPERATURE TOO HIGH! Thermal protection needed!"

# 4. Inject low voltage
ros2 topic pub --once /sensor_command_controller/voltage_cmd \
  std_msgs/msg/Float64 "{data: 9.5}"
# → Should see: "⚠️  Low battery warning!"

# 5. Inject high current
ros2 topic pub --once /sensor_command_controller/current_cmd \
  std_msgs/msg/Float64 "{data: 9.0}"
# → Should see: "⚠️  High current draw!"

# 6. Inject close obstacle
ros2 topic pub --once /sensor_command_controller/range_cmd \
  std_msgs/msg/Float64 "{data: 0.05}"
# → Should see: "⚠️  Obstacle detected nearby!"
```

## Next Steps

1. **Build the complete stack**: Build all components we've created
2. **Test in simulation**: Launch Gazebo and verify sensor data flow
3. **Load sensor controllers**: Spawn sensor_state_broadcaster
4. **Optionally load command controller**: For testing with injected values
5. **Create real safety systems**: Implement actual shutdown/protection logic
6. **Transition to real hardware**: Change sim_mode to false, use sigyn_to_sensor_v2

This architecture provides a clean, testable, and maintainable way to integrate custom sensors into your ROS2 control system!
