# Complete Sensor Integration Reference

This document provides a comprehensive reference for the sensor integration system, including architecture, implementation details, and usage examples for testing safety systems.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Component Details](#component-details)
3. [Data Flow](#data-flow)
4. [Testing Your Safety Systems](#testing-your-safety-systems)
5. [Python Examples](#python-examples)
6. [C++ Examples](#cpp-examples)
7. [Configuration Reference](#configuration-reference)
8. [Troubleshooting](#troubleshooting)

## Architecture Overview

### System Components

```
┌────────────────────────────────────────────────────────────────┐
│                     Your Safety System                          │
│         (Subscribes to sensor data, makes decisions)            │
└─────────────────────────┬──────────────────────────────────────┘
                          │
            ┌─────────────▼──────────────┐
            │ Sensor Data Topics          │
            │ (Standard ROS messages)     │
            └─────────────┬──────────────┘
                          │
┌─────────────────────────▼──────────────────────────────────────┐
│              SensorStateBroadcaster                             │
│  - Reads state interfaces from hardware                         │
│  - Publishes sensor_msgs/Temperature, BatteryState, Range       │
│  - Update rate: 10 Hz (configurable)                            │
└─────────────────────────┬──────────────────────────────────────┘
                          │ state_interfaces
                          │
┌─────────────────────────▼──────────────────────────────────────┐
│         Hardware Interface (Dual Implementation)                │
│                                                                  │
│  REAL HARDWARE (sim_mode:=false)                                │
│  ├─ SensorHardwareInterface                                     │
│  ├─ Subscribes to Teensy topics                                 │
│  └─ READ ONLY - no command interfaces                           │
│                                                                  │
│  SIMULATION (sim_mode:=true)                                    │
│  ├─ SensorHardwareInterfaceSim                                  │
│  ├─ Generates synthetic sensor data                             │
│  ├─ Exposes command interfaces for testing                      │
│  └─ read() uses commands if non-zero, else simulation           │
└─────────────────────────┬──────────────────────────────────────┘
                          │ command_interfaces (sim only)
                          │
┌─────────────────────────▼──────────────────────────────────────┐
│         SensorCommandController (Simulation Only)               │
│  - Subscribes to Float64 command topics                         │
│  - Writes to command interfaces                                 │
│  - Enables test value injection                                 │
└─────────────────────────┬──────────────────────────────────────┘
                          │
            ┌─────────────▼──────────────┐
            │ Command Topics (Float64)    │
            │ - temperature_cmd           │
            │ - voltage_cmd               │
            │ - current_cmd               │
            │ - range_cmd                 │
            └─────────────┬──────────────┘
                          │
           ┌──────────────▼─────────────────┐
           │ Test Scripts / Your Test Code  │
           │ - sensor_writer.py             │
           │ - ros2 topic pub               │
           │ - Custom Python/C++ test code  │
           └────────────────────────────────┘
```

### Key Design Decisions

1. **Dual Hardware Interface Pattern**
   - Same controllers work with both sim and real hardware
   - Hardware interface layer handles sim vs real differences
   - Enables testing safety logic in simulation

2. **Command Interfaces for Testing**
   - Only available in simulation mode
   - Allows injecting specific sensor values
   - Non-zero command overrides simulation
   - Zero command returns to normal simulation

3. **Standard ROS Messages**
   - sensor_msgs/Temperature for temperature data
   - sensor_msgs/BatteryState for voltage/current
   - sensor_msgs/Range for distance measurements
   - Ensures compatibility with existing ROS tools

## Component Details

### 1. SensorHardwareInterfaceSim (Simulation)

**File:** `src/sensor_hardware_interface_sim.cpp`  
**Header:** `include/r2c_tutorial/sensor_hardware_interface_sim.hpp`

**Purpose:** Provides simulated sensor data for Gazebo environments with test value injection capability.

**State Interfaces (read by controllers):**
```cpp
temperature_sensor/temperature  // °C
battery_sensor/voltage          // V
battery_sensor/current          // A
range_sensor/range              // m
```

**Command Interfaces (written by test controller):**
```cpp
temperature_sensor/temperature  // Test injection
battery_sensor/voltage          // Test injection
battery_sensor/current          // Test injection
range_sensor/range              // Test injection
```

**Simulation Logic:**
```cpp
// Normal simulation (when commands == 0.0)
temperature = 25.0 + 5.0 * sin(t * 0.1) + 2.0 * sin(t * 0.05);
voltage = 12.0 - 0.0001 * t + 0.05 * sin(t * 0.3);
current = 2.0 + 1.0 * sin(t * 0.15) + 0.5 * sin(t * 0.08);
range = 1.0 + 0.5 * sin(t * 0.2);

// Test mode (when any command != 0.0)
if (temperature_cmd != 0.0) temperature = temperature_cmd;
if (voltage_cmd != 0.0) voltage = voltage_cmd;
// ... etc
```

**Key Methods:**
- `initSim()` - Initialize for Gazebo integration
- `on_init()` - Standard hardware interface initialization
- `on_configure()` - Configuration phase
- `on_activate()` - Activation phase
- `read()` - Generate or use commanded sensor values
- `write()` - Store commanded test values (no-op, commands processed in read())
- `export_state_interfaces()` - Create state interfaces
- `export_command_interfaces()` - Create command interfaces

### 2. SensorHardwareInterface (Real Hardware)

**File:** `src/sensor_hardware_interface.cpp`  
**Header:** `include/r2c_tutorial/sensor_hardware_interface.hpp`

**Purpose:** Interfaces with real Teensy hardware via ROS topics.

**State Interfaces:**
Same as simulation (read-only)

**Command Interfaces:**
None - real sensors are read-only

**Data Source:**
Subscribes to Teensy topics:
```
/teensy/sensors/temperature
/teensy/sensors/voltage
/teensy/sensors/current
/teensy/sensors/range
```

### 3. SensorStateBroadcaster (Controller)

**File:** `src/sensor_state_broadcaster.cpp`  
**Header:** `include/r2c_tutorial/sensor_state_broadcaster.hpp`

**Purpose:** Reads sensor state interfaces and publishes standard ROS messages.

**Claimed Interfaces:**
```cpp
temperature_sensor/temperature  // Read
battery_sensor/voltage          // Read
battery_sensor/current          // Read
range_sensor/range              // Read
```

**Published Topics:**
```
~/temperature (sensor_msgs/msg/Temperature)
~/battery (sensor_msgs/msg/BatteryState)
~/range (sensor_msgs/msg/Range)
```

**Configuration Parameters:**
```yaml
sensor_state_broadcaster:
  ros__parameters:
    temperature_sensor_name: "temperature_sensor"
    battery_sensor_name: "battery_sensor"
    range_sensor_name: "range_sensor"
    publish_rate: 10.0  # Hz
    temperature_frame_id: "temperature_link"
    battery_frame_id: "base_link"
    range_frame_id: "range_link"
```

**Update Loop:**
```cpp
// Called at controller_manager update rate (50 Hz typical)
update(time, period) {
  // Read state interfaces
  double temp = state_interfaces_[0].get_value();
  double voltage = state_interfaces_[1].get_value();
  // ...
  
  // Publish at configured rate (10 Hz default)
  if (time - last_publish_time_ >= publish_period) {
    temperature_msg.temperature = temp;
    temperature_pub_->publish(temperature_msg);
    // ... publish other messages
  }
}
```

### 4. SensorCommandController (Simulation Only)

**File:** `src/sensor_command_controller.cpp`  
**Header:** `include/r2c_tutorial/sensor_command_controller.hpp`

**Purpose:** Subscribes to command topics and writes test values to command interfaces.

**Claimed Interfaces:**
```cpp
temperature_sensor/temperature  // Write
battery_sensor/voltage          // Write
battery_sensor/current          // Write
range_sensor/range              // Write
```

**Subscribed Topics:**
```
~/temperature_cmd (std_msgs/msg/Float64)
~/voltage_cmd (std_msgs/msg/Float64)
~/current_cmd (std_msgs/msg/Float64)
~/range_cmd (std_msgs/msg/Float64)
```

**Update Loop:**
```cpp
update(time, period) {
  // Write commanded values to hardware command interfaces
  if (temperature_cmd_received_) {
    command_interfaces_[0].set_value(temperature_cmd_);
  }
  // ... write other commands
  
  return controller_interface::return_type::OK;
}
```

**Callback Example:**
```cpp
void temperature_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  temperature_cmd_ = msg->data;
  temperature_cmd_received_ = true;
}
```

## Data Flow

### Normal Operation (Simulation)

1. **Hardware Interface read()** generates synthetic sensor data
2. **State interfaces** hold sensor values
3. **SensorStateBroadcaster** reads state interfaces at 50 Hz
4. **SensorStateBroadcaster** publishes messages at 10 Hz
5. **Your safety system** receives and processes sensor data

### Test Mode (Simulation with Command Injection)

1. **Your test script** publishes to `/sensor_command_controller/temperature_cmd`
2. **SensorCommandController** receives command in callback
3. **SensorCommandController update()** writes to command interface
4. **Hardware Interface read()** detects non-zero command, uses it instead of simulation
5. **State interface** now holds injected test value
6. **SensorStateBroadcaster** reads and publishes test value
7. **Your safety system** receives and reacts to fault condition

### Real Hardware Mode

1. **Teensy** publishes to `/teensy/sensors/*` topics
2. **Hardware Interface** callbacks receive Teensy data
3. **Hardware Interface read()** copies latest values to state interfaces
4. **SensorStateBroadcaster** reads and republishes in standard format
5. **Your safety system** receives sensor data (same as simulation!)

## Testing Your Safety Systems

### Quick Reference

```bash
# Start simulation
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# Monitor sensors
ros2 run r2c_tutorial sensor_reader.py

# Inject test temperature
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0

# Or use direct topic publish
ros2 topic pub /sensor_command_controller/temperature_cmd std_msgs/msg/Float64 "{data: 93.0}" --once

# Reset to simulation
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=0.0
```

See **[SENSOR_TESTING_GUIDE.md](SENSOR_TESTING_GUIDE.md)** for complete testing documentation.

## Python Examples

### Simple Monitor

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, BatteryState, Range

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        self.create_subscription(
            Temperature, '/sensor_state_broadcaster/temperature',
            self.temp_callback, 10)
        self.create_subscription(
            BatteryState, '/sensor_state_broadcaster/battery',
            self.battery_callback, 10)
        self.create_subscription(
            Range, '/sensor_state_broadcaster/range',
            self.range_callback, 10)
    
    def temp_callback(self, msg):
        if msg.temperature > 80.0:
            self.get_logger().error(f'OVERHEAT: {msg.temperature}°C')
    
    def battery_callback(self, msg):
        if msg.voltage < 10.0:
            self.get_logger().error(f'LOW BATTERY: {msg.voltage}V')
    
    def range_callback(self, msg):
        if msg.range < 0.1:
            self.get_logger().error(f'COLLISION RISK: {msg.range}m')

def main():
    rclpy.init()
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Automated Test Script

See **[SENSOR_TESTING_GUIDE.md](SENSOR_TESTING_GUIDE.md)** Section "Method 3: Python Test Code"

## C++ Examples

### Simple Monitor

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/range.hpp>

class SafetyMonitor : public rclcpp::Node
{
public:
  SafetyMonitor() : Node("safety_monitor")
  {
    temp_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
      "/sensor_state_broadcaster/temperature", 10,
      std::bind(&SafetyMonitor::temp_callback, this, std::placeholders::_1));
    
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "/sensor_state_broadcaster/battery", 10,
      std::bind(&SafetyMonitor::battery_callback, this, std::placeholders::_1));
    
    range_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "/sensor_state_broadcaster/range", 10,
      std::bind(&SafetyMonitor::range_callback, this, std::placeholders::_1));
  }

private:
  void temp_callback(const sensor_msgs::msg::Temperature::SharedPtr msg)
  {
    if (msg->temperature > 80.0) {
      RCLCPP_ERROR(get_logger(), "OVERHEAT: %.1f°C", msg->temperature);
    }
  }
  
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    if (msg->voltage < 10.0) {
      RCLCPP_ERROR(get_logger(), "LOW BATTERY: %.2fV", msg->voltage);
    }
  }
  
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    if (msg->range < 0.1) {
      RCLCPP_ERROR(get_logger(), "COLLISION RISK: %.3fm", msg->range);
    }
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}
```

### Automated Test Code

See **[SENSOR_TESTING_GUIDE.md](SENSOR_TESTING_GUIDE.md)** Section "Method 4: C++ Test Code"

## Configuration Reference

### controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz - controller manager update rate

# Publishes sensor data to ROS topics
sensor_state_broadcaster:
  ros__parameters:
    type: r2c_tutorial/SensorStateBroadcaster
    temperature_sensor_name: "temperature_sensor"
    battery_sensor_name: "battery_sensor"
    range_sensor_name: "range_sensor"
    publish_rate: 10.0  # Hz - topic publishing rate
    temperature_frame_id: "temperature_link"
    battery_frame_id: "base_link"
    range_frame_id: "range_link"

# Injects test values (simulation only)
sensor_command_controller:
  ros__parameters:
    type: r2c_tutorial/SensorCommandController
    temperature_sensor_name: "temperature_sensor"
    battery_sensor_name: "battery_sensor"
    range_sensor_name: "range_sensor"
```

### URDF Configuration

```xml
<!-- Simulation mode -->
<xacro:if value="${sim_mode}">
  <ros2_control name="sensor_system" type="system">
    <hardware>
      <plugin>r2c_tutorial/SensorHardwareInterfaceSim</plugin>
    </hardware>
    
    <!-- Temperature sensor -->
    <sensor name="temperature_sensor">
      <state_interface name="temperature"/>
      <command_interface name="temperature"/>  <!-- Test injection -->
    </sensor>
    
    <!-- Battery sensor (combined voltage + current) -->
    <sensor name="battery_sensor">
      <state_interface name="voltage"/>
      <state_interface name="current"/>
      <command_interface name="voltage"/>     <!-- Test injection -->
      <command_interface name="current"/>     <!-- Test injection -->
    </sensor>
    
    <!-- Range sensor -->
    <sensor name="range_sensor">
      <state_interface name="range"/>
      <command_interface name="range"/>       <!-- Test injection -->
    </sensor>
  </ros2_control>
</xacro:if>

<!-- Real hardware mode -->
<xacro:unless value="${sim_mode}">
  <ros2_control name="sensor_system" type="system">
    <hardware>
      <plugin>r2c_tutorial/SensorHardwareInterface</plugin>
    </hardware>
    
    <!-- Same sensors, but NO command interfaces -->
    <sensor name="temperature_sensor">
      <state_interface name="temperature"/>
    </sensor>
    
    <sensor name="battery_sensor">
      <state_interface name="voltage"/>
      <state_interface name="current"/>
    </sensor>
    
    <sensor name="range_sensor">
      <state_interface name="range"/>
    </sensor>
  </ros2_control>
</xacro:unless>
```

## Troubleshooting

### Controllers Not Loading

**Symptom:** `ros2 control list_controllers` shows controllers as inactive.

**Check:**
```bash
# Verify hardware component loaded
ros2 control list_hardware_components
# Should show: sensor_system [active]

# Check available interfaces
ros2 control list_hardware_interfaces
# Should show:
#   sensor_system/temperature_sensor/temperature [available] [claimed]
#   sensor_system/battery_sensor/voltage [available] [claimed]
#   ...

# Check controller manager logs
ros2 topic echo /controller_manager/transition_event
```

**Common Causes:**
1. Interface name mismatch between URDF and controller config
2. Hardware component not activated
3. Missing plugin in plugin XML
4. Controller claiming wrong interface names

### Command Injection Not Working

**Symptom:** Published commands don't affect sensor values.

**Check:**
```bash
# Verify sensor_command_controller is active
ros2 control list_controllers | grep sensor_command

# Check command topics exist
ros2 topic list | grep sensor_command_controller

# Verify message type (must be Float64, not Float32!)
ros2 topic info /sensor_command_controller/temperature_cmd

# Monitor if messages are received
ros2 topic echo /sensor_command_controller/temperature_cmd
```

**Common Causes:**
1. Using Float32 instead of Float64
2. Controller not active in simulation mode
3. Command interfaces not exported by hardware
4. Publishing to wrong topic name

### Sensor Values Not Updating

**Symptom:** sensor_reader.py shows no data or stale data.

**Check:**
```bash
# Verify sensor_state_broadcaster is active
ros2 control list_controllers | grep sensor_state

# Check published topics
ros2 topic list | grep sensor_state_broadcaster

# Check publish rate
ros2 topic hz /sensor_state_broadcaster/temperature

# Monitor raw values
ros2 topic echo /sensor_state_broadcaster/temperature
```

**Common Causes:**
1. Hardware interface read() not called
2. Controller manager not running
3. Publish rate set to 0
4. Interface names don't match configuration

### Simulation vs Real Hardware

**Symptom:** Works in simulation but not on real hardware (or vice versa).

**Check:**
```bash
# Verify sim_mode parameter
ros2 param get /controller_manager use_sim_time

# Check which hardware plugin loaded
ros2 control list_hardware_components

# For real hardware, verify Teensy topics exist
ros2 topic list | grep teensy
```

**Remember:**
- Simulation: SensorHardwareInterfaceSim with command interfaces
- Real hardware: SensorHardwareInterface, subscribes to Teensy topics, NO command interfaces
- sensor_command_controller should NOT be spawned in real hardware mode

## Related Documentation

- **[SENSOR_TESTING_GUIDE.md](SENSOR_TESTING_GUIDE.md)** - Complete testing guide with examples
- **[SENSOR_INTEGRATION_GUIDE.md](SENSOR_INTEGRATION_GUIDE.md)** - Architecture and design decisions
- **[TUTORIAL.md](TUTORIAL.md)** - General ros2_control tutorial
- **[ADVANCED_TOPICS.md](ADVANCED_TOPICS.md)** - Performance and optimization

## Quick Command Reference

```bash
# Launch simulation
ros2 launch r2c_tutorial gazebo_foxglove.launch.py

# Monitor all sensors
ros2 run r2c_tutorial sensor_reader.py

# Test thermal protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0

# Test battery protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=9.5

# Test overcurrent protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=10.0

# Test collision avoidance
ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.05

# Reset to simulation
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=0.0

# Check controller status
ros2 control list_controllers

# Check hardware interfaces
ros2 control list_hardware_interfaces

# Monitor raw sensor topics
ros2 topic echo /sensor_state_broadcaster/temperature
ros2 topic echo /sensor_state_broadcaster/battery
ros2 topic echo /sensor_state_broadcaster/range
```

---

**This completes the sensor integration reference. You now have everything you need to:**
- Understand the complete architecture
- Monitor sensor data in your safety systems
- Inject test values to validate fault handling
- Write automated tests in Python or C++
- Debug any issues that arise

**Happy testing! 🛡️**
