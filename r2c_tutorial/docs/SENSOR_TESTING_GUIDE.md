# Sensor Testing Guide for Safety Systems

This guide explains how to use the sensor command injection system to test safety systems in simulation mode.

## Overview

The sensor testing system allows you to inject specific sensor values in simulation mode to test how your safety systems respond to various fault conditions. This is essential for validating thermal protection, battery management, overcurrent protection, and collision avoidance systems without risking real hardware damage.

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Your Safety System                          │
│  (subscribes to /sensor_state_broadcaster/* topics)              │
└─────────────────┬───────────────────────────────────────────────┘
                  │ Reacts to sensor values
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│              SensorStateBroadcaster Controller                   │
│  Reads state interfaces → Publishes ROS messages                 │
└─────────────────┬───────────────────────────────────────────────┘
                  │ Reads from hardware
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│         SensorHardwareInterfaceSim (Simulation)                  │
│  State interfaces ← Command interfaces (test injection)          │
│  - Normal mode: Generates synthetic sensor data                  │
│  - Test mode: Uses commanded values (when non-zero)              │
└─────────────────┬───────────────────────────────────────────────┘
                  │ Receives commands from
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│           SensorCommandController (Simulation Only)              │
│  Subscribes to command topics → Writes to command interfaces     │
└─────────────────┬───────────────────────────────────────────────┘
                  │ Receives from
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│              Test Scripts / Command Line                         │
│  - sensor_writer.py (Python)                                     │
│  - ros2 topic pub (command line)                                 │
│  - Your custom test code (Python or C++)                         │
└─────────────────────────────────────────────────────────────────┘
```

## Command Topics

The `sensor_command_controller` subscribes to these topics:

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/sensor_command_controller/temperature_cmd` | `std_msgs/msg/Float64` | Inject temperature (°C) |
| `/sensor_command_controller/voltage_cmd` | `std_msgs/msg/Float64` | Inject battery voltage (V) |
| `/sensor_command_controller/current_cmd` | `std_msgs/msg/Float64` | Inject battery current (A) |
| `/sensor_command_controller/range_cmd` | `std_msgs/msg/Float64` | Inject distance (m) |

**Important:** Send `0.0` to any command topic to return to normal simulation mode.

## Method 1: Using sensor_writer.py (Recommended)

### Basic Usage

```bash
# Test thermal protection (high temperature)
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0

# Test battery protection (low voltage)
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=9.5

# Test overcurrent protection
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=10.0

# Test collision avoidance (close obstacle)
ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.05
```

### Combined Test Scenarios

```bash
# Critical battery state: Low voltage + high current
ros2 run r2c_tutorial sensor_writer.py --ros-args \
    -p voltage_cmd:=9.5 \
    -p current_cmd:=8.5

# Thermal overload: High temperature + high current
ros2 run r2c_tutorial sensor_writer.py --ros-args \
    -p temperature_cmd:=85.0 \
    -p current_cmd:=9.0

# Emergency shutdown test: Multiple failures
ros2 run r2c_tutorial sensor_writer.py --ros-args \
    -p temperature_cmd:=95.0 \
    -p voltage_cmd:=8.5 \
    -p current_cmd:=10.0 \
    -p range_cmd:=0.03
```

### Reset to Normal Simulation

```bash
# Reset all sensors to normal simulation
ros2 run r2c_tutorial sensor_writer.py --ros-args \
    -p temperature_cmd:=0.0 \
    -p voltage_cmd:=0.0 \
    -p current_cmd:=0.0 \
    -p range_cmd:=0.0
```

## Method 2: Direct Topic Publishing

### Single Value Injection

```bash
# Inject high temperature
ros2 topic pub /sensor_command_controller/temperature_cmd \
    std_msgs/msg/Float64 "{data: 93.0}" --once

# Inject low voltage
ros2 topic pub /sensor_command_controller/voltage_cmd \
    std_msgs/msg/Float64 "{data: 9.5}" --once

# Inject high current
ros2 topic pub /sensor_command_controller/current_cmd \
    std_msgs/msg/Float64 "{data: 10.0}" --once

# Inject close obstacle
ros2 topic pub /sensor_command_controller/range_cmd \
    std_msgs/msg/Float64 "{data: 0.05}" --once
```

### Continuous Publishing

```bash
# Continuously inject high temperature at 10 Hz
ros2 topic pub /sensor_command_controller/temperature_cmd \
    std_msgs/msg/Float64 "{data: 93.0}" --rate 10
```

### Reset to Normal

```bash
# Reset temperature to simulation
ros2 topic pub /sensor_command_controller/temperature_cmd \
    std_msgs/msg/Float64 "{data: 0.0}" --once
```

## Method 3: Python Test Code

### Simple Test Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time


class SafetySystemTester(Node):
    def __init__(self):
        super().__init__('safety_system_tester')
        
        # Create publishers
        self.temp_pub = self.create_publisher(
            Float64, '/sensor_command_controller/temperature_cmd', 10)
        self.voltage_pub = self.create_publisher(
            Float64, '/sensor_command_controller/voltage_cmd', 10)
        self.current_pub = self.create_publisher(
            Float64, '/sensor_command_controller/current_cmd', 10)
        self.range_pub = self.create_publisher(
            Float64, '/sensor_command_controller/range_cmd', 10)
        
        time.sleep(0.5)  # Wait for connections
    
    def inject_temperature(self, temp):
        """Inject a temperature value."""
        msg = Float64()
        msg.data = temp
        self.temp_pub.publish(msg)
        self.get_logger().info(f'Injected temperature: {temp}°C')
    
    def inject_voltage(self, voltage):
        """Inject a voltage value."""
        msg = Float64()
        msg.data = voltage
        self.voltage_pub.publish(msg)
        self.get_logger().info(f'Injected voltage: {voltage}V')
    
    def inject_current(self, current):
        """Inject a current value."""
        msg = Float64()
        msg.data = current
        self.current_pub.publish(msg)
        self.get_logger().info(f'Injected current: {current}A')
    
    def inject_range(self, range_val):
        """Inject a range value."""
        msg = Float64()
        msg.data = range_val
        self.range_pub.publish(msg)
        self.get_logger().info(f'Injected range: {range_val}m')
    
    def reset_all(self):
        """Reset all sensors to normal simulation."""
        self.inject_temperature(0.0)
        self.inject_voltage(0.0)
        self.inject_current(0.0)
        self.inject_range(0.0)
        self.get_logger().info('Reset to normal simulation')


def main():
    rclpy.init()
    tester = SafetySystemTester()
    
    try:
        # Example test sequence
        tester.get_logger().info('Starting safety system test...')
        
        # Test 1: Gradual temperature increase
        tester.get_logger().info('Test 1: Thermal protection')
        for temp in [50.0, 60.0, 70.0, 80.0, 90.0, 95.0]:
            tester.inject_temperature(temp)
            time.sleep(2)  # Wait 2 seconds between steps
        
        # Reset
        tester.reset_all()
        time.sleep(2)
        
        # Test 2: Battery discharge
        tester.get_logger().info('Test 2: Battery protection')
        for voltage in [11.5, 11.0, 10.5, 10.0, 9.5, 9.0]:
            tester.inject_voltage(voltage)
            time.sleep(2)
        
        # Reset
        tester.reset_all()
        time.sleep(2)
        
        # Test 3: Approaching obstacle
        tester.get_logger().info('Test 3: Collision avoidance')
        for distance in [1.0, 0.5, 0.3, 0.2, 0.1, 0.05]:
            tester.inject_range(distance)
            time.sleep(1)
        
        # Final reset
        tester.reset_all()
        
        tester.get_logger().info('Test sequence complete!')
        
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Method 4: C++ Test Code

### Complete Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <chrono>
#include <thread>

class SafetySystemTester : public rclcpp::Node
{
public:
  SafetySystemTester() : Node("safety_system_tester")
  {
    // Create publishers
    temp_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/sensor_command_controller/temperature_cmd", 10);
    voltage_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/sensor_command_controller/voltage_cmd", 10);
    current_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/sensor_command_controller/current_cmd", 10);
    range_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/sensor_command_controller/range_cmd", 10);
    
    // Wait for connections
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  void inject_temperature(double temp)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = temp;
    temp_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Injected temperature: %.1f°C", temp);
  }
  
  void inject_voltage(double voltage)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = voltage;
    voltage_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Injected voltage: %.2fV", voltage);
  }
  
  void inject_current(double current)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = current;
    current_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Injected current: %.2fA", current);
  }
  
  void inject_range(double range)
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = range;
    range_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Injected range: %.3fm", range);
  }
  
  void reset_all()
  {
    inject_temperature(0.0);
    inject_voltage(0.0);
    inject_current(0.0);
    inject_range(0.0);
    RCLCPP_INFO(get_logger(), "Reset to normal simulation");
  }
  
  void run_test_sequence()
  {
    RCLCPP_INFO(get_logger(), "Starting safety system test...");
    
    // Test 1: Thermal protection
    RCLCPP_INFO(get_logger(), "Test 1: Thermal protection");
    std::vector<double> temps = {50.0, 60.0, 70.0, 80.0, 90.0, 95.0};
    for (double temp : temps) {
      inject_temperature(temp);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    reset_all();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Test 2: Battery protection
    RCLCPP_INFO(get_logger(), "Test 2: Battery protection");
    std::vector<double> voltages = {11.5, 11.0, 10.5, 10.0, 9.5, 9.0};
    for (double voltage : voltages) {
      inject_voltage(voltage);
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    reset_all();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Test 3: Collision avoidance
    RCLCPP_INFO(get_logger(), "Test 3: Collision avoidance");
    std::vector<double> distances = {1.0, 0.5, 0.3, 0.2, 0.1, 0.05};
    for (double distance : distances) {
      inject_range(distance);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    reset_all();
    RCLCPP_INFO(get_logger(), "Test sequence complete!");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr range_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto tester = std::make_shared<SafetySystemTester>();
  
  // Run test sequence in separate thread
  std::thread test_thread([&tester]() {
    tester->run_test_sequence();
  });
  
  // Spin node
  rclcpp::spin(tester);
  
  test_thread.join();
  rclcpp::shutdown();
  return 0;
}
```

### Build Configuration

Add to your `CMakeLists.txt`:

```cmake
add_executable(safety_system_tester src/safety_system_tester.cpp)
ament_target_dependencies(safety_system_tester rclcpp std_msgs)
install(TARGETS safety_system_tester DESTINATION lib/${PROJECT_NAME})
```

## Recommended Test Scenarios

### 1. Thermal Protection Test

**Goal:** Verify system shuts down or enters safe mode at high temperature.

```bash
# Gradual increase
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=70.0
# Wait and observe response
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=80.0
# Wait and observe response
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=90.0
# Should trigger protection!
```

**Expected behavior:**
- Warning at 80°C
- Emergency shutdown at 90°C
- Robot stops moving
- Error logged

### 2. Battery Protection Test

**Goal:** Verify low battery warning and emergency behavior.

```bash
# Normal battery
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=11.5
# Low battery warning
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=10.5
# Critical battery
ros2 run r2c_tutorial sensor_writer.py --ros-args -p voltage_cmd:=9.5
```

**Expected behavior:**
- Warning at <11.0V
- Reduced performance at <10.5V
- Emergency return-to-home at <10.0V
- Shutdown at <9.5V

### 3. Overcurrent Protection Test

**Goal:** Verify current limiting and motor protection.

```bash
# Normal operation
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=5.0
# High load
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=8.0
# Overcurrent
ros2 run r2c_tutorial sensor_writer.py --ros-args -p current_cmd:=10.0
```

**Expected behavior:**
- Warning at >7.0A
- Speed reduction at >8.0A
- Motor disable at >10.0A

### 4. Collision Avoidance Test

**Goal:** Verify obstacle detection and avoidance.

```bash
# Far obstacle
ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.5
# Approaching
ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.2
# Very close
ros2 run r2c_tutorial sensor_writer.py --ros-args -p range_cmd:=0.05
```

**Expected behavior:**
- Slow down at <0.3m
- Stop at <0.1m
- Reverse at <0.05m

### 5. Combined Failure Test

**Goal:** Verify behavior under multiple simultaneous failures.

```bash
# Simulate complete system failure
ros2 run r2c_tutorial sensor_writer.py --ros-args \
    -p temperature_cmd:=95.0 \
    -p voltage_cmd:=9.0 \
    -p current_cmd:=10.0 \
    -p range_cmd:=0.03
```

**Expected behavior:**
- Immediate emergency shutdown
- All motors disabled
- Error logged to file
- Visual/audio alarm (if available)

## Monitoring Test Results

### Terminal 1: Run Simulation
```bash
ros2 launch r2c_tutorial gazebo_foxglove.launch.py
```

### Terminal 2: Monitor Sensors
```bash
ros2 run r2c_tutorial sensor_reader.py
```

### Terminal 3: Inject Commands
```bash
ros2 run r2c_tutorial sensor_writer.py --ros-args -p temperature_cmd:=93.0
```

### Terminal 4: Monitor Your Safety System
```bash
# Your safety system node
ros2 run your_package safety_monitor
```

## Important Notes

### Simulation Only

⚠️ **This command injection system ONLY works in simulation mode!**

In real hardware mode (`sim_mode:=false`):
- Sensor values come from actual Teensy hardware
- Command interfaces are NOT exposed
- Attempting to use sensor_command_controller will fail

### Reset Behavior

- Sending `0.0` to any command topic returns that sensor to normal simulation
- Normal simulation generates realistic synthetic sensor data
- Non-zero commands override simulation until reset

### Safety Considerations

Even in simulation, test your safety systems thoroughly:
1. Test each protection mechanism individually
2. Test combinations of failures
3. Verify recovery after reset
4. Log all test results
5. Document expected vs. actual behavior

## Troubleshooting

### Commands Not Working

```bash
# Check controller is active
ros2 control list_controllers

# Should show:
# sensor_command_controller[r2c_tutorial/SensorCommandController] active

# Check topics exist
ros2 topic list | grep sensor_command_controller
```

### No Response to Commands

```bash
# Verify messages are being received
ros2 topic echo /sensor_command_controller/temperature_cmd

# Check sensor values are updating
ros2 topic echo /sensor_state_broadcaster/temperature
```

### Controllers Not Loading

```bash
# Check you're in simulation mode
ros2 param get /controller_manager use_sim_time
# Should return: true

# Check hardware component
ros2 control list_hardware_components
# sensor_system should show ACTIVE
```

## Next Steps

1. **Implement your safety system** that subscribes to sensor topics
2. **Write automated tests** using Python or C++ examples above
3. **Document test results** for each protection mechanism
4. **Create CI/CD tests** to run automatically
5. **Extend to real hardware** (remove command injection code)

## Additional Resources

- [sensor_state_broadcaster.hpp](../include/r2c_tutorial/sensor_state_broadcaster.hpp) - Controller that publishes sensor data
- [sensor_command_controller.hpp](../include/r2c_tutorial/sensor_command_controller.hpp) - Controller that injects test values
- [sensor_hardware_interface_sim.hpp](../include/r2c_tutorial/sensor_hardware_interface_sim.hpp) - Hardware interface simulation
- [sensor_reader.py](../scripts/sensor_reader.py) - Example monitoring script
- [sensor_writer.py](../scripts/sensor_writer.py) - Example command injection script

---

**Happy Testing! 🛡️**
