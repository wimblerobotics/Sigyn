# Sensor Integration with ros2_control

This tutorial demonstrates how to integrate custom sensors into ros2_control, supporting both real hardware and simulation.

## Architecture

### Two Hardware Systems

1. **r2c_test_system** (Gazebo) - Controls joints:
   - left_wheel_joint (velocity)
   - right_wheel_joint (velocity)
   - arm_joint (position)

2. **sensor_system** (Custom Plugin) - Monitors sensors:
   - temperature_sensor/temperature (°C)
   - voltage_sensor/voltage (V)
   - current_sensor/current (A)
   - vl53l0x_sensor/range (m)

### How It Works

The `SensorHardwareInterface` plugin subscribes to ROS topics and exposes the data through ros2_control state interfaces.

#### Real Hardware Mode (use_sim_time:=false)
```
Teensy → sigyn_to_sensor_v2 → Topics → SensorHardwareInterface → ros2_control
```

Topics:
- `/sigyn/teensy_bridge/battery_state` (BatteryState: voltage, current)
- `/sensors/temperature` (Temperature)
- `/sensors/range` (Range: VL53L0X)

#### Simulation Mode (use_sim_time:=true)
```
sensor_simulator.py → Topics → SensorHardwareInterface → ros2_control
```

Topics:
- `/sensors/temperature` (simulated temperature)
- `/sensors/voltage` (simulated battery voltage)
- `/sensors/current` (simulated current draw)
- `/sensors/range` (simulated VL53L0X distance)

## Benefits

✅ **Unified Interface**: Same ros2_control interface for real and simulated sensors
✅ **Easy Swapping**: Change hardware without modifying controllers
✅ **Topic-Based**: Leverages existing ROS ecosystem (no custom protocols)
✅ **Testable**: Full simulation without hardware

## Usage

### Check Available Hardware Interfaces
```bash
ros2 control list_hardware_interfaces
```

You should see:
```
state interfaces
  temperature_sensor/temperature
  voltage_sensor/voltage  
  current_sensor/current
  vl53l0x_sensor/range
```

### Read Sensor Values
```bash
# Via ros2_control
ros2 control list_hardware_interfaces

# Via standard ROS topics
ros2 topic echo /sensors/temperature
ros2 topic echo /sensors/voltage
ros2 topic echo /sensors/current
ros2 topic echo /sensors/range
```

## Integration with Your Robot

When integrating with Sigyn's real hardware:

1. Ensure `sigyn_to_sensor_v2` is running and publishing to:
   - `/sigyn/teensy_bridge/battery_state`
   
2. The hardware interface will automatically subscribe and update ros2_control state

3. Controllers can read sensor values through the hardware interface

## Future Enhancements

- Add GPIO controller to broadcast sensor states to topics
- Create custom controller that uses sensor feedback
- Add more sensors (IMU, additional VL53L0X, etc.)
- Implement sensor fusion in controllers
