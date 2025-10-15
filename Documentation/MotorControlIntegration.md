# Motor Control Integration for ros2_control TeensyV2

## Overview
This document describes the motor control integration between ROS2 and the TeensyV2 ros2_control firmware on Board1.

## Architecture

### Signal Flow
```
/cmd_vel (Twist) 
    ↓
teensy_bridge_node (ROS2)
    ↓
MOTOR:left_rpm:X,right_rpm:Y (Serial)
    ↓
SerialManager (TeensyV2)
    ↓
RoboClawMonitor::handleMotorCommand()
    ↓
RoboClaw Motor Controller (Hardware)
```

## Message Format

### ROS2 Topic
- **Topic**: `/cmd_vel`
- **Type**: `geometry_msgs/msg/Twist`
- **Fields**:
  - `linear.x`: Forward velocity (m/s)
  - `angular.z`: Rotational velocity (rad/s)

### TeensyV2 Serial Protocol
- **Format**: `MOTOR:left_rpm:<value>,right_rpm:<value>\n`
- **Example**: `MOTOR:left_rpm:25.5,right_rpm:30.2\n`

## Kinematics

### Robot Parameters
- **Wheel Diameter**: 0.102224144529039 m
- **Wheel Base**: 0.3906 m
- **Quadrature Pulses/Rev**: 1000
- **Max Speed**: 1392 QPPS

### Differential Drive Equations
```
v_left = linear.x - (angular.z × wheel_base / 2)
v_right = linear.x + (angular.z × wheel_base / 2)

left_rpm = (v_left / (π × diameter)) × 60
right_rpm = (v_right / (π × diameter)) × 60
```

## Implementation Details

### teensy_bridge Node
**Location**: `sigyn_to_sensor_v2/src/teensy_bridge.cpp`

**Key Function**: `CmdVelCallback()`
- Subscribes to `/cmd_vel`
- Converts Twist to differential wheel velocities
- Formats MOTOR command
- Queues for transmission to Board1 via serial

### TeensyV2 Firmware
**Location**: `TeensyV2/modules/roboclaw/roboclaw_monitor.cpp`

**Key Functions**:
- `SerialManager::handleCommand()` - Parses incoming MOTOR commands
- `RoboClawMonitor::handleMotorCommand()` - Processes motor RPM commands
- `RoboClawMonitor::setMotorRPMs()` - Converts RPM to QPPS
- `RoboClawMonitor::executeMotorCommand()` - Sends to RoboClaw hardware

### Safety Features
- **Command Timeout**: 200ms - motors stop if no new commands
- **Max Speed Limits**: Enforced in firmware (1392 QPPS)
- **E-Stop Integration**: Emergency stop overrides all motor commands
- **Runaway Detection**: Monitors encoder feedback vs commanded speed

## Testing

### Launch the Bridge
```bash
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py
```

### Verify /cmd_vel Subscription
```bash
ros2 topic info /cmd_vel
# Should show: Subscription count: 1
```

### Send Test Commands
```bash
# Forward motion (0.1 m/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotation (0.2 rad/s)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Use Test Script
```bash
./scripts/test_motor_cmd_vel.sh
```

### Monitor Debug Output
```bash
# Watch teensy_bridge logs
ros2 run rqt_console rqt_console

# Or via command line
ros2 topic echo /rosout | grep teensy_bridge
```

## Troubleshooting

### No /cmd_vel Listener
**Symptom**: `ros2 topic info /cmd_vel` shows 0 subscriptions
**Fix**: Ensure teensy_bridge is running: `ros2 node list | grep teensy_bridge`

### Motors Not Responding
**Symptom**: Commands sent but wheels don't move
**Check**:
1. Verify serial connection: Check teensy_bridge logs for "Connected to Board 1"
2. Check TeensyV2 is receiving: Monitor USB serial output from Teensy
3. Verify RoboClaw connection: Check ROBOCLAW diagnostic messages
4. E-Stop status: Ensure no safety violations active

### Incorrect Wheel Speeds
**Symptom**: Robot moves faster/slower than expected
**Fix**: Verify wheel diameter and base measurements in both teensy_bridge.cpp and roboclaw_monitor.cpp

### Command Timeout Errors
**Symptom**: Motors stop unexpectedly
**Cause**: No cmd_vel received for >200ms
**Fix**: Ensure continuous cmd_vel publication (e.g., from nav2 or teleop)

## Configuration

### Adjusting Safety Timeouts
Edit `TeensyV2/modules/roboclaw/roboclaw_monitor.cpp`:
```cpp
constexpr uint32_t MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS = 200;
```

### Adjusting Max Speeds
Edit `TeensyV2/modules/roboclaw/roboclaw_monitor.cpp`:
```cpp
constexpr uint32_t MAX_MOTOR_SPEED_QPPS = 1392;
constexpr uint32_t MAX_ACCELERATION_QPPS2 = 3000;
```

## Integration with Nav2
The `/cmd_vel` topic is the standard interface for Nav2 navigation stack:
- `nav2_controller` publishes to `/cmd_vel`
- `teensy_bridge` subscribes and forwards to hardware
- No additional configuration needed for Nav2 integration

## Future Enhancements
- [ ] Add odometry feedback loop verification
- [ ] Implement velocity profiling/smoothing
- [ ] Add diagnostic monitoring for command latency
- [ ] Support for closed-loop position control
- [ ] Integration with ros2_control DiffDriveController (full ros2_control stack)
