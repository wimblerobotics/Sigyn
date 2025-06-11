# SigynCore Firmware

This is a rewritten C++ firmware project for the Teensy 4.2 microprocessor designed to manage low-level hardware control for the 'Sigyn' home robot. It provides a robust, modular architecture for interfacing between ROS2 and robot hardware.

## Architecture Overview

The firmware is built around a modular architecture using the `TModule` base class, which provides:

- **Singleton Pattern**: Each hardware module is implemented as a singleton for safe global access
- **State Machine Design**: Modules use state machines for robust operation and error recovery
- **Performance Monitoring**: Built-in timing statistics for all modules
- **Safety Systems**: Comprehensive error detection and recovery mechanisms

## Core Components

### Main Application
- **`SigynCore.ino`**: Main Arduino sketch that orchestrates all modules
- **`config.h`**: Central configuration file containing all robot parameters

### Module Framework
- **`TModule.h/.cpp`**: Abstract base class providing the modular framework
  - Automatic registration and lifecycle management
  - Performance timing and statistics collection
  - Safety monitoring interface

### Hardware Modules

#### RoboClawModule (`RoboClawModule.h/.cpp`)
**Primary motor control and odometry module**

**Key Features:**
- **Differential Drive Control**: Converts twist commands to individual wheel speeds
- **Real-time Odometry**: High-frequency pose and velocity calculation from encoder data
- **State Machine Operation**: Robust connection management and error recovery
- **Safety Monitoring**: Motor current monitoring and error detection
- **Performance Optimized**: Intelligent rate limiting to balance responsiveness and efficiency

**Communication:**
- Serial interface to RoboClaw 2x15A motor controller at 38400 baud
- Encoder feedback processing (1000 pulses per revolution)
- Current monitoring and error status reporting

#### BatteryModule (`BatteryModule.h/.cpp`)
**Battery monitoring and management**

**Key Features:**
- **Voltage Monitoring**: Continuous battery voltage reading with averaging
- **Percentage Calculation**: Smart battery percentage estimation
- **Safety Thresholds**: Low battery detection and warnings
- **Telemetry**: Regular battery status reporting

#### SerialManager (`SerialManager.h/.cpp`)
**USB serial communication management**

**Key Features:**
- **Message Formatting**: Structured output for different message types
- **Diagnostic Reporting**: Error and status message handling
- **High-Speed Communication**: 921600 baud USB serial interface

## Communication Protocol

### Serial Interface (USB)

The firmware communicates with the host PC (ROS2) via USB serial at 921600 baud using a structured message protocol.

#### Incoming Messages (PC → Teensy)

**Twist Commands:**
```
Format: TWIST:<linear_x>,<angular_z>
Example: TWIST:0.5,0.2
```
- `linear_x`: Forward/backward velocity in m/s
- `angular_z`: Rotational velocity in rad/s

#### Outgoing Messages (Teensy → PC)

**Odometry Data:**
```
Format: px=<x>,py=<y>,ox=<qx>,oy=<qy>,oz=<qz>,ow=<qw>,vx=<vx>,vy=<vy>,wz=<wz>
Example: px=1.23,py=0.45,ox=0.12,oy=0.34,oz=0.56,ow=0.78,vx=0.5,vy=0.0,wz=0.2
```

**Battery Status:**
```
Format: b <voltage_V> <percentage_%>
Example: b 12.5 80
```

**RoboClaw Status (JSON):**
```json
{
  "LogicVoltage": 4.9,
  "MainVoltage": 12.6,
  "Encoder_Left": 1500,
  "Encoder_Right": 1520,
  "LeftMotorCurrent": 1.2,
  "RightMotorCurrent": 1.3,
  "LeftMotorSpeed": 850,
  "RightMotorSpeed": 860,
  "Error": 0
}
```

**Diagnostic Messages:**
```
Format: DIAG:<message>
Example: DIAG:INFO [RoboClawModule] Connected successfully
```

## Robot Physical Parameters

The firmware is configured for a differential drive robot with the following specifications:

```cpp
// Wheel Configuration
#define WHEEL_DIAMETER_M 0.102224144529039f    // ~4 inches
#define WHEEL_BASE_M 0.3906f                   // ~15.4 inches
#define QUADRATURE_PULSES_PER_REVOLUTION 1000  // Encoder resolution

// Performance Limits
#define MAX_MOTOR_SPEED_QPPS 1392             // ~1 mph max speed
#define MAX_ACCELERATION_QPPS2 3000           // Acceleration limit

// Safety Timeouts
#define MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS 200  // 200ms timeout
```

## Key Features

### Intelligent Rate Limiting
The firmware implements smart rate limiting to balance performance and responsiveness:
- **Motor Commands**: Limited to ~80 Hz (4x typical teleop rate)
- **Odometry Updates**: Configurable update rate for optimal performance
- **Status Publishing**: 1 Hz comprehensive status reports

### Error Recovery
**State Machine Design**: Each module operates with robust state machines that handle:
- **Connection Loss**: Automatic reconnection and reinitialization
- **Communication Errors**: Graceful degradation and recovery
- **Hardware Faults**: Safe shutdown and diagnostic reporting

### Performance Monitoring
All modules provide detailed performance statistics:
- **Execution Time**: Min/max/average execution times
- **Loop Frequency**: Actual vs. target performance metrics
- **Error Rates**: Communication and hardware error tracking

## Hardware Interface

### Pin Assignments
```cpp
#define E_STOP_PIN 5              // Emergency stop control
#define MAIN_BATTERY_PIN 27       // Battery voltage sensing
#define ROBOCLAW_SERIAL Serial6   // RoboClaw communication
```

### RoboClaw Configuration
- **Address**: 0x80 (default)
- **Baud Rate**: 38400
- **Timeout**: 10ms
- **Expected Version**: "USB Roboclaw 2x15a v4.2.8"

## Development and Compilation

### Prerequisites
- **Arduino IDE** with Teensyduino add-on
- **RoboClaw Arduino Library**
- **Teensy 4.2** board support

### Build Process
1. Open `SigynCore.ino` in Arduino IDE
2. Select "Teensy 4.2" board
3. Configure USB Type to "Serial"
4. Set CPU Speed as appropriate (600 MHz recommended)
5. Upload to Teensy

### Dependencies
- Standard Arduino libraries
- RoboClaw library (for motor controller communication)
- Built-in Teensy 4.2 hardware libraries

## Safety Features

### Motor Safety
- **Timeout Protection**: Motors stop if no command received within 200ms
- **Speed Limiting**: Hardware-enforced maximum speed limits
- **Current Monitoring**: Real-time motor current tracking
- **Error Detection**: Comprehensive RoboClaw error monitoring

### System Safety
- **E-Stop Support**: Hardware emergency stop capability
- **Battery Monitoring**: Low voltage detection and warnings
- **Communication Monitoring**: Serial link health checking
- **Graceful Degradation**: Safe operation during partial failures

## Performance Characteristics

### Typical Execution Times
- **Motor Command Update**: ~26ms (when not rate-limited)
- **Odometry Calculation**: ~5-10ms per update
- **Status Publishing**: ~24-25ms (1 Hz)
- **Main Loop**: High frequency (limited by module timing)

### Communication Bandwidth
- **USB Serial**: 921600 baud
- **RoboClaw Serial**: 38400 baud
- **Update Rates**: Optimized for real-time operation

## Integration with ROS2

This firmware is designed to work with the `sigyn_to_teensy` ROS2 package, which:
- Subscribes to `geometry_msgs/Twist` on `/cmd_vel`
- Publishes `nav_msgs/Odometry` on `/wheel_odom`
- Provides battery and diagnostic topics
- Handles coordinate frame transformations

## Future Enhancements

The modular architecture supports easy addition of:
- **Additional Sensors**: IMU, ultrasonic, time-of-flight
- **Display Integration**: Status displays and user interfaces
- **Advanced Safety**: Collision detection and avoidance
- **Performance Optimization**: Further timing and efficiency improvements

## Troubleshooting

### Common Issues
1. **RoboClaw Version Mismatch**: Check version string in config.h
2. **Serial Communication**: Verify baud rates and connections
3. **Motor Not Responding**: Check RoboClaw address and wiring
4. **Odometry Drift**: Verify encoder connections and wheel parameters

### Diagnostic Messages
The firmware provides detailed diagnostic output via USB serial. Monitor these messages for:
- Connection status
- Error conditions
- Performance metrics
- Safety alerts

## License

This firmware is part of the Sigyn robot project. See the main project LICENSE for details.
