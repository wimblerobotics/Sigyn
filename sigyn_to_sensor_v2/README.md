# sigyn_to_sensor_v2

Advanced ROS2 interface package for the TeensyV2 embedded system, providing real-time sensor data processing, safety coordination, and comprehensive parameter management for the Sigyn robotic platform.

## Overview

This package serves as the primary communication bridge between the TeensyV2 embedded system (running on dual Teensy 4.1 boards) and the ROS2 ecosystem. It provides:

- **Real-time Communication**: High-speed serial communication with embedded boards
- **Safety Coordination**: E-stop management and safety status monitoring
- **Parameter Interface**: Runtime configuration of embedded system parameters
- **Sensor Integration**: Battery monitoring, performance tracking, and diagnostic reporting
- **Modular Architecture**: Clean separation of concerns with dedicated nodes for different functions

## Architecture

### Node Structure

The package implements a streamlined single-node architecture:

```
sigyn_to_sensor_v2/
└── teensy_bridge_node      # Main communication bridge and data processing
```

### Communication Flow

```
[Teensy Board 1] ←→ [teensy_bridge_node] ←→ [ROS2 Topics/Services]
[Teensy Board 2] ←→ [teensy_bridge_node] ←→ [ROS2 Topics/Services]
                              ↓
                    [Direct Topic Publishing]
                              ↓
                    [Battery, IMU, Safety, Performance Topics]
```

**Note**: Previous architecture with separate monitor nodes has been consolidated into the main bridge node for improved performance and reduced complexity.

## Features

### Real-Time Communication
- **High-Speed Serial**: 921600 baud communication with embedded boards
- **Message Parsing**: Efficient parsing of structured TeensyV2 message protocol
- **Error Recovery**: Automatic reconnection and error handling
- **Message Validation**: CRC checking and sequence number validation

### Safety System Integration
- **E-Stop Management**: Hardware and software emergency stop coordination
- **Safety Status Publishing**: Real-time safety state monitoring
- **Multi-Source E-Stops**: Battery, performance, motor, and manual E-stop sources
- **Automatic Recovery**: Configurable automatic recovery for transient conditions

### Parameter Management
- **Runtime Configuration**: Live parameter updates to embedded system
- **Persistent Storage**: Parameter persistence across system restarts
- **Validation**: Parameter range checking and validation
- **Default Handling**: Comprehensive default parameter management

### Monitoring and Diagnostics
- **Performance Monitoring**: Real-time timing and frequency analysis
- **Battery Monitoring**: Voltage, current, and power consumption tracking
- **System Health**: Comprehensive diagnostic reporting
- **Statistical Analysis**: Performance trends and violation tracking

## Quick Start

### Prerequisites
```bash
# Ensure ROS2 Jazzy is installed and sourced
source /opt/ros/jazzy/setup.bash

# Install dependencies (if not already available)
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-sensor-msgs
```

### Building
```bash
cd /path/to/sigyn_ws
colcon build --packages-select sigyn_to_sensor_v2 --symlink-install
source install/setup.bash
```

### Running
```bash
# Launch the TeensyV2 interface bridge
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py

# Or run the main bridge node directly
ros2 run sigyn_to_sensor_v2 teensy_bridge_node
```

### Configuration
```bash
# View available parameters
ros2 param list

# Update battery monitoring thresholds
ros2 param set /battery_monitor critical_low_voltage 30.0
ros2 param set /battery_monitor warning_low_voltage 32.0

# Configure performance monitoring
ros2 param set /performance_monitor target_frequency 85.0
ros2 param set /performance_monitor max_execution_time_ms 2.0
```

## Topics

### Published Topics

#### Battery Monitoring
- `/battery_state` (sensor_msgs/BatteryState) - Battery voltage, current, percentage, and location
  - **New Feature**: `location` field indicates physical battery location (e.g., "36VLIPO", "5VDCDC")
  - Supports multiple battery types: 36V LiPo main battery, 5V DC-DC, 24V DC-DC, 3.3V DC-DC, 12V DC-DC

#### IMU Data
- `/imu/sensor0` (sensor_msgs/Imu) - Primary IMU sensor data (quaternion orientation)
- `/imu/sensor1` (sensor_msgs/Imu) - Secondary IMU sensor data (quaternion orientation)

#### Safety and Diagnostics
- `/estop_status` (custom message) - Emergency stop events and status
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) - System diagnostic messages
- `/performance_stats` (custom message) - Real-time performance monitoring

### Subscribed Topics

#### Commands
- `/commands/config` - Configuration update commands
- `/commands/estop` - Software E-stop trigger commands

### Message Format Notes

#### BATT Messages
The TeensyV2 system now includes battery location information in BATT messages:
```
BATT:idx:0,V:42.51,A:1.15,charge:1.00,state:NORMAL,location:36VLIPO
```

The `location` field maps to the `location` field in the ROS2 `sensor_msgs/BatteryState` message, providing clear identification of which physical battery is reporting data.

## Services

### Configuration Services
- Configuration updates are handled via the messaging protocol
- Parameter changes are synchronized with the embedded system automatically
- No dedicated services - all configuration is parameter-driven

## Parameters

### Communication Parameters
- `serial_port` (string, default: "/dev/ttyACM0") - Serial port for Board 1
- `serial_port_board2` (string, default: "/dev/ttyACM1") - Serial port for Board 2  
- `baud_rate` (int, default: 921600) - Serial communication baud rate
- `reconnect_timeout` (double, default: 5.0) - Automatic reconnection timeout

### Safety Parameters
- `enable_auto_estop_recovery` (bool, default: true) - Enable automatic E-stop recovery
- `estop_recovery_delay` (double, default: 1.0) - Delay before automatic recovery
- `safety_check_frequency` (double, default: 10.0) - Safety monitoring frequency

### Battery Parameters
- `battery_critical_voltage` (double, default: 32.0) - Critical low voltage threshold
- `battery_warning_voltage` (double, default: 34.0) - Warning voltage threshold
- `battery_critical_current` (double, default: 15.0) - Critical high current threshold

### Performance Parameters
- `target_loop_frequency` (double, default: 85.0) - Target embedded system frequency
- `max_execution_time_ms` (double, default: 2.0) - Maximum module execution time
- `performance_report_interval` (double, default: 1.0) - Performance reporting interval

## Message Protocol

### TeensyV2 Message Format
All messages follow the format: `TYPE:key1=val1,key2=val2,...`

#### Message Types
- **BATT**: Battery status and measurements
- **PERF**: Performance metrics and timing data  
- **SAFETY**: Safety status and E-stop conditions
- **ESTOP**: E-stop trigger/clear notifications
- **CONFIG**: Configuration updates and responses
- **DIAG**: Diagnostic and error messages

#### Example Messages
```
BATT:id=0,v=39.8,c=1.2,p=47.8,pct=0.85,state=NORMAL
PERF:freq=84.2,exec_time=1.8,violations=0,modules=5
ESTOP:active=true,source=BATTERY,reason=low_voltage,value=31.2
SAFETY:state=NORMAL,hw_estop=false,conditions=0
```

## Development

### Adding New Message Types
1. Define message structure in `include/sigyn_to_sensor_v2/message_parser.h`
2. Implement parsing logic in `src/message_parser.cpp`
3. Add ROS2 topic publishing in appropriate node
4. Update documentation

### Adding New Parameters
1. Declare parameter in node constructor with default value
2. Add parameter callback for runtime updates
3. Implement parameter validation
4. Send parameter update to embedded system

### Testing
```bash
# Build with tests
colcon build --packages-select sigyn_to_sensor_v2 --cmake-args -DBUILD_TESTING=ON

# Run unit tests
colcon test --packages-select sigyn_to_sensor_v2

# Check test results
colcon test-result --verbose
```

## Troubleshooting

### Common Issues

#### Serial Communication Problems
```bash
# Check device permissions
ls -l /dev/ttyACM*
sudo usermod -a -G dialout $USER

# Monitor serial communication
ros2 topic echo /sigyn/diagnostics/serial_status
```

#### Parameter Issues
```bash
# Reset all parameters to defaults
ros2 service call /sigyn/config/reset_parameters std_srvs/Empty

# Check parameter validation
ros2 param describe /battery_monitor critical_low_voltage
```

#### Safety System Issues
```bash
# Check E-stop status
ros2 topic echo /sigyn/safety/estop_status

# Reset E-stop manually
ros2 service call /sigyn/safety/reset_estop sigyn_interfaces/ResetEstop "{source: 'ALL', force_reset: true}"
```

## Contributing

1. Follow ROS2 coding standards and Google C++ style guide
2. Add comprehensive unit tests for new functionality
3. Update documentation for any API changes
4. Test with both simulated and real hardware

## License

This package is part of the Sigyn robotics platform. See LICENSE file for details.
