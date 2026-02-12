# sigyn_to_sensor_v2

Advanced ROS2 Jazzy bridge package for the TeensyV2 embedded system, providing real-time sensor data processing, enhanced message parsing, safety coordination, and comprehensive diagnostic reporting for the Sigyn autonomous house patroller robot.

## Overview

This package serves as the critical communication bridge between the TeensyV2 embedded system (dual Teensy 4.1 boards) and the ROS2 Jazzy ecosystem. It provides:

- **Enhanced Message Parsing**: Support for board-identified messages (IMU1/IMU2, BATT1/BATT2, etc.)
- **Real-time Communication**: High-speed serial communication at 921600 baud
- **Safety Coordination**: Multi-source E-stop management and safety status monitoring  
- **Sensor Integration**: Battery monitoring, dual IMU sensors, performance tracking, and odometry
- **Diagnostic System**: Comprehensive ROS2 diagnostics with enhanced string representation
- **JSON Payload Extraction**: Advanced parsing for structured data fields

## Key Features & Recent Enhancements

### Message Parser Improvements
- **Board-Specific Message Types**: Recognizes IMU1/IMU2 as IMU message type variants
- **Enhanced ODOM Parsing**: Extracts position (px,py), orientation (ox,oy,oz,ow), and velocity (vx,vy,wz) fields
- **Diagnostic String Support**: Preserves both enum and string representations for diagnostic level display
- **JSON Field Extraction**: Robust parsing of nested JSON payloads from TeensyV2 boards

### Dual IMU Support
- **Primary IMU**: `/imu/sensor_0` from sensor_0 messages
- **Secondary IMU**: `/imu/sensor_1` from sensor_1 messages  
- **Board Identification**: Automatic routing based on board ID in message headers
- **Synchronized Publishing**: Coordinated publishing from both embedded boards

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
cd /home/ros/sigyn_ws
colcon build --packages-select sigyn_to_sensor_v2 --symlink-install
source install/setup.bash
```

### Running the Bridge
```bash
# Launch the complete TeensyV2 interface bridge
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py

# Or run the main bridge node directly  
ros2 run sigyn_to_sensor_v2 teensy_bridge_node

# Monitor all TeensyV2-related topics
ros2 topic list | grep -E "(battery|imu|estop|performance|gripper)"
```

### Verification and Testing
```bash
# Verify dual IMU sensor data
ros2 topic echo /imu/sensor_0 --once
ros2 topic echo /imu/sensor_1 --once

# Check battery monitoring with location info
ros2 topic echo /battery_state --once

# Monitor odometry data (high-frequency)
ros2 topic hz /odom

# View comprehensive diagnostics
ros2 topic echo /diagnostics --once

# Monitor gripper status (10 Hz auto-publishing)
ros2 topic echo /gripper/status
ros2 topic hz /gripper/status  # Should show ~10 Hz

# Test gripper positioning
ros2 topic pub --once /gripper/home std_msgs/msg/Empty  # Home first
ros2 topic pub --once /gripper/position/command sigyn_interfaces/msg/GripperPositionCommand "{elevator_position: 0.1, extender_position: 0.05}"

# Test gripper actions with feedback
ros2 action send_goal /gripper/move_elevator sigyn_interfaces/action/MoveElevator "{goal_position: 0.5}" --feedback
ros2 action send_goal /gripper/move_extender sigyn_interfaces/action/MoveExtender "{goal_position: 0.2}" --feedback
```

### Configuration
```bash
# View available parameters
ros2 param list | grep -E "(battery|performance|serial)"

# Update battery monitoring thresholds
ros2 param set /teensy_bridge_node battery_critical_voltage 30.0
ros2 param set /teensy_bridge_node battery_warning_voltage 32.0

# Configure performance monitoring  
ros2 param set /teensy_bridge_node target_loop_frequency 85.0
ros2 param set /teensy_bridge_node max_execution_time_ms 2.0

# Update serial communication settings
ros2 param set /teensy_bridge_node serial_port "/dev/ttyACM0"
ros2 param set /teensy_bridge_node serial_port_board2 "/dev/ttyACM1"
```

## Topics

### Published Topics

#### Battery Monitoring
- `/battery_state` (sensor_msgs/BatteryState) - Enhanced battery monitoring with location identification
  - **Board Support**: Receives BATT1/BATT2 messages from both TeensyV2 boards
  - **Location Field**: Physical battery location (e.g., "36VLIPO", "5VDCDC", "24VDCDC")
  - **Multi-Battery**: Supports main 36V LiPo, 5V DC-DC, 24V DC-DC, 3.3V DC-DC, 12V DC-DC

#### IMU Data (Enhanced Dual Sensor Support)
- `/imu/sensor_0` (sensor_msgs/Imu) - Primary IMU sensor (from sensor_0 messages)
- `/imu/sensor_1` (sensor_msgs/Imu) - Secondary IMU sensor (from sensor_1 messages)
  - **Board Identification**: Automatic message routing based on IMU1/IMU2 message types
  - **Quaternion Orientation**: Full orientation data from BNO055 sensors
  - **Synchronized Updates**: Coordinated sensor readings from both embedded boards

#### Odometry Data (High-Frequency)
- `/odom` (nav_msgs/Odometry) - High-frequency robot odometry (≥70Hz)
  - **Enhanced Parsing**: Extracts position (px,py), orientation (ox,oy,oz,ow), velocity (vx,vy,wz)
  - **Real-time Updates**: Low-latency odometry for precise navigation
  - **Source**: ODOM1 messages from TeensyV2 Board 1 (main controller)

#### Safety and Diagnostics
- `/estop_status` (diagnostic_msgs/DiagnosticArray) - Emergency stop events and safety status
- `/diagnostics` (diagnostic_msgs/DiagnosticArray) - Enhanced system diagnostic messages
  - **String Preservation**: Includes both enum values and human-readable diagnostic strings
  - **Board Identification**: Diagnostics tagged with source board information
  - **Level Display**: Enhanced level representation for diagnostic viewers
- `/performance_stats` (diagnostic_msgs/DiagnosticArray) - Real-time performance monitoring and timing analysis

#### Gripper Control (Board 3)
- `/gripper/status` (sigyn_interfaces/GripperStatus) - **10 Hz auto-published** gripper position and status
  - **Position Feedback**: Current elevator and extender positions in meters
  - **Limit States**: Hardware limit switch states (none/upper/lower/both)
  - **Motion Status**: is_moving flag based on position changes between updates
  - **Homed Status**: is_homed flag indicating if motors have been homed
  - **Hardware Limits**: Maximum travel distances for both axes

### Subscribed Topics

#### Commands
- `/commands/config` - Configuration update commands
- `/commands/estop` - Software E-stop trigger commands
- `/cmd_vel_gripper` (geometry_msgs/Twist) - Velocity-based gripper control (legacy)
  - `linear.x`: Elevator velocity (0.001m increments)
  - `angular.z`: Extender velocity (0.001m increments)

#### Gripper Position Control
- `/gripper/position/command` (sigyn_interfaces/GripperPositionCommand) - Position-based gripper control
  - **elevator_position** (float): Target elevator position in meters (0.0-0.8999m, -1.0 = no change)
  - **extender_position** (float): Target extender position in meters (0.0-0.3418m, -1.0 = no change)
- `/gripper/home` (std_msgs/Empty) - Initiate non-blocking homing sequence
  - Retracts extender first, then lowers elevator
  - Status continues publishing at 10 Hz during homing

### Action Servers

#### Gripper Position Actions
- `/gripper/move_elevator` (sigyn_interfaces/action/MoveElevator) - Goal-based elevator positioning
  - **Goal**: Target position in meters (0.0-0.8999m)
  - **Feedback**: Current position at 10 Hz
  - **Result**: Final position when complete
  - **Success Tolerance**: ±5mm
  - **Timeout**: 30 seconds
- `/gripper/move_extender` (sigyn_interfaces/action/MoveExtender) - Goal-based extender positioning
  - **Goal**: Target position in meters (0.0-0.3418m)
  - **Feedback**: Current position at 10 Hz  
  - **Result**: Final position when complete
  - **Success Tolerance**: ±5mm
  - **Timeout**: 30 seconds

### Message Format Notes

### Message Format Notes

#### Enhanced Message Protocol
The TeensyV2 system now uses board-identified messages for proper routing and debugging:

**Battery Messages (BATT1/BATT2):**
```
BATT1:id=0,v=42.51,c=1.15,p=47.8,pct=0.85,state=NORMAL,location=36VLIPO
BATT2:id=1,v=5.05,c=0.25,p=1.26,pct=1.0,state=NORMAL,location=5VDCDC
```

**IMU Messages (IMU1/IMU2):**
```
IMU1:{"qx":0.123,"qy":0.456,"qz":0.789,"qw":0.987,"status":"OK"}
IMU2:{"qx":0.124,"qy":0.457,"qz":0.788,"qw":0.986,"status":"OK"}
```

**Odometry Messages (ODOM1):**
```
ODOM1:px=1.234,py=5.678,ox=0.0,oy=0.0,oz=0.707,ow=0.707,vx=0.5,vy=0.0,wz=0.1
```

The `location` field in battery messages maps directly to the `location` field in the ROS2 `sensor_msgs/BatteryState` message, providing clear identification of which physical battery is reporting data.

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
All messages follow the enhanced format with board identification: `TYPEX:payload`
Where `X` is the board identifier (1 or 2).

#### Message Types
- **BATT1/BATT2**: Battery status and measurements from specific boards
- **IMU1/IMU2**: IMU sensor data from specific sensors  
- **ODOM1**: High-frequency odometry data from main controller
- **PERF1/PERF2**: Performance metrics and timing data from each board
- **ESTOP1/ESTOP2**: E-stop events from specific boards
- **TEMP1/TEMP2**: Temperature sensor readings
- **ROBOCLAW1**: Motor controller status and encoder data

#### Example Enhanced Messages
```
BATT1:id=0,v=39.8,c=1.2,p=47.8,pct=0.85,state=NORMAL,location=36VLIPO
IMU2:{"qx":0.123,"qy":0.456,"qz":0.789,"qw":0.987,"status":"OK"}
ODOM1:px=1.234,py=5.678,ox=0.0,oy=0.0,oz=0.707,ow=0.707,vx=0.5,vy=0.0,wz=0.1
PERF2:{"freq":80.2,"exec_time":1.8,"violations":0,"modules":5}
ESTOP1:src=battery,state=active,reason=low_voltage,value=31.2
TEMP1:id=0,temp=23.5,status=OK
```

## Development

### Adding New Message Types
1. Define message structure in `include/sigyn_to_sensor_v2/message_parser.h`
2. Add parsing logic in `StringToMessageType()` function in `src/message_parser.cpp`
3. Implement field extraction in `ParseJsonPayload()` for JSON messages
4. Add ROS2 topic publishing in teensy_bridge_node
5. Update documentation and tests

### Message Parser Enhancements
The message parser now supports:
- **Board-Specific Types**: IMU1/IMU2 automatically recognized as IMU message type
- **Enhanced ODOM Parsing**: Extracts px,py,ox,oy,oz,ow,vx,vy,wz fields from ODOM messages
- **Diagnostic String Preservation**: Maintains both enum and string representations for diagnostic displays
- **JSON Field Extraction**: Robust parsing of nested JSON payloads from structured messages

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
# Check device permissions and connectivity
ls -l /dev/ttyACM*
sudo usermod -a -G dialout $USER

# Monitor serial communication health
ros2 topic echo /diagnostics | grep -i serial

# Test individual board connections
ros2 param get /teensy_bridge_node serial_port
ros2 param get /teensy_bridge_node serial_port_board2
```

#### IMU Sensor Issues
```bash
# Verify both IMU sensors are publishing
ros2 topic list | grep imu
ros2 topic hz /imu/sensor_0
ros2 topic hz /imu/sensor_1

# Check for IMU data quality
ros2 topic echo /imu/sensor_0 --once
ros2 topic echo /imu/sensor_1 --once

# Monitor IMU-related diagnostics
ros2 topic echo /diagnostics | grep -i imu
```

#### Message Parsing Issues
```bash
# Monitor message parser diagnostics
ros2 topic echo /diagnostics | grep -i parser

# Check for ODOM data completeness
ros2 topic echo /odom --once

# Verify enhanced diagnostics with string levels
rqt_robot_monitor  # GUI diagnostic viewer
```

#### Parameter Issues
```bash
# List all available parameters
ros2 param list | grep teensy_bridge

# Reset specific parameters to defaults
ros2 param set /teensy_bridge_node battery_critical_voltage 32.0
ros2 param set /teensy_bridge_node target_loop_frequency 85.0

# Check parameter validation
ros2 param describe /teensy_bridge_node battery_critical_voltage
```

#### Safety System Issues
```bash
# Check E-stop status from both boards
ros2 topic echo /estop_status --once

# Monitor safety-related diagnostics
ros2 topic echo /diagnostics | grep -i estop

# View comprehensive safety status
ros2 topic echo /diagnostics | grep -i safety
```

## Recent Enhancements (2024-2025)

### Message Parser Improvements
- **Board-Specific Recognition**: Enhanced `StringToMessageType()` to recognize IMU1/IMU2 as IMU message variants
- **ODOM Field Extraction**: Added comprehensive odometry field parsing (px,py,ox,oy,oz,ow,vx,vy,wz)
- **Diagnostic String Preservation**: Enhanced `ToDiagnosticArrayMsg()` to include both enum and string level representations
- **Robust JSON Parsing**: Improved `ParseJsonPayload()` function for structured data extraction

### Dual IMU Support
- **Synchronized Publishing**: Coordinated IMU sensor data from both TeensyV2 boards
- **Automatic Routing**: Message routing based on board identification in message headers
- **Enhanced Topics**: Separated `/imu/sensor_0` and `/imu/sensor_1` for independent sensor monitoring

### Communication Enhancements  
- **Enhanced Serial Reliability**: Improved error handling and reconnection logic
- **Board-Identified Messages**: Support for BATT1/BATT2, IMU1/IMU2, PERF1/PERF2 message routing
- **High-Frequency Odometry**: Optimized ODOM1 message processing for ≥70Hz updates

### Diagnostic System Improvements
- **RQT Compatibility**: Enhanced diagnostic messages for improved GUI display compatibility
- **String Level Display**: Diagnostic level strings now properly displayed in diagnostic viewers
- **Comprehensive Monitoring**: Enhanced system health monitoring and reporting

## Contributing

1. Follow ROS2 Jazzy coding standards and Google C++ style guide
2. Add comprehensive unit tests for new message parsing functionality
3. Update documentation for any message protocol or API changes
4. Test with both simulated and real TeensyV2 hardware
5. Ensure real-time performance requirements are maintained for critical data streams

## License

This package is part of the Sigyn autonomous house patroller robotic platform. See LICENSE file for details.
