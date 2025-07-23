# TeensyV2 - Real-Time Embedded Control System

## Overview

TeensyV2 is a professionally designed, real-time embedded control system for the Sigyn autonomous robot. Built on a modular architecture, it provides high-performance sensor monitoring, motor control, and comprehensive safety systems across multiple Teensy 4.1 microcontrollers.

The system follows modern embedded software engineering practices including:
- **Modular Architecture**: Clean separation of concerns with automatic module registration
- **Real-Time Performance**: Strict timing guarantees with microsecond-precision monitoring
- **Safety-First Design**: Multiple layers of protection with comprehensive E-stop coordination
- **Professional Code Quality**: Google C++ Style Guide compliance with extensive documentation
- **Build System Excellence**: Modern PlatformIO integration with full IDE support

## Key Features

### Real-Time Performance Monitoring
- Maintains 80-100Hz control loops with microsecond-precision timing
- Automatic detection of performance violations (>2ms execution time)
- Configurable thresholds for different operational modes
- Comprehensive diagnostic reporting for optimization

### Modular Architecture
- Clean separation of concerns with automatic module registration
- Singleton pattern enforcement for reliable resource management
- Easy addition of new functionality without architectural changes
- Zero-overhead abstractions where possible

### Global Safety Coordination
- Multi-layer E-stop system with hardware and software triggers
- Inter-board safety communication for system-wide protection
- Automatic recovery procedures when conditions improve
- Integration with ROS2 safety monitoring

### Efficient Communication
- Optimized message protocol for low-latency ROS2 integration
- Structured logging with configurable verbosity levels
- JSON-formatted diagnostic reports for external monitoring
- Minimal overhead design preserving real-time performance

### Configuration Management
- Runtime parameter updates via ROS2 parameter interface
- Persistent storage of critical settings in EEPROM
- Validation of parameter ranges to prevent unsafe configurations
- Multiple operational mode configurations

## Quick Start

### Prerequisites
- **PlatformIO IDE** (recommended) or Arduino IDE with Teensy support
- **ROS2 Jazzy** for high-level robot control and monitoring
- **Two Teensy 4.1 boards** with appropriate sensors and interface hardware
- **Hardware**: INA226 current sensors, IMU modules, motor controllers as required

### Compilation

The system uses PlatformIO for modern, reliable builds with dependency management:

```bash
# Navigate to TeensyV2 directory
cd TeensyV2

# Build individual boards (recommended for development)
/path/to/venv/bin/pio run -e board1  # Main controller board
/path/to/venv/bin/pio run -e board2  # Sensor/battery board

# Build both boards simultaneously (production)
/path/to/venv/bin/pio run

# Debug builds with additional instrumentation
/path/to/venv/bin/pio run -e board1_debug
/path/to/venv/bin/pio run -e board2_debug

# Clean build (recommended after major changes)
/path/to/venv/bin/pio run --target clean
/path/to/venv/bin/pio run
```

### Flash and Upload
```bash
# Upload to specific board
/path/to/venv/bin/pio run -e board1 --target upload
/path/to/venv/bin/pio run -e board2 --target upload

# Monitor serial output for debugging
/path/to/venv/bin/pio device monitor --baud 921600
```

### ROS2 Integration
```bash
# Build the ROS2 bridge package
cd /path/to/sigyn_ws
colcon build --packages-select sigyn_to_sensor_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Launch the Teensy communication bridge
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py

# Monitor system status
ros2 topic echo /robot/battery_status
ros2 topic echo /robot/emergency_stop
ros2 topic echo /robot/performance_stats
```

### Initial System Verification
```bash
# Check that both boards are communicating
ros2 topic list | grep robot

# Verify performance monitoring
ros2 topic echo /robot/performance_stats

# Test emergency stop system
ros2 service call /robot/test_estop std_srvs/srv/Trigger
```

## Architecture

### System Overview
The TeensyV2 architecture implements a distributed control system with two specialized microcontroller boards, each optimized for specific functions while maintaining tight coordination for safety and performance.

### Board Assignments
- **Board 1 (Main Controller)**: 
  - Motor control and odometry processing
  - VL53L0X time-of-flight sensor management
  - RoboClaw motor driver interface
  - Primary E-stop coordination and safety management
  - High-priority ROS2 communication (navigation commands, odometry)
  - Target frequency: 85Hz for precise motor control

- **Board 2 (Sensor & Power Board)**: 
  - Battery monitoring with INA226 current sensors
  - IMU sensor data collection and filtering
  - Environmental sensors (temperature, humidity)
  - Secondary safety monitoring and backup E-stop
  - Lower-priority ROS2 communication (status, diagnostics)
  - Target frequency: 80Hz for sensor data collection

### Core Components

#### Module System
The Module base class provides automatic registration and lifecycle management:
- **Automatic Registration**: Modules self-register during static initialization
- **Unified Execution**: All modules execute through centralized `loopAll()` method
- **Performance Monitoring**: Built-in timing measurement with violation detection
- **Safety Integration**: Global safety state aggregation across all modules
- **Resource Management**: RAII patterns prevent resource leaks

#### Performance Monitor
Dedicated monitoring module ensuring real-time constraints:
- **Timing Analysis**: Microsecond-precision execution time tracking
- **Violation Detection**: Configurable thresholds for safety responses
- **Trend Analysis**: Historical performance data for predictive maintenance
- **Adaptive Thresholds**: Different limits for various operational modes
- **Diagnostic Reporting**: JSON-formatted reports for external analysis

#### Safety Coordinator
Comprehensive safety management with multiple protection layers:
- **Hardware E-Stop**: Physical button integration with immediate response
- **Software E-Stop**: Programmatic triggers from performance or sensor violations
- **Inter-Board Communication**: Safety state sharing between controllers
- **Automatic Recovery**: Intelligent restoration when conditions improve
- **Escalation Procedures**: Progressive responses from warnings to emergency shutdown

#### Serial Manager
Optimized communication system for ROS2 integration:
- **Structured Messaging**: Parseable message format for reliable communication
- **Priority Management**: Critical messages bypass normal queuing
- **Error Recovery**: Automatic reconnection and message retry
- **Flow Control**: Prevents buffer overflow under high message loads
- **Debugging Support**: Configurable logging levels for development

#### Configuration Manager
Runtime parameter management without firmware updates:
- **ROS2 Integration**: Dynamic parameter updates via ROS2 parameter server
- **Validation**: Range checking and consistency validation for all parameters
- **Persistence**: Critical parameters stored in EEPROM for power-cycle retention
- **Versioning**: Configuration schema versioning for backward compatibility
- **Rollback**: Automatic restoration of previous values on validation failure

## Directory Structure

```
TeensyV2/
├── README.md                   # This file
├── docs/                       # Architecture and design documentation
│   ├── Architecture.md         # Detailed system architecture
│   ├── MessageProtocol.md      # Communication protocol specification
│   ├── SafetySystem.md         # E-stop and safety architecture
│   └── ModuleDevelopment.md    # Guide for creating new modules
├── common/                     # Shared code across all boards
│   ├── core/                   # Core framework (Module, SerialManager, etc.)
│   ├── safety/                 # Safety and E-stop coordination
│   ├── utils/                  # Utility functions and helpers
│   └── config/                 # Configuration management
├── modules/                    # Sensor and device modules
│   ├── battery/                # Battery monitoring (INA226, analog)
│   ├── imu/                    # BNO055 IMU sensors
│   ├── motor/                  # RoboClaw motor control
│   ├── sensors/                # VL53L0X, SONAR, temperature
│   └── performance/            # Performance monitoring
├── src/                        # Board-specific main programs
│   ├── board1_main.cpp         # Main program for board 1 (navigation/safety)
│   └── board2_main.cpp         # Main program for board 2 (power/sensors)
├── docs/                       # Documentation
├── platformio.ini              # PlatformIO configuration for both boards
└── .gitignore                  # Git ignore patterns
```

## Message Protocol

All messages follow a compact, parseable format:
```
TYPE:key1=val1,key2=val2,...
```

Examples:
- `BATT:id=0,v=39.8,p=0.82,c=1.2,s=OK`
- `IMU:id=0,qx=0.1,qy=0.2,qz=0.3,qw=0.9`
- `ESTOP:src=motor,state=active,reason=overcurrent`
- `TEMP:id=0,temp=23.5,status=OK` (Temperature sensor data)

### Performance (PERF) JSON Format

The performance monitoring system outputs detailed JSON reports for real-time system analysis. Field names are abbreviated to reduce bandwidth and prevent buffer overflow:

**Top-Level Fields:**
- `freq`: Current loop frequency in Hz
- `tfreq`: Target minimum loop frequency threshold in Hz  
- `mviol`: Total count of module timing violations since startup
- `fviol`: Total count of frequency violations since startup

**Violation Details (violdet) - only present when violations are active:**
- `cmod`: Consecutive module timing violations count
- `cfreq`: Consecutive frequency violations count  
- `lastms`: Timestamp (milliseconds) of last violation

**Module Performance Array (mods) - one entry per registered module:**
- `n`: Module name (string)
- `min`: Minimum execution time in milliseconds since startup
- `max`: Maximum execution time in milliseconds since startup
- `avg`: Average execution time in milliseconds since startup
- `last`: Last execution time in milliseconds
- `cnt`: Total execution count since startup
- `viol`: Violation status - "T" (true) if last execution exceeded threshold, "F" (false) otherwise

**Example PERF Message:**
```json
PERF:{"freq":85.2, "tfreq":75.0, "mviol":0, "fviol":0, "mods":[
  {"n":"PerformanceMonitor","min":0.01,"max":0.15,"avg":0.05,"last":0.06,"cnt":12543,"viol":"F"},
  {"n":"BatteryMonitor","min":0.20,"max":1.85,"avg":0.75,"last":0.82,"cnt":12543,"viol":"F"},
  {"n":"RoboClawMonitor","min":2.10,"max":8.45,"avg":4.25,"last":3.80,"cnt":12543,"viol":"F"}
]}
```

**Temperature Sensor Messages:**
Temperature sensors now use the standardized message format:
- `TEMP:id=<sensor_id>,temp=<temperature_celsius>,status=<OK|ERROR>`
- Example: `TEMP:id=0,temp=23.5,status=OK`
- Example: `TEMP:id=1,temp=45.2,status=ERROR`

## Performance Requirements

### Real-Time Constraints
The TeensyV2 system maintains strict real-time performance requirements to ensure reliable robot operation:

**Loop Frequency Targets:**
- **Board 1**: 85Hz (11.76ms period) for motor control precision
- **Board 2**: 80Hz (12.5ms period) for sensor data collection
- **Tolerance**: ±5Hz acceptable, >10Hz deviation triggers warnings

**Module Execution Limits:**
- **Individual Module**: Maximum 2ms execution time per loop iteration
- **Total System**: Maximum 8ms total execution time per loop
- **Safety Margin**: 4ms reserved for interrupt handling and system overhead

**Memory Constraints:**
- **Flash Usage**: <100KB per board (leaving 7.9MB available for future expansion)
- **RAM Usage**: <100KB per board (leaving 424KB available for buffers and processing)
- **Stack Depth**: <4KB maximum to prevent stack overflow

**Communication Performance:**
- **Serial Baud Rate**: 921600 bps for low-latency ROS2 communication
- **Message Latency**: <5ms from sensor reading to ROS2 publication
- **Throughput**: >200 messages/second sustained without buffer overflow

### Performance Monitoring
Continuous monitoring ensures system health and early problem detection:

**Automatic Violation Detection:**
- Module execution time exceeding 2ms threshold
- Loop frequency dropping below 75Hz for more than 3 consecutive cycles
- Memory usage approaching 80% of available capacity
- Communication errors or timeouts

**Escalation Procedures:**
1. **Warning Level**: Log message, increase monitoring frequency
2. **Error Level**: Reduce non-critical functionality, notify ROS2 system
3. **Critical Level**: Trigger controlled shutdown, activate E-stop if necessary

## Safety System

### Multi-Layer Protection
The safety system implements defense-in-depth principles with multiple independent protection layers:

#### Layer 1: Hardware E-Stop
- **Physical Button**: Immediate hardware-level motor disconnection
- **Response Time**: <1ms from button press to motor shutdown
- **Failsafe Design**: Normally-closed contacts, fail-safe on power loss
- **Redundancy**: Multiple E-stop points throughout robot chassis

#### Layer 2: Software E-Stop
- **Performance Monitoring**: Automatic trigger on timing violations
- **Sensor Monitoring**: Battery voltage, current, temperature thresholds
- **Communication Loss**: Automatic E-stop on ROS2 communication timeout
- **Response Time**: <10ms from detection to safety response

#### Layer 3: Inter-Board Safety
- **Cross-Monitoring**: Each board monitors the other's health status
- **Heartbeat Protocol**: Regular "alive" signals between boards
- **Isolation Capability**: Either board can independently trigger system shutdown
- **Response Time**: <50ms for inter-board safety communication

#### Layer 4: ROS2 Safety Integration
- **High-Level Monitoring**: Navigation system safety checks
- **Operator Override**: Manual E-stop from operator console
- **Mission Abort**: Automatic abort on navigation failures
- **Response Time**: <100ms for ROS2-initiated safety responses

### Safety Thresholds

**Battery Safety:**
- Critical Low Voltage: 32.0V (immediate E-stop)
- Warning Low Voltage: 34.0V (return-to-base procedure)
- High Current: 20A (immediate E-stop, indicates short circuit or stall)
- Temperature: >60°C (thermal protection)

**Performance Safety:**
- Module Timeout: >2ms execution time (indicates runaway code)
- System Frequency: <75Hz for >3 cycles (indicates system overload)
- Memory Usage: >90% (indicates memory leak or excessive allocation)

**Communication Safety:**
- ROS2 Timeout: >500ms without high-level commands
- Inter-Board Timeout: >100ms without heartbeat signal
- Sensor Timeout: >200ms without critical sensor data

### Emergency Procedures

**Automatic Recovery:**
- E-stop conditions automatically reset when underlying problems resolve
- Graceful system restart without manual intervention when safe
- Progressive re-enablement of functionality based on system health
- Operator notification and approval required for critical system recovery

**Manual Override:**
- Physical reset button for emergency recovery
- ROS2 service calls for controlled system restart
- Diagnostic mode for troubleshooting without full system activation
- Maintenance mode with safety systems active but operational systems disabled

## Development

### Adding New Modules
1. Inherit from `Module` base class
2. Implement required virtual methods
3. Register via singleton pattern
4. See `docs/ModuleDevelopment.md` for details

### Configuration
- Default values in `config.h` files
- Runtime updates via ROS2 parameters
- Persistent storage in EEPROM for critical values

### Testing
- Unit tests for critical safety functions
- Hardware-in-the-loop testing framework
- Performance benchmarking tools

## Related Packages

- **sigyn_to_sensor_v2**: ROS2 bridge for communication with Teensy boards
- **sigyn_interfaces**: Message and service definitions
- **sigyn_safety**: High-level safety monitoring and response

## Support

For issues, questions, or contributions:
1. Check existing documentation in `docs/`
2. Review module-specific README files
3. Create issue with detailed description and logs

## License

[Specify your license here]
