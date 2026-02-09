# TeensyV2 - Advanced Robotic Control System

High-performance multiple-board embedded system for real-time robotic control, implementing distributed safety monitoring, precise odometry, and optimized sensor fusion for the Sigyn autonomous house patroller robot.

## Overview

TeensyV2 represents a complete redesign of the embedded control architecture, featuring multiple Teensy 4.1 microcontrollers working in coordination to provide:

- **Real-time Performance Monitoring**: Microsecond-precision timing analysis for deterministic system behavior
- **Safety-First Architecture**: Multi-layer emergency stop system with automatic recovery capabilities  
- **High-Frequency Odometry**: Precise robot localization with ≥70Hz encoder-based position tracking
- **Modular Framework**: Extensible component system with automatic registration and lifecycle management
- **Power Management**: Advanced battery monitoring with current sensing and predictive analysis
- **Sensor Fusion**: Unified sensor data collection and processing across multiple I²C devices
- **Dual IMU Support**: BNO055 sensors with staggered reporting for redundant orientation data

## Project Status
- **Current Version**: 2.0 (Major architecture revision)  
- **Development Status**: Beta / Active Development
- **Hardware Platform**: Multiple Teensy 4.1 boards with specialized sensor arrays
- **Integration**: Full ROS2 Jazzy compatibility via sigyn_to_sensor_v2 bridge
- **Message Protocol**: Enhanced JSON-based communication with board-specific identifiers

## Documentation Index

**Start here for comprehensive system understanding:**

| Document | Purpose | Audience |
|----------|---------|----------|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | System design, module overview, performance metrics | All developers |
| [Message_Formats.md](docs/Message_Formats.md) | Serial protocol, JSON message structure, examples | Integration engineers, debugging |
| [Safety_System.md](docs/Safety_System.md) | Fault detection, E-stop logic, recovery mechanisms | Safety-critical developers |
| [Module_Reference.md](docs/Module_Reference.md) | Per-module parameters, thresholds, fault codes | Module maintainers |
| [Testing.md](docs/Testing.md) | Unit tests, mocks, debugging guide, troubleshooting | QA, developers |

**For quick answers:**
- "How does the system work?" → Read [ARCHITECTURE.md](docs/ARCHITECTURE.md)
- "What messages does Teensy send?" → Read [Message_Formats.md](docs/Message_Formats.md)
- "Motor stopped, why?" → Check [Safety_System.md](docs/Safety_System.md) fault codes
- "RoboClaw current threshold?" → See [Module_Reference.md](docs/Module_Reference.md)
- "How do I debug?" → Use [Testing.md](docs/Testing.md)

## Quick Start

### Prerequisites
```bash
# Install PlatformIO Core
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py -o get-platformio.py
python3 get-platformio.py

# Create Python virtual environment (recommended)
python3 -m venv ~/.platformio/venv
source ~/.platformio/venv/bin/activate
```

### System Architecture Overview

TeensyV2 provides high-performance sensor monitoring, motor control, and comprehensive safety systems across multiple Teensy 4.1 microcontrollers. The system follows modern embedded software engineering practices including:

- **Modular Architecture**: Clean separation of concerns with automatic module registration
- **Real-Time Performance**: Strict timing guarantees with microsecond-precision monitoring
- **Safety-First Design**: Multiple layers of protection with comprehensive E-stop coordination
- **Professional Code Quality**: Google C++ Style Guide compliance with extensive documentation
- **Build System Excellence**: Modern PlatformIO integration with full IDE support

## Build System

### Prerequisites
- **PlatformIO IDE** (recommended) or Arduino IDE with Teensy support
- **ROS2 Jazzy** for high-level robot control and monitoring
- **Three Teensy 4.1 boards** (board1 navigation/safety, board2 power/sensors, board3 elevator)
- **Hardware**: INA226 voltage and current sensors, IMU modules, motor controllers as required

### Compilation

The system uses PlatformIO for modern, reliable builds with dependency management:

```bash
# Navigate to TeensyV2 directory
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2

# Build individual boards (recommended for development)
~/.platformio/venv/bin/pio run -e board1          # Navigation/Safety
~/.platformio/venv/bin/pio run -e board2          # Power/Sensors
~/.platformio/venv/bin/pio run -e elevator_board  # Elevator/Gripper

# Build all boards (production)
~/.platformio/venv/bin/pio run

# Debug builds with additional instrumentation
~/.platformio/venv/bin/pio run -e board1_debug
~/.platformio/venv/bin/pio run -e board2_debug

# Clean build (recommended after major changes)
~/.platformio/venv/bin/pio run --target clean
~/.platformio/venv/bin/pio run

# Upload to specific board
~/.platformio/venv/bin/pio run -e board1 --target upload
~/.platformio/venv/bin/pio run -e board2 --target upload
~/.platformio/venv/bin/pio run -e elevator_board --target upload

# Monitor serial output for debugging
~/.platformio/venv/bin/pio device monitor --baud 921600
```

## Key Features

### Real-Time Performance Monitoring
- Enforces per-board minimum loop rates (Board1 50 Hz, Board2 20 Hz, Board3 10 Hz from `config.h`) and runs faster when workload allows (logs show ~298 Hz when light)
- High-frequency odometry from RoboClaw encoders (critical path ~67 Hz cadence)
- Per-module execution limits match board configs (e.g., 2/3/5 ms max module times)
- PerformanceMonitor reports diagnostics; it does not assert E-stop

### Modular Architecture
- Clean separation of concerns with automatic module registration
- Singleton pattern enforcement for reliable resource management
- Easy addition of new functionality without architectural changes
- Zero-overhead abstractions where possible

### Global Safety Coordination
- SafetyCoordinator aggregates faults with severities; E-stop asserted via RoboClaw line when compiled with control pin
- Fault sources include RoboClaw overcurrent/runaway/error bits, battery critical thresholds (32/34 V, >20 A), temperature limits, and explicit software E-stop commands
- Recovery is caller-driven; faults can self heal when the condition recovers or can clear when modules request `deactivateFault` or host triggers `resetAllSafetyFlags`
- ROS2 integration via sigyn_to_sensor_v2 exposes FAULT/DIAG streams

### Efficient Communication
- Optimized message protocol for low-latency ROS2 integration
- Structured logging with configurable verbosity levels
- JSON-formatted diagnostic reports for external monitoring
- Minimal overhead design preserving real-time performance

### Configuration Management
- Runtime parameter updates via ROS2 parameter interface
- Validation of parameter ranges to prevent unsafe configurations
- Multiple operational mode configurations

## Quick Start

### Building and Uploading

```bash
# Navigate to TeensyV2 directory
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2

# Build individual boards (recommended for development)
~/.platformio/venv/bin/pio run -e board1  # Main controller board
~/.platformio/venv/bin/pio run -e board2  # Sensor/battery board

# Upload to specific board
~/.platformio/venv/bin/pio run -e board1 --target upload
~/.platformio/venv/bin/pio run -e board2 --target upload

# Monitor serial output for debugging
~/.platformio/venv/bin/pio device monitor --baud 921600
```

### ROS2 Integration
```bash
# Build the ROS2 bridge package
cd /home/ros/sigyn_ws
colcon build --packages-select sigyn_to_sensor_v2 --symlink-install

# Source the workspace
source install/setup.bash

# Launch the Teensy communication bridge
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py

# Monitor system status
ros2 topic echo /battery_state
ros2 topic echo /estop_status
ros2 topic echo /performance_stats
ros2 topic echo /imu/sensor_0
ros2 topic echo /imu/sensor_1
```

### Initial System Verification
```bash
# Check that both boards are communicating
ros2 topic list | grep -E "(battery|imu|estop|performance)"

# Verify IMU sensors
ros2 topic echo /imu/sensor_0 --once
ros2 topic echo /imu/sensor_1 --once

# Check battery monitoring
ros2 topic echo /battery_state --once

# Verify performance monitoring
ros2 topic echo /performance_stats --once

# Test emergency stop system
ros2 topic echo /estop_status --once
```

## Architecture

### System Overview
The TeensyV2 architecture implements a distributed control system with three specialized microcontroller boards, each optimized for specific functions while maintaining tight coordination for safety and performance.

### Board Assignments
- **Board 1 (Main Controller)**: 
  - **High-frequency odometry processing (≥70Hz)** for precise robot localization
  - **Real-time motor control with optimized cmd_vel handling** for maximum responsiveness  
  - VL53L0X time-of-flight sensor management with non-blocking reads
  - Motor temperature monitoring with fail-safe limits
  - RoboClaw motor driver interface with separated frequency tiers:
    - Critical operations (encoder reads + odometry): 70Hz
    - Motor status monitoring: 30Hz  
    - System diagnostics: 3Hz
  - Primary E-stop coordination and safety management
  - High-priority ROS2 communication (navigation commands, odometry)
  - Target frequency: 85Hz for precise motor control

- **Board 2 (Sensor & Power Board)**: 
  - Battery monitoring with INA226 voltage and current sensors
  - IMU sensor data collection and filtering
  - Secondary safety monitoring and backup E-stop
  - Lower-priority ROS2 communication (status, diagnostics)
  - Target frequency: 80Hz for sensor data collection

- **Board 3 (Elevator & Gripper Board)**:
  - Stepper motor control (elevator and extender positioning)
  - Position-based control with absolute position commands
  - Velocity-based control for legacy compatibility (TWIST)
  - Homing sequence with hardware limit switch detection
  - Real-time position feedback and limit switch status
  - Hardware measured travel limits (elevator: 0.8999m, extender: 0.3418m)
  - Target frequency: 100+Hz for smooth motion control

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

#### SerialManager
Optimized communication system for ROS2 integration:
- **Structured Messaging**: Parseable message format for reliable communication
- **Priority Management**: Critical messages bypass normal queuing
- **Error Recovery**: Automatic reconnection and message retry
- **Flow Control**: Prevents buffer overflow under high message loads
- **Debugging Support**: Configurable logging levels for development

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
├── modules/                    # Sensor and device modules
│   ├── battery/                # Battery monitoring (INA226, analog)
│   ├── imu/                    # BNO055 IMU sensors
│   ├── roboclaw/               # RoboClaw motor control with high-frequency odometry
│   ├── sensors/                # VL53L0X, temperature sensors  
│   ├── safety/                 # Safety coordinator
│   └── performance/            # Performance monitoring
├── platform/                   # Board-specific main programs
│   ├── board1/                 # Main program for board 1 (navigation/safety)
│   └── board2/                 # Main program for board 2 (power/sensors)
│   └── elevatorBoard/          # Gripper
├── platformio.ini              # PlatformIO configuration for all boards
├── library.json                # PlatformIO library metadata
```

## Message Protocol

### Current Message Format

All messages follow the enhanced protocol with board identification:
```
TYPEX:JSON_PAYLOAD
```
Where `X` is the board identifier (1, 2 OR 3), enabling proper message routing and debugging.

### Message Types in Production

Based on actual system operation, the following message types are currently active:

**Board 1 (Main Controller) Messages:**
- **ODOM1**: High-frequency odometry data (JSON format)
- **ROBOCLAW1**: Motor controller status and encoder data (JSON format)  
- **TEMPERATURE1**: Temperature sensor array data (JSON format)
- **DIAG1**: Diagnostic messages from all modules (JSON format)
- **PERF1**: Performance monitoring and timing statistics (JSON format)
- **VL53L0X1**: Time-of-flight sensors (JSON format)

**Board 2 (Sensor & Power Board) Messages:**
- **IMU2**: Dual IMU sensor data with sensor ID (JSON format)
- **BATT2**: Multi-battery monitoring data (JSON format)
- **DIAG2**: Diagnostic messages from sensor modules (JSON format)
- **PERF2**: Performance monitoring for sensor board (JSON format)

**Board 3 (Elevator & Gripper Board) Messages:**
- **STEPPERSTAT3**: Stepper motor position and status (JSON format)
- **DIAG3**: Diagnostic messages from stepper module (JSON format)
- **PERF3**: Performance monitoring for elevator board (JSON format)

**Command Messages (from ROS2 to Teensy):**
- **TWIST**: Velocity commands for RoboClaw motors (Board 1) or stepper motors (Board 3)
- **STEPPOS**: Position commands for stepper motors (Board 3) - *elevator:X,extender:Y*
- **STEPHOME**: Homing command for stepper motors (Board 3)
- **STEPSTATUS**: Status query for stepper motors (Board 3)
- **ESTOP**: Emergency stop trigger/reset

**Message Frequency Observations:**
- **ODOM1**: Very high frequency (continuous stream)
- **ROBOCLAW1**: Medium frequency (~3Hz)
- **TEMPERATURE1**: Low frequency (~2Hz)
- **VL53L0X1**: High frequency per sensor
- **DIAG1**: Event-driven diagnostic messages
- **PERF1**: Periodic performance reports

### Enhanced Message Examples

**Performance (JSON Format):**
```
PERF1:{"freq":297.6, "tfreq":30.0, "mviol":1543, "fviol":0, "violdet":{"cmod":255,"cfreq":0,"lastms":16630}, "mods":[{"n":"SDLogger","min":0.00,"max":0.04,"avg":0.00,"last":0.03,"cnt":1544,"viol":false},{"n":"PerformanceMonitor","min":0.00,"max":0.17,"avg":0.00,"last":0.00,"cnt":1543,"viol":false},{"n":"SafetyCoordinator","min":0.00,"max":0.01,"avg":0.00,"last":0.00,"cnt":1543,"viol":false},{"n":"RoboClawMonitor","min":0.80,"max":4.77,"avg":1.37,"last":0.81,"cnt":1543,"viol":false},{"n":"VL53L0XMonitor","min":2.08,"max":3.77,"avg":2.44,"last":2.49,"cnt":1543,"viol":true},{"n":"TemperatureMonitor","min":0.00,"max":1.94,"avg":0.08,"last":0.06,"cnt":1543,"viol":false}]}
```

**VL53L0X Distance Sensors (JSON Format):**
```
VL53L0X1:{"total_sensors":8,"active_sensors":8,"min_distance":172,"max_distance":1676,"obstacles":true,"distances":[{"id":0,"mm":265,"raw":265,"age_us":2702,"degraded":false},{"id":1,"mm":1031,"raw":1025,"age_us":9866,"degraded":false},{"id":2,"mm":1043,"raw":1050,"age_us":2702,"degraded":false},{"id":3,"mm":172,"raw":181,"age_us":16505,"degraded":false},{"id":4,"mm":191,"raw":194,"age_us":2702,"degraded":false},{"id":5,"mm":1676,"raw":1690,"age_us":13203,"degraded":false},{"id":6,"mm":838,"raw":820,"age_us":9866,"degraded":false},{"id":7,"mm":212,"raw":209,"age_us":13203,"degraded":false}]}
```

**High-Frequency Odometry (JSON Format - Board 1):**
```
ODOM1:{"px":0.000,"py":0.000,"ox":0.000,"oy":0.000,"oz":0.000,"ow":1.000,"vx":0.000,"vy":0.000,"wz":0.000}
```

**Motor Status (JSON Format - Board 1):**
```
ROBOCLAW1:{"LogicVoltage":0.0,"MainVoltage":24.1,"Encoder_Left":0,"Encoder_Right":0,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":"0","ErrorDecoded":"No errors"}
```

**Temperature Monitoring (JSON Format):**
```
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[25.4,24.9,null,null,null,null,null,null],"avg_temp":25.1,"max_temp":25.4,"min_temp":24.9,"hottest_sensor":0,"system_warning":false,"system_critical":false,"rate_hz":19.9,"readings":120,"errors":0}
```

**Diagnostic Messages (JSON Format):**
```
DIAG1:{"level":"DEBUG","module":"SDLogger","message":"SDLogger: Performance stats: Buffer usage: 0 bytes (0%), Write rate: 10565.00 B/s, Total writes: 239","timestamp":16630}
FAULT1:{"active_fault":"false"}
```

**Battery Monitoring (JSON Format):**
```
BATT2:{"idx":0,"V":42.13,"A":1.64,"charge":1.00,"state":"NORMAL","location":"36VLIPO"}
BATT2:{"idx":1,"V":4.96,"A":2.45,"charge":0.00,"state":"NORMAL","location":"5VDCDC"}
BATT2:{"idx":2,"V":24.33,"A":0.11,"charge":0.00,"state":"NORMAL","location":"24VDCDC"}
BATT2:{"idx":3,"V":3.38,"A":0.67,"charge":0.00,"state":"NORMAL","location":"3.3VDCDC"}
BATT2:{"idx":4,"V":12.24,"A":3.23,"charge":0.00,"state":"NORMAL","location":"12VDCDC"}
```

**IMU Sensors (JSON Format):**
```
IMU2:{"id":0,"qx":0.9979,"qy":-0.0496,"qz":0.0071,"qw":-0.0404}
IMU2:{"id":1,"qx":0.0272,"qy":-0.0062,"qz":0.1066,"qw":-0.9939}
```

**High-Frequency Odometry (JSON Format - Board 1):**
```
ODOM1:{"px":-0.103,"py":0.043,"ox":0.000,"oy":0.000,"oz":-0.053,"ow":0.999,"vx":0.000,"vy":0.000,"wz":0.000}
```

**Motor Status (JSON Format - Board 1):**
```
ROBOCLAW1:{"LogicVoltage":5.1,"MainVoltage":27.0,"Encoder_Left":-2581,"Encoder_Right":4934,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0,"ErrorDecoded":"No errors"}
```

**Temperature Monitoring (JSON Format):**
```
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[26.9,28.9,null,null,null,null,null,null],"avg_temp":27.9,"max_temp":28.9,"min_temp":26.9,"hottest_sensor":1,"system_warning":false,"system_critical":false,"rate_hz":2.0,"readings":12164,"errors":0}
```

**Diagnostic Messages (JSON Format):**
```
DIAG1:{"level":"DEBUG","module":"SDLogger","message":"SDLogger: Performance stats: Buffer usage: 0 bytes (0%), Write rate: 4520.00 B/s, Total writes: 174754","timestamp":6086746}
DIAG1:{"level":"INFO","module":"RoboClawMonitor","message":"Diagnostic report,details:commands:3851207,errors:0,safety_violations:10,connection_state:2,emergency_stop:false","timestamp":6086747}
```

**Performance Monitoring (JSON Format):**
```
PERF1:{"freq":451.3, "tfreq":30.0, "mviol":8466, "fviol":127, "mods":[{"n":"SDLogger","min":0.00,"max":313.21,"avg":0.00,"last":0.00,"cnt":15976359,"viol":"F"},{"n":"PerformanceMonitor","min":0.00,"max":0.13,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"},{"n":"SafetyCoordinator","min":0.00,"max":0.06,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"},{"n":"RoboClawMonitor","min":0.00,"max":102.39,"avg":0.38,"last":2.21,"cnt":15976358,"viol":"F"},{"n":"TemperatureMonitor","min":0.00,"max":2.01,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"}]}
PERF2:{"freq":500000.0, "tfreq":30.0, "mviol":63856, "fviol":53, "mods":[{"n":"SDLogger","min":0.00,"max":44.60,"avg":0.00,"last":0.00,"cnt":2539818293,"viol":"F"},{"n":"PerformanceMonitor","min":0.00,"max":0.15,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"SafetyCoordinator","min":0.00,"max":0.00,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"BatteryMonitor","min":0.00,"max":2.26,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"BNO055Monitor","min":0.00,"max":5.42,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"}]}
```

### Odometry Message Format (ODOM)

High-frequency odometry messages are published at ≥70Hz for precise robot localization using JSON format:
```
ODOM1:{"px":-0.103,"py":0.043,"ox":0.000,"oy":0.000,"oz":-0.053,"ow":0.999,"vx":0.000,"vy":0.000,"wz":0.000}
```

- **Position**: `px`, `py` in meters (robot position in odometry frame)
- **Orientation**: `ox`, `oy`, `oz`, `ow` as quaternion components
- **Velocity**: `vx`, `vy` in m/s, `wz` in rad/s (current robot velocities)
- **Update Rate**: 70-85Hz depending on encoder read performance
- **Precision**: 3 decimal places for position/orientation, velocities as needed

### RoboClaw Status Message Format (ROBOCLAW)

Motor controller status published at ~3Hz for efficiency using JSON format:
```json
ROBOCLAW1:{"LogicVoltage":5.1,"MainVoltage":27.0,"Encoder_Left":-2581,"Encoder_Right":4934,"LeftMotorCurrent":0.000,"RightMotorCurrent":0.000,"LeftMotorSpeed":0,"RightMotorSpeed":0,"Error":0,"ErrorDecoded":"No errors"}
```

### Temperature Monitoring (TEMPERATURE)

Enhanced temperature monitoring with comprehensive sensor array data:
```json
TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[26.9,28.9,null,null,null,null,null,null],"avg_temp":27.9,"max_temp":28.9,"min_temp":26.9,"hottest_sensor":1,"system_warning":false,"system_critical":false,"rate_hz":2.0,"readings":12164,"errors":0}
```

- **total_sensors**: Total number of temperature sensors configured
- **active_sensors**: Number of sensors currently providing readings
- **temperatures**: Array of temperature readings (°C) with null for inactive sensors
- **avg_temp, max_temp, min_temp**: Statistical summary of active sensors
- **hottest_sensor**: Index of sensor with highest temperature
- **system_warning, system_critical**: Boolean flags for thermal protection
- **rate_hz**: Current temperature monitoring frequency
- **readings, errors**: Total count of successful readings and error count

### Battery Monitoring (BATT)

Multi-battery monitoring with comprehensive power system data:
```json
BATT2:{"idx":0,"V":42.13,"A":1.64,"charge":1.00,"state":"NORMAL","location":"36VLIPO"}
BATT2:{"idx":1,"V":4.96,"A":2.45,"charge":0.00,"state":"NORMAL","location":"5VDCDC"}
```

- **idx**: Battery index (0-4 for different power rails)
- **V**: Voltage reading in volts
- **A**: Current reading in amperes
- **charge**: Charge level (0.0-1.0) where applicable
- **state**: Battery status ("NORMAL", "WARNING", "CRITICAL")
- **location**: Physical battery/power rail identifier

### Diagnostic Messages (DIAG)

System diagnostic messages with module-specific information:
```json
DIAG1:{"level":"DEBUG","module":"SDLogger","message":"SDLogger: Performance stats: Buffer usage: 0 bytes (0%), Write rate: 4520.00 B/s, Total writes: 174754","timestamp":6086746}
DIAG1:{"level":"INFO","module":"RoboClawMonitor","message":"Diagnostic report,details:commands:3851207,errors:0,safety_violations:10,connection_state:2,emergency_stop:false","timestamp":6086747}
```

- **level**: Diagnostic level ("DEBUG", "INFO", "WARN", "ERROR", "FATAL")
- **module**: Source module generating the diagnostic
- **message**: Human-readable diagnostic message
- **timestamp**: System timestamp in milliseconds

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

**Example PERF Messages:**
```json
PERF1:{"freq":451.3, "tfreq":30.0, "mviol":8466, "fviol":127, "mods":[{"n":"SDLogger","min":0.00,"max":313.21,"avg":0.00,"last":0.00,"cnt":15976359,"viol":"F"},{"n":"PerformanceMonitor","min":0.00,"max":0.13,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"},{"n":"SafetyCoordinator","min":0.00,"max":0.06,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"},{"n":"RoboClawMonitor","min":0.00,"max":102.39,"avg":0.38,"last":2.21,"cnt":15976358,"viol":"F"},{"n":"TemperatureMonitor","min":0.00,"max":2.01,"avg":0.00,"last":0.00,"cnt":15976358,"viol":"F"}]}

PERF2:{"freq":500000.0, "tfreq":30.0, "mviol":63856, "fviol":53, "mods":[{"n":"SDLogger","min":0.00,"max":44.60,"avg":0.00,"last":0.00,"cnt":2539818293,"viol":"F"},{"n":"PerformanceMonitor","min":0.00,"max":0.15,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"SafetyCoordinator","min":0.00,"max":0.00,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"BatteryMonitor","min":0.00,"max":2.26,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"},{"n":"BNO055Monitor","min":0.00,"max":5.42,"avg":0.00,"last":0.00,"cnt":2539818292,"viol":"F"}]}
```

**Note**: The performance data shows actual operational frequencies significantly higher than target minimums:
- **Board 1**: Operating at ~450Hz (well above 85Hz target)
- **Board 2**: Operating at ~500kHz (extremely high performance)
- **Module violations**: Tracking shows occasional timing violations under heavy load
- **Long-term operation**: Both boards showing millions of loop iterations with stable performance

## Performance Requirements

### Real-Time Constraints
The TeensyV2 system maintains strict real-time performance requirements to ensure reliable robot operation:

**Loop Frequency Targets:**
- **Board 1**: 85Hz minimum (currently operating at ~450Hz)
- **Board 2**: 80Hz minimum (currently operating at ~500kHz) 
- **Tolerance**: ±5Hz acceptable, >10Hz deviation triggers warnings
- **Actual Performance**: Both boards significantly exceed minimum requirements

**Module Execution Limits:**
- **Critical Modules (RoboClawMonitor)**: Maximum 4ms execution time per loop iteration
- **Standard Modules**: Maximum 2ms execution time per loop iteration  
- **Total System**: Maximum 10ms total execution time per loop
- **Safety Margin**: 2ms reserved for interrupt handling and system overhead

**RoboClawMonitor Performance Tiers:**
- **High Frequency (≥70Hz)**: Encoder reads and odometry calculation for precise localization
- **Medium Frequency (~30Hz)**: Motor status monitoring and safety checks
- **Low Frequency (~3Hz)**: System diagnostics (voltage, current, temperature)
- **Cmd_vel Processing**: Up to 67Hz with intelligent rate limiting for responsive control

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
- Critical module execution time exceeding 4ms threshold (RoboClawMonitor)
- Standard module execution time exceeding 2ms threshold
- Loop frequency dropping below 70Hz for more than 3 consecutive cycles
- Memory usage approaching 80% of available capacity
- Communication errors or timeouts

**Escalation Procedures:**
1. **Warning Level**: Log message, increase monitoring frequency
2. **Error Level**: Reduce non-critical functionality, notify ROS2 system
3. **Critical Level**: Trigger controlled shutdown, activate E-stop if necessary

## Motor Control Optimization

### RoboClawMonitor Performance Architecture

The RoboClawMonitor has been extensively optimized for high-frequency odometry and responsive motor control, implementing a tiered execution model based on the legacy roboclaw_module.cpp design:

#### Execution Frequency Tiers

**Tier 1: High-Frequency Operations (≥70Hz)**
- **Encoder Reading**: Direct read of both motor encoders for odometry calculation
- **Odometry Calculation**: Real-time pose and velocity computation with microsecond timestamps
- **Cmd_vel Processing**: Velocity command processing with up to 67Hz rate limiting
- **Execution Time**: 2-4ms per cycle (includes encoder I/O and calculations)

**Tier 2: Medium-Frequency Operations (~30Hz)**  
- **Motor Status Monitoring**: Speed readings and basic system health checks
- **Safety Checks**: Overcurrent detection and runaway motor monitoring
- **Execution Time**: 1-2ms per cycle (spread across multiple loop iterations)

**Tier 3: Low-Frequency Operations (~3Hz)**
- **System Diagnostics**: Voltage, current, and temperature readings
- **Error Status**: RoboClaw error register monitoring  
- **Status Reporting**: JSON-formatted status messages to ROS2
- **Execution Time**: <1ms per cycle (minimal impact on real-time performance)

#### Odometry Performance

**High-Frequency Odometry Updates:**
- **Update Rate**: 70-85Hz (limited by encoder read speed, not computation)
- **Latency**: <15ms from wheel movement to ODOM message publication
- **Precision**: 3 decimal places for position (millimeter accuracy)
- **Integration Method**: Midpoint integration for improved accuracy over time
- **Data Freshness**: Encoder values read immediately before each odometry calculation

**Optimization Techniques:**
- **Direct Encoder Access**: Bypasses state machine for critical odometry reads
- **Minimal Computation**: Optimized trigonometric calculations
- **Reduced Message Overhead**: Compact ODOM message format
- **Smart Rate Limiting**: Only publishes when meaningful motion detected

#### Motor Command Processing

**Responsive Command Handling:**
- **Command Rate**: Up to 67Hz for velocity commands (15ms intervals)
- **Significant Change Detection**: Only sends commands when motor speeds change >10 QPPS
- **Force Updates**: Periodic updates every 100ms to maintain control authority
- **Timeout Protection**: Automatic motor stop after 200ms without commands

**Control Flow Optimization:**
- **Bypass State Machine**: Critical motor commands bypass slower diagnostic reads
- **Intelligent Batching**: Groups related motor operations for efficiency
- **Error Recovery**: Continues operation even with temporary communication errors
- **Rate Limiting**: Prevents overwhelming the RoboClaw controller

### Legacy Design Integration

The optimizations are based on proven patterns from the legacy roboclaw_module.cpp:

**Adopted Patterns:**
- **Frequency Separation**: Different update rates for odometry vs status reporting
- **Direct Hardware Access**: Minimal abstraction for time-critical operations  
- **Rate Limiting Logic**: Proven algorithms for preventing controller overload
- **Command Batching**: Efficient grouping of related motor operations

**Modern Improvements:**
- **State Machine Architecture**: Better error handling and recovery
- **Safety Integration**: Unified safety coordination across all modules
- **Performance Monitoring**: Real-time execution time tracking
- **Modular Design**: Clean separation of concerns for maintainability

## Safety System

### Multi-Layer Protection
The safety system implements defense-in-depth principles with multiple independent protection layers:

#### Layer 1: Hardware E-Stop
- **Physical Button**: Immediate hardware-level motor disconnection
- **Response Time**: <1ms from button press to motor shutdown
- **Failsafe Design**: Normally-open contacts (to be implemented), fail-safe on power loss

#### Layer 2: Software E-Stop
- **Performance Monitoring**: Automatic trigger on timing violations
- **Sensor Monitoring**: Battery voltage, current, runaway, temperature thresholds
- **Communication Loss**: Automatic E-stop on ROS2 communication timeout
- **Response Time**: <10ms from detection to safety response

#### Layer 3: Inter-Board Safety
- **Cross-Monitoring**: Each board monitors the other's health status (to be implemented)
- **Heartbeat Protocol**: Regular "alive" signals between boards (to be implemented)
- **Isolation Capability**: Either board can independently trigger system shutdown
- **Response Time**: <50ms for inter-board safety communication (to be measured)

#### Layer 4: ROS2 Safety Integration
- **High-Level Monitoring**: Navigation system safety checks
- **Operator Override**: Manual E-stop from operator console
- **Mission Abort**: Automatic abort on navigation failures
- **Response Time**: <100ms for ROS2-initiated safety responses (to be measured)

### Safety Thresholds

**Battery Safety:**
- Critical Low Voltage: 32.0V (immediate E-stop)
- Warning Low Voltage: 34.0V (return-to-base procedure)
- High Current: 20A (immediate E-stop, indicates short circuit or stall)
- Temperature: >60°C (thermal protection)

**Performance Safety:**
- Critical Module Timeout: >4ms execution time for RoboClawMonitor (indicates performance issues)
- Standard Module Timeout: >2ms execution time (indicates runaway code)
- System Frequency: <70Hz for >3 cycles (indicates system overload)

**Communication Safety:**
- ROS2 Timeout: >500ms without high-level commands (to be measured)
- Inter-Board Timeout: >100ms without heartbeat signal (to be implemented)
- Sensor Timeout: >200ms without critical sensor data (to be implemented)

### Emergency Procedures

**Automatic Recovery:**
- E-stop conditions automatically reset when underlying problems resolve
- Graceful system restart without manual intervention when safe
- Progressive re-enablement of functionality based on system health
- Operator notification and approval required for critical system recovery

**Manual Override:**
- ROS2 service calls for controlled system restart
- Diagnostic mode for troubleshooting without full system activation
- Maintenance mode with safety systems active but operational systems disabled

## Development

### Adding New Modules
1. Inherit from `Module` base class
2. Implement required virtual methods
3. Register via singleton pattern
4. See `docs/ModuleReference.md` for details

### Configuration
- Default values in `config.h` files
- Runtime updates via ROS2 parameters

### Testing
- Unit tests for critical safety functions
- Hardware-in-the-loop testing framework
- Performance benchmarking tools

## Recent Enhancements (2024-2025)

### Message Protocol Improvements
- **Board Identification**: Enhanced message format with board-specific identifiers (IMU1/IMU2, BATT1/BATT2)
- **JSON Payload Support**: Structured data transmission for complex sensor readings
- **Enhanced Diagnostics**: Improved error reporting and system status monitoring

### Dual IMU Implementation
- **Staggered Reporting**: Fixed timing logic in BNO055Monitor for proper sensor alternation
- **Board-Specific Messages**: IMU1/IMU2 message types for proper sensor identification
- **Synchronized Updates**: Coordinated sensor readings from both embedded boards

### Performance Optimizations
- **High-Frequency Odometry**: Maintained ≥70Hz odometry updates for precise navigation
- **Optimized Serial Communication**: Enhanced message throughput and reduced latency
- **Real-time Monitoring**: Improved performance tracking and violation detection

### Safety System Enhancements
- **Multi-Board Safety**: Enhanced safety coordination between multiple Teensy boards
- **Improved E-stop Logic**: More robust emergency stop detection and recovery
- **Battery Protection**: Enhanced battery monitoring with location-specific thresholds

## License

This project is part of the Sigyn autonomous house patroller system. See LICENSE file for details.

## Contributing

1. Follow embedded C++ best practices and Arduino coding standards
2. Test thoroughly on both debug and production hardware configurations  
3. Update documentation for any protocol or API changes
4. Ensure real-time performance requirements are maintained
5. Add comprehensive unit tests for safety-critical functionality

## Support

For technical support, hardware questions, or development contributions:
1. Review existing documentation in `docs/` directory
2. Check module-specific implementation details
3. Test with both individual and integrated board configurations
4. Create detailed issue reports with serial output logs
