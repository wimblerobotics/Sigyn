# TeensyV2 - Real-Time Embedded Control System

## Overview

TeensyV2 is a redesigned real-time embedded control system for the Sigyn autonomous robot. It provides modular, high-performance sensor monitoring, motor control, and safety systems across multiple Teensy 4.1 microcontrollers.

## Key Features

- **Real-Time Performance Monitoring**: Maintains 80-100Hz control loops with violation detection
- **Modular Architecture**: Clean separation of concerns with automatic module registration
- **Global Safety Coordination**: Comprehensive E-stop system with multiple trigger sources
- **Efficient Communication**: Optimized message protocol for low-latency ROS2 integration
- **Configuration Management**: Runtime parameter updates via ROS2 parameter interface

## Quick Start

### Prerequisites
- PlatformIO IDE or Arduino IDE with Teensy support
- ROS2 Jazzy
- Two Teensy 4.1 boards with appropriate sensors

### Compilation
```bash
# For PlatformIO (recommended)
cd TeensyV2/platform/board1
pio run

cd ../board2  
pio run

# For Arduino IDE
# Open TeensyV2/platform/board1/board1_main.ino
# Select Tools -> Board -> Teensy 4.1
# Compile and upload
```

### ROS2 Integration
```bash
cd /path/to/sigyn_ws
colcon build --packages-select sigyn_to_sensor_v2
source install/setup.bash
ros2 launch sigyn_to_sensor_v2 teensy_bridge.launch.py
```

## Architecture

### Board Assignments
- **Board 1**: Motor control, odometry, VL53L0X sensors, RoboClaw interface, E-stop coordination
- **Board 2**: Battery monitoring, IMU sensors, additional sensors

### Core Components
- **Module System**: Base class for all functionality with automatic registration
- **Performance Monitor**: Real-time timing analysis and safety violation detection  
- **Safety Coordinator**: Global E-stop management and emergency procedures
- **Message Protocol**: Efficient, parseable communication format
- **Configuration Manager**: Runtime parameter updates

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
└── platform/                  # Board-specific implementations
    ├── board1/                 # Motor control board
    │   ├── board1_main.ino     # Main program for board 1
    │   ├── platformio.ini      # PlatformIO configuration
    │   └── config.h            # Board 1 specific configuration
    └── board2/                 # Sensor monitoring board
        ├── board2_main.ino     # Main program for board 2
        ├── platformio.ini      # PlatformIO configuration
        └── config.h            # Board 2 specific configuration
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

## Safety System

The E-stop system provides multiple layers of protection:

1. **Local Detection**: Each board detects safety violations in real-time
2. **Global Coordination**: Board 1 aggregates all E-stop conditions
3. **Automatic Recovery**: E-stops reset when conditions clear
4. **ROS2 Integration**: E-stop status published to `/robot/emergency_stop`

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
