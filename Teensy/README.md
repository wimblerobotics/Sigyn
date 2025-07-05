# Sigyn Teensy Code

This directory contains the embedded C++ code for Sigyn's two Teensy 4.1 microcontrollers. Each Teensy board handles a specific set of sensors and actuators, providing real-time data collection and motor control for the robot.

## Architecture Overview

The code follows a modular architecture where:
- Each sensor/actuator is implemented as a `Module` subclass
- Modules automatically register themselves with the main system
- The main loop calls `Module::Loop()` which executes all registered modules
- Each module's `loop()` function must complete within ~2ms for real-time performance

## Board Configuration

### Teensy Board 1 (`teensy_board1/`)
**Primary Functions**: Motor control, distance sensing, battery monitoring
- **RoboClaw Motor Controller**: Differential drive motor control
- **VL53L0X ToF Sensor**: Distance measurement
- **Sonar Module**: Ultrasonic distance sensing (optional)
- **Battery Monitor**: Voltage monitoring and low-battery detection

### Teensy Board 2 (`teensy_board2/`)
**Primary Functions**: Inertial measurement and orientation
- **BNO055 IMU**: 9-DOF sensor providing quaternions, gyroscope, and accelerometer data
- Supports multiple BNO055 sensors via I2C multiplexer

## Common Modules

Both boards share several common modules:
- **Module Base Class**: Framework for all sensor/actuator modules
- **Serial Manager**: USB communication with the main computer/ROS2
- **SD Card Module**: Data logging capabilities
- **Config**: Board-specific configuration constants

## Communication Protocol

Both Teensy boards communicate with the main computer via USB Serial using a simple message format:
```
TYPE:DATA
```

### Supported Message Types:
- `IMU:` - IMU-related commands
- `TWIST:` - Motor velocity commands (linear.x, angular.z)
- `SDDIR:` - SD card directory listing
- `SDFILE:` - SD card file operations

## Performance Requirements

- **Target Loop Rate**: ~100Hz for critical sensors (motor control, IMU)
- **Module Loop Time**: Each module's `loop()` must complete in ≤2ms
- **Long Operations**: Use state machines to break work into small chunks

## Development Guidelines

### Code Style
- Follow Google C++ Style Guide
- Use `snake_case` for variables and functions
- Use `PascalCase` for classes
- Add `override` keyword for virtual function overrides

### Module Development
1. Inherit from the `Module` base class
2. Implement singleton pattern with automatic registration
3. Keep `loop()` functions fast (≤2ms)
4. Use state machines for long operations
5. Implement safety checks via `isUnsafe()` and `resetSafetyFlags()`

### Testing
- Monitor module performance statistics via serial output
- Check timing constraints using built-in performance monitoring
- Test safety systems and emergency stops

## Build Instructions

These programs are compiled using the Arduino IDE with the Teensyduino add-on:

1. Install Arduino IDE 2.x
2. Install Teensyduino from PJRC
3. Configure for Teensy 4.1
4. Open the respective `.ino` file for each board
5. Select appropriate board and port
6. Compile and upload

## Hardware Setup

### Teensy Board 1 Connections:
- **I2C**: VL53L0X sensors, battery monitoring
- **Serial**: RoboClaw motor controller
- **Analog**: Battery voltage divider
- **Digital**: Sonar trigger/echo pins

### Teensy Board 2 Connections:
- **I2C**: BNO055 IMU(s), I2C multiplexer
- **SPI**: SD card (optional)

## Safety Features

Both boards implement safety systems:
- Module-level safety monitoring via `isUnsafe()` 
- Emergency stop capabilities
- Battery level monitoring and warnings
- Sensor failure detection and reporting

## Troubleshooting

### Common Issues:
1. **Module not responding**: Check if module registered successfully
2. **Poor performance**: Monitor module timing statistics  
3. **Communication errors**: Verify USB connection. Baud rate is irrelevant for Teensy boards using USB.
4. **Sensor failures**: Check I2C connections and addresses
5. ** RoboClaw not responding**: Verify serial connections, baud rate and firwmware version.

### Debug Output:
Enable diagnostic messages via `SerialManager` for troubleshooting:
- Module performance statistics
- Sensor readings
- Error conditions
- Safety status

## Contributing

When adding new modules:
1. Follow the existing Module pattern
2. Keep `loop()` functions fast
3. Add appropriate safety checks
4. Document functionality and usage
5. Test real-time performance requirements
