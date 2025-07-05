# Teensy Board 1 - Motor Control and Sensing

This Teensy 4.1 board handles motor control, distance sensing, and battery monitoring for the Sigyn robot.

## Hardware Configuration

### Sensors and Actuators:
- **RoboClaw Motor Controller**: Differential drive control
- **VL53L0X Time-of-Flight Sensor**: Precision distance measurement
- **Battery Monitor**: Main battery voltage and health monitoring
- **Sonar Module**: Ultrasonic distance sensing (currently disabled)

### Pin Assignments:
- **I2C (SDA/SCL)**: VL53L0X sensors
- **Serial7**: RoboClaw communication
- **Analog Pin A0**: Battery voltage divider (configured in `config.h`)
- **Digital Pins**: Sonar trigger/echo (when enabled)

## Module Overview

### Active Modules:

#### BatteryModule
- Monitors main battery voltage via analog input
- Implements rolling average for stable readings
- Sends battery status via serial at regular intervals
- Implements safety warnings for low battery conditions
- **Performance**: Reads every 100ms, reports every 1000ms

#### RoboClawModule  
- Controls differential drive motors via RoboClaw controller
- Handles TWIST commands (linear.x, angular.z velocities)
- Provides odometry feedback
- Implements safety stops and emergency braking
- **Performance**: Updates motor commands at ~50Hz

#### VL53L0XModule
- Manages VL53L0X time-of-flight distance sensors
- Supports multiple sensors on I2C bus
- Provides obstacle detection data
- **Performance**: Reads sensors at configurable intervals

#### SdModule
- SD card data logging (currently disabled in main loop)
- Automatic log file creation with incremental numbering
- Buffered writing for performance
- Directory listing and file dump capabilities

### Inactive/Commented Modules:

#### SonarModule
- Ultrasonic distance sensor support
- Currently commented out but available for future use

## Communication Protocol

### Incoming Messages (from PC/ROS2):
- `TWIST:linear_x,angular_z` - Motor velocity commands
- `SDDIR:` - Request SD card directory listing  
- `SDFILE:filename` - Request file contents from SD card

### Outgoing Messages (to PC/ROS2):
- Battery status and voltage readings
- RoboClaw status and odometry data
- VL53L0X distance measurements
- Diagnostic and error messages

## Configuration

Key configuration parameters in `config.h`:
- Battery monitoring pins and thresholds
- Motor controller settings
- Sensor update rates
- Safety limits

## Safety Features

- **Battery Monitoring**: Low voltage warnings and critical shutdown
- **Motor Safety**: Emergency stop capabilities, velocity limits
- **Sensor Monitoring**: Failure detection and reporting
- **Timeout Protection**: Communication timeout handling

## Performance Characteristics

- **Main Loop**: Targets 100Hz operation
- **Critical Modules**: Motor control prioritized for responsiveness
- **Battery Monitoring**: Lower frequency (1Hz) sufficient for battery health
- **Distance Sensors**: Configurable rates based on application needs

## Build and Upload

1. Open `teensy_board1.ino` in Arduino IDE with Teensyduino
2. Select **Board**: Teensy 4.1
3. Select **USB Type**: Serial
4. Configure **CPU Speed**: 600 MHz
5. Verify connections match pin assignments
6. Compile and upload

## Troubleshooting

### Motor Control Issues:
- Check RoboClaw connections and configuration
- Verify TWIST message format
- Monitor battery voltage (low voltage affects motor performance)

### Sensor Issues:
- Verify I2C connections for VL53L0X
- Check for address conflicts on I2C bus
- Monitor serial output for sensor status messages

### Battery Monitoring:
- Verify voltage divider circuit
- Check analog reference voltage
- Calibrate voltage reading coefficients if needed

### Communication Problems:
- Verify USB connection and serial port
- Monitor message format and syntax

## Future Enhancements

- Enable sonar module for redundant distance sensing
- Add SD card logging for diagnostic data
- Implement advanced motor control algorithms
- Add more sophisticated battery management
