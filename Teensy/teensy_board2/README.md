# Teensy Board 2 - IMU and Orientation Sensing

This Teensy 4.1 board handles inertial measurement and orientation sensing for the Sigyn robot using BNO055 9-DOF sensors.

## Hardware Configuration

### Sensors:
- **BNO055 9-DOF IMU**: Provides quaternions, gyroscope, accelerometer, and Euler angles
- **I2C Multiplexer (PCA9548)**: Enables multiple BNO055 sensors on same bus
- **SD Card Module**: Optional data logging capability

### Pin Assignments:
- **I2C (SDA/SCL)**: BNO055 sensors and I2C multiplexer
- **SPI**: SD card (when enabled)
- **USB Serial**: Communication with main computer/ROS2

## Module Overview

### Active Modules:

#### BNO055Module
- Manages up to 2 BNO055 IMU sensors via I2C multiplexer
- Provides real-time orientation data (quaternions, Euler angles)
- Outputs gyroscope data (angular velocities in rad/s)
- Outputs linear acceleration data (m/s²)
- Implements coordinate frame conversion for ROS compatibility
- **Performance**: Reads sensors at 100Hz, sends data at 10Hz

#### SdModule
- SD card data logging capabilities (currently disabled in main loop)
- Automatic log file creation with incremental numbering
- Buffered writing for optimal performance
- Directory listing and file dump functionality

### Sensor Data Output:

#### IMU Data Structure:
```cpp
struct IMUData {
  float qw, qx, qy, qz;             // Quaternion (orientation)
  float gx, gy, gz;                 // Gyroscope (rad/s)
  float ax, ay, az;                 // Linear acceleration (m/s²)  
  float euler_h, euler_r, euler_p;  // Euler angles (degrees)
  uint32_t read_time_tenths_ms;     // Read timing (0.1ms units)
  bool valid;                       // Data validity flag
};
```

## Communication Protocol

### Incoming Messages (from PC/ROS2):
- `IMU:command` - IMU-related commands and configuration
- `SDDIR:` - Request SD card directory listing
- `SDFILE:filename` - Request file contents from SD card

### Outgoing Messages (to PC/ROS2):
- IMU data in both native BNO055 and ROS coordinate frames
- Sensor status and calibration information
- Diagnostic messages and error reports

## BNO055 Configuration

### Sensor Setup:
- **Operation Mode**: NDOF (Nine Degrees of Freedom fusion mode)
- **I2C Address**: 0x28 (default BNO055 address)
- **Multiplexer Address**: 0x70 (PCA9548 default)
- **Update Rate**: 100Hz sensor reads, 10Hz data transmission

### Coordinate Frame Conversion:
The module provides both native BNO055 data and ROS-compatible coordinate frames:
- **BNO055 Frame**: Right-handed, Z-up
- **ROS Frame**: Converted for compatibility with ROS navigation stack

### Calibration:
- Automatic calibration status monitoring
- Gyroscope calibration on startup
- Accelerometer and magnetometer calibration via movement

## Performance Characteristics

- **Sensor Read Rate**: 100Hz per sensor for real-time performance
- **Data Transmission**: 10Hz to reduce serial bandwidth usage
- **Read Time**: Typically <1ms per sensor for 2ms total loop time
- **Multiple Sensors**: Automatic time-division multiplexing

## Safety and Monitoring

### Safety Features:
- Sensor failure detection and reporting
- I2C communication error handling
- Automatic sensor re-initialization on failure
- Data validity checking and flagging

### Diagnostic Output:
- Calibration status monitoring
- Read timing statistics
- Communication error reporting
- Sensor availability status

## Build and Upload

1. Open `teensy_board2.ino` in Arduino IDE with Teensyduino
2. Select **Board**: Teensy 4.1
3. Select **USB Type**: Serial
4. Configure **CPU Speed**: 600 MHz
5. Verify I2C connections and addresses
6. Compile and upload

## Hardware Setup

### I2C Connections:
```
Teensy 4.1    PCA9548 Multiplexer    BNO055 Sensors
SDA (18)  ->  SDA                    
SCL (19)  ->  SCL                    
3.3V      ->  VCC                    
GND       ->  GND                    
          ->  SC0/SD0  ->  SDA/SCL   (Sensor 0)
          ->  SC1/SD1  ->  SDA/SCL   (Sensor 1)
```

### I2C Address Configuration:
- **Multiplexer**: 0x70 (adjustable via hardware jumpers)
- **BNO055 Sensors**: 0x28 each (same address, different mux channels)

## Troubleshooting

### IMU Communication Issues:
- Verify I2C connections and pull-up resistors
- Check multiplexer addressing and channel selection
- Monitor serial output for initialization status
- Verify 3.3V power supply stability

### Data Quality Issues:
- Check sensor calibration status
- Ensure sensors are properly mounted and isolated from vibration
- Verify coordinate frame transformations
- Monitor data validity flags

### Performance Issues:
- Check loop timing statistics in serial output
- Verify sensor read intervals are appropriate
- Monitor I2C bus for conflicts or errors
- Ensure adequate processing time between sensor reads

### Calibration Problems:
- Follow BNO055 calibration procedures
- Ensure sensors have proper magnetic field environment
- Allow adequate warm-up time after power-on
- Check for metal interference near sensors

## Future Enhancements

- Support for additional BNO055 sensors (up to 8 via multiplexer)
- Enhanced calibration status reporting
- SD card logging of IMU data for analysis
- Advanced sensor fusion algorithms
- Temperature compensation and monitoring
