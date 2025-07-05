# BNO055 IMU Module Integration Guide

## Overview

The BNO055Module has been successfully integrated into the Teensy modular system. It provides high-performance, non-blocking IMU data reading with PC/ROS2 communication capabilities.

## Features

### Hardware Support
- Supports multiple BNO055 chips (currently configured for 1, expandable to 2)
- I2C communication at 400kHz after initialization
- Automatic chip detection and initialization
- Robust error handling and timeout management

### Data Output
- Quaternion (qw, qx, qy, qz)
- Gyroscope data (gx, gy, gz) in rad/s
- Linear acceleration (ax, ay, az) in m/s²
- Euler angles (heading, roll, pitch) in degrees
- Read timing information (in 0.1ms units)
- Data validity flags

### Communication Protocol

#### Automatic Data Transmission
The module automatically sends IMU data every 100ms (10Hz) in the following format:
```
IMU_DATA:chip_id,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,h,r,p,read_time
```

Example:
```
IMU_DATA:0,0.9998,0.0123,-0.0045,0.0167,0.012,-0.003,0.001,0.02,-0.01,9.81,1.23,-2.45,0.78,23
```

#### Command Interface
Send commands to the Teensy via serial in the format: `IMU:command`

Available commands:
- `IMU:IMU_GET_DATA` - Request immediate data from all sensors
- `IMU:IMU_SET_RATE:chip_id:rate_ms` - Set read rate for specific chip (1-1000ms)
- `IMU:IMU_CALIBRATE:chip_id` - Get calibration status for specific chip

#### Command Examples
```bash
# Get immediate data
echo "IMU:IMU_GET_DATA" > /dev/ttyACM0

# Set chip 0 to read every 50ms (20Hz)
echo "IMU:IMU_SET_RATE:0:50" > /dev/ttyACM0

# Check calibration status of chip 0
echo "IMU:IMU_CALIBRATE:0" > /dev/ttyACM0
```

#### Response Examples
```
# Data response
IMU_DATA:0,0.9998,0.0123,-0.0045,0.0167,0.012,-0.003,0.001,0.02,-0.01,9.81,1.23,-2.45,0.78,23

# Rate setting confirmation
IMU_RATE_SET:0:50

# Calibration status (hex value: sys, gyro, accel, mag calibration levels)
IMU_CALIB:0:FF

# Error responses
IMU_ERROR:Invalid parameters
IMU_ERROR:Unknown command
```

## Performance Characteristics

### Timing Performance
- Constructor: ~1 second (includes sensor initialization and calibration)
- loop() execution: < 2ms (meets real-time requirements)
- I2C read operations: ~200-500 microseconds per sensor
- Supports ~100Hz overall system operation

### Safety Features
- `isUnsafe()`: Returns true if sensor data is invalid
- `resetSafetyFlags()`: Clears error states
- Automatic retry on communication failures
- Non-blocking operations in main loop

## Integration Details

### Module Architecture
The BNO055Module follows the Module base class pattern:
- Singleton pattern with automatic registration
- Non-blocking loop() for real-time performance
- Blocking constructor for one-time initialization
- Performance statistics tracking

### Code Integration
```cpp
#include "bno055_module.h"

// In main program
BNO055Module& bno055_module = BNO055Module::singleton();

void setup() {
    Module::Setup(); // Calls setup() on all modules
}

void loop() {
    Module::Loop(); // Calls loop() on all modules + handles serial
}
```

### Data Access API
```cpp
// Get data from specific chip
IMUData data;
if (bno055_module.getIMUData(0, data)) {
    // Use data.qw, data.qx, etc.
    if (data.valid) {
        // Data is good
    }
}

// Check number of chips
uint8_t num_chips = bno055_module.getNumChips();
```

## Configuration

### Hardware Configuration
- Chip 0: I2C address 0x28 (BNO055_ADDRESS_A)
- Chip 1: I2C address 0x29 (BNO055_ADDRESS_B) - if enabled
- Default read rate: 10ms per chip
- Default transmission rate: 100ms (10Hz)

### Modifying Configuration
Edit `bno055_module.cpp` constructor:
```cpp
// Change number of chips
chips_.resize(2); // For 2 chips

// Change read rates
chips_[0].read_interval_ms = 20; // 50Hz

// Change transmission rate
data_send_interval_ms_ = 50; // 20Hz
```

## Troubleshooting

### Common Issues
1. **Sensor not detected**: Check I2C wiring and power
2. **Invalid data**: Check calibration status using IMU_CALIBRATE command
3. **Slow performance**: Check that loop() is completing in < 2ms
4. **No data transmission**: Verify serial connection and baud rate (115200)

### Debug Information
The module provides detailed debug output including:
- Initialization status for each chip
- Communication errors
- Performance timing statistics
- Calibration status

### Performance Monitoring
Module statistics are automatically reported every second, including:
- Execution timing for BNO055Module loop()
- Overall system loops per second
- Performance warnings for slow operations

## ROS2 Integration

### Message Format
The ASCII output can be easily parsed by ROS2 nodes:
```python
# Example Python parsing
def parse_imu_data(line):
    if line.startswith("IMU_DATA:"):
        parts = line[9:].split(',')
        chip_id = int(parts[0])
        qw, qx, qy, qz = map(float, parts[1:5])
        gx, gy, gz = map(float, parts[5:8])
        ax, ay, az = map(float, parts[8:11])
        h, r, p = map(float, parts[11:14])
        read_time = int(parts[14])
        return chip_id, (qw, qx, qy, qz), (gx, gy, gz), (ax, ay, az), (h, r, p), read_time
```

### Recommended ROS2 Topics
- `/imu/data_raw` - Raw IMU data (sensor_msgs/Imu)
- `/imu/mag` - Magnetometer data (sensor_msgs/MagneticField)
- `/imu/calibration` - Calibration status (custom message)

## ROS Integration

For ROS compatibility, the BNO055Module provides specialized functions that handle coordinate system transformation and quaternion reordering:

### ROS Conversion Functions
```cpp
// Get IMU data in ROS format (x,y,z,w quaternion order, ROS coordinate system)
bool getIMUDataROS(uint8_t chip_index, float &qx, float &qy, float &qz, float &qw,
                   float &gyro_x, float &gyro_y, float &gyro_z,
                   float &accel_x, float &accel_y, float &accel_z);

// Manual conversion function
void convertQuaternionToROS(float bno_w, float bno_x, float bno_y, float bno_z,
                           float &ros_x, float &ros_y, float &ros_z, float &ros_w);
```

**See `BNO055_ROS_Compatibility_Guide.md` for detailed information about coordinate systems, quaternion conventions, and ROS integration.**

## Future Enhancements

### Potential Improvements
1. **Dynamic chip discovery**: Auto-detect available chips on startup
2. **Advanced calibration**: Automated calibration procedures
3. **Fusion algorithms**: Additional sensor fusion beyond hardware NDOF
4. **Configuration via commands**: Runtime configuration changes
5. **Data logging**: SD card logging of IMU data
6. **Temperature compensation**: Use built-in temperature sensor

### Performance Optimizations
1. **Burst reading**: Read multiple registers in single I2C transaction
2. **Adaptive rates**: Automatically adjust read rates based on data changes
3. **Prediction**: Interpolate data between reads for higher apparent rates
4. **Filtering**: Software filtering options for noisy environments
