# BNO055 Quaternion Output and ROS Compatibility Guide

## Overview

This document clarifies the BNO055 quaternion output format, coordinate system, and compatibility with ROS conventions for the Sigyn robotics project.

## BNO055 Quaternion Format

### Data Order and Scaling
The BNO055 outputs quaternions in the order **W, X, Y, Z** (scalar first):
- **Register order**: W_LSB, W_MSB, X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
- **Raw values**: 16-bit signed integers
- **Scaling factor**: 1/16384 (2^-14) to convert to floating point
- **Output range**: [-1.0, 1.0] for normalized quaternions

### Implementation in Module
```cpp
// From bno055_module.cpp:
bool BNO055Module::read_quaternion(uint8_t addr, float *w, float *x, float *y, float *z) {
    uint8_t buf[8];
    if (!bno_read(addr, BNO055_QUATERNION_DATA_W_LSB, buf, 8)) return false;
    int16_t qw = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t qx = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t qy = (int16_t)(buf[5] << 8 | buf[4]);
    int16_t qz = (int16_t)(buf[7] << 8 | buf[6]);
    *w = qw / 16384.0f;  // Scalar component first
    *x = qx / 16384.0f;  // Vector x component
    *y = qy / 16384.0f;  // Vector y component
    *z = qz / 16384.0f;  // Vector z component
    return true;
}
```

## BNO055 Coordinate System

### Board Orientation (Adafruit BNO055 Breakout)
According to Adafruit documentation and the webserial_3d example:

```
     Board Layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
```

### Axis Definition
- **X-axis**: Points to the right when looking at the component side
- **Y-axis**: Points forward (away from pin labels)
- **Z-axis**: Points up (out of the board surface)
- **Chirality**: Right-handed coordinate system

### Default Axis Mapping (REMAP_CONFIG_P1)
- **X → X**: No remapping
- **Y → Y**: No remapping  
- **Z → Z**: No remapping
- **Signs**: All positive (REMAP_SIGN_P1)

## ROS Coordinate System Conventions

### ROS Standard (REP-103)
According to [ROS REP-103](https://www.ros.org/reps/rep-0103.html):

**Body Frame Convention:**
- **X**: Forward
- **Y**: Left  
- **Z**: Up
- **Chirality**: Right-handed
- **Quaternion Order**: `x, y, z, w` (vector components first, scalar last)

**ROS geometry_msgs/Quaternion:**
```cpp
float64 x  // Vector component
float64 y  // Vector component  
float64 z  // Vector component
float64 w  // Scalar component
```

## Compatibility Analysis

### 1. Quaternion Component Order
- **BNO055**: W, X, Y, Z (scalar first)
- **ROS**: X, Y, Z, W (scalar last)
- **Solution**: Reorder components when publishing to ROS

### 2. Coordinate System Alignment
The BNO055 and ROS coordinate systems may require axis remapping depending on physical mounting:

**If BNO055 is mounted with:**
- Component side up
- Y-axis pointing robot forward
- X-axis pointing robot right

**Then the mapping is:**
- BNO055 X → ROS Y (right → left, sign flip needed)
- BNO055 Y → ROS X (forward → forward)  
- BNO055 Z → ROS Z (up → up)

### 3. Recommended Conversion for ROS
```cpp
// Convert BNO055 quaternion to ROS format
void convertToROS(float bno_w, float bno_x, float bno_y, float bno_z,
                  float &ros_x, float &ros_y, float &ros_z, float &ros_w) {
    // Reorder: BNO055 (w,x,y,z) → ROS (x,y,z,w)
    // Apply coordinate transform if needed based on mounting
    ros_x = bno_y;   // BNO055 Y → ROS X (forward)
    ros_y = -bno_x;  // BNO055 X → ROS Y (right→left, flip sign)
    ros_z = bno_z;   // BNO055 Z → ROS Z (up)
    ros_w = bno_w;   // Scalar component
}
```

## Quick Reference

### BNO055 vs ROS Comparison Table

| Aspect | BNO055 | ROS (REP-103) |
|--------|--------|---------------|
| **Quaternion Order** | W, X, Y, Z | X, Y, Z, W |
| **X-axis** | Right | Forward |
| **Y-axis** | Forward | Left |
| **Z-axis** | Up | Up |
| **Coordinate System** | Right-handed | Right-handed |
| **Units** | Normalized [-1,1] | Normalized [-1,1] |

### Transformation Summary
```cpp
// Quaternion reordering + coordinate transform
ros_x = bno_y;    // Forward axis (Y→X)
ros_y = -bno_x;   // Right→Left with sign flip (X→Y)  
ros_z = bno_z;    // Up axis unchanged (Z→Z)
ros_w = bno_w;    // Scalar component unchanged
```

### Usage in Code
```cpp
// Get ROS-ready data
float qx, qy, qz, qw, gx, gy, gz, ax, ay, az;
if (BNO055Module::singleton().getIMUDataROS(0, qx, qy, qz, qw, gx, gy, gz, ax, ay, az)) {
    // Data is now in ROS format and coordinate system
    // Ready for sensor_msgs::Imu publication
}
```

---

## Current Implementation Status

### IMUData Structure
```cpp
struct IMUData {
    bool valid;
    uint32_t timestamp_ms;
    float qw, qx, qy, qz;        // Quaternion (BNO055 order: w,x,y,z)
    float gyro_x, gyro_y, gyro_z; // Gyroscope (rad/s)
    float accel_x, accel_y, accel_z; // Linear acceleration (m/s²)
    float euler_x, euler_y, euler_z; // Euler angles (degrees)
    uint8_t calibration_status;
};
```

### Serial Output Format
The current serial interface outputs quaternions in BNO055 order:
```
IMU_DATA,timestamp,qw,qx,qy,qz,gx,gy,gz,ax,ay,az,ex,ey,ez,cal
```

## Recommendations

### 1. For ROS Integration
- Add conversion function to reorder quaternion components (w,x,y,z → x,y,z,w)
- Apply coordinate transform based on physical sensor mounting
- Document the specific mounting orientation used

### 2. Coordinate System Documentation
- Clearly mark the sensor orientation on robot
- Include axis direction diagrams in system documentation
- Test orientation with known movements (pitch up, roll right, yaw clockwise)

### 3. Calibration and Validation
- Use ROS visualization tools (rviz) to verify quaternion orientation
- Compare with other IMU sources if available
- Test with robot motion to ensure directions match expectations

## References

1. [BNO055 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
2. [Adafruit BNO055 Library](https://github.com/adafruit/Adafruit_BNO055)
3. [ROS REP-103: Standard Units and Coordinate Conventions](https://www.ros.org/reps/rep-0103.html)
4. [ROS geometry_msgs/Quaternion](https://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Quaternion.html)
5. [Adafruit BNO055 Guide](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)

## Notes

- The BNO055 quaternion output is mathematically correct and follows the Hamilton convention (w + xi + yj + zk)
- The coordinate system differences are due to different robotics conventions, not errors in the sensor
- Both systems use right-handed coordinate systems, just with different axis assignments
- Proper coordinate transformation ensures compatibility while preserving orientation accuracy
