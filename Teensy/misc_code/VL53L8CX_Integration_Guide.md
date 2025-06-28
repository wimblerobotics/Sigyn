# VL53L8CX Multi-Zone Time-of-Flight Sensor Module

## Overview

The VL53L8CX module provides advanced multi-zone time-of-flight distance sensing capabilities for the Teensy-based robotics system. The VL53L8CX is a sophisticated ToF sensor that provides distance measurements across multiple zones (4x4 or 8x8 grid), but this module simplifies usage by reporting the center zone distance. It supports up to 8 VL53L8CX sensors connected through an I2C multiplexer, with automatic initialization, error handling, and real-time distance measurements.

## Features

- **Multi-Zone ToF Sensor**: VL53L8CX with 4x4 or 8x8 zone resolution (center zone reported)
- **Multiple Sensor Support**: Up to 8 VL53L8CX sensors via I2C multiplexer
- **High-Speed Operation**: 100Hz ranging frequency with 4x4 resolution
- **Non-blocking Operation**: Fast loop() execution (< 2ms) for real-time performance
- **Automatic Data Streaming**: Distance data sent every 10ms per sensor
- **Error Handling**: Robust initialization and NaN reporting for failed sensors
- **Safety Integration**: Obstacle detection for collision avoidance
- **Modular Architecture**: Integrates seamlessly with the Module system

## Prerequisites

### VL53L8CX Library Installation

This module requires the ST VL53L8CX library. Install it through:

1. **Arduino Library Manager**: Search for "VL53L8CX" and install the ST official library
2. **Manual Installation**: Download from [ST's VL53L8CX GitHub repository](https://github.com/stmicroelectronics/VL53L8CX)
3. **Alternative**: Use the ST X-NUCLEO-53L8A1 expansion board library

**Important**: Update the `#include` statements in `vl53l8cx_module.cpp` to match your library:
```cpp
#include <vl53l8cx_class.h>  // ST official library
// or
#include <VL53L8CX.h>       // Community libraries
```

## Hardware Setup

### I2C Multiplexer Connection
- **Multiplexer Address**: 0x70 (TCA9548A or similar)
- **I2C Pins**: Standard Teensy I2C (SDA/SCL)
- **VL53L8CX Sensors**: Connect to multiplexer channels 0-7

### Wiring Example
```
Teensy 4.1        I2C Multiplexer (TCA9548A)        VL53L8CX Sensors
SDA (18) -------- SDA                               
SCL (19) -------- SCL                               
3.3V     -------- VCC                               
GND      -------- GND                               
                  CH0 -------------------- Sensor 0 (SDA/SCL)
                  CH1 -------------------- Sensor 1 (SDA/SCL)
                  CH2 -------------------- Sensor 2 (SDA/SCL)
                  ...                      ...
                  CH7 -------------------- Sensor 7 (SDA/SCL)
```

## Software Integration

### Basic Usage

```cpp
#include "vl53l8cx_module.h"
#include "module.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize the module (automatic registration)
    VL53L8CXModule::singleton();
    
    // Setup all modules
    Module::Setup();
}

void loop() {
    // Run all modules (including VL53L8CX)
    Module::Loop();
    
    // Optional: Manual distance reading
    float distance = VL53L8CXModule::singleton().getDistanceMm(0);
    if (!isnan(distance)) {
        Serial.printf("Sensor 0: %.1f mm\\n", distance);
    }
}
```

### Configuration

To enable more sensors, modify the header file:

```cpp
// In vl53l8cx_module.h
static const uint8_t ENABLED_SENSORS = 4; // Enable 4 sensors instead of 1
```

### Serial Output Format

The module automatically sends distance data via SerialManager:

```
VL53L8CX_DISTANCE:sensor_id,distance_mm
```

**Note**: The distance represents the center zone measurement from the VL53L8CX's multi-zone array (zone 7 for 4x4 resolution, zone 27 for 8x8 resolution).

Examples:
```
VL53L8CX_DISTANCE:0,123.4    # Sensor 0: 123.4mm (center zone)
VL53L8CX_DISTANCE:1,NAN      # Sensor 1: Not initialized or no valid target
VL53L8CX_DISTANCE:2,456.7    # Sensor 2: 456.7mm (center zone)
```

## API Reference

### Public Methods

```cpp
class VL53L8CXModule : public Module {
public:
    // Get singleton instance
    static VL53L8CXModule& singleton();
    
    // Get distance from specific sensor center zone (returns NaN if not initialized)
    float getDistanceMm(uint8_t sensor_index);
    
    // Check if sensor is initialized
    bool isSensorInitialized(uint8_t sensor_index);
    
    // Safety interface
    bool isUnsafe() override;           // Returns true if obstacle < 50mm
    void resetSafetyFlags() override;   // Reset safety state
};
```

### Constants

```cpp
static const uint8_t MAX_SENSORS = 8;      // Maximum supported sensors
static const uint8_t ENABLED_SENSORS = 1;  // Currently enabled sensors (default: 1 for testing)
```

### VL53L8CX Specific Features

- **Resolution**: 4x4 zones (16 zones) by default for faster operation
- **Ranging Frequency**: 100Hz for real-time performance
- **Zone Selection**: Center zone used for distance reporting
- **Target Status**: Only valid targets (VL53L8CX_TARGET_STATUS_RANGED) reported

## Performance Characteristics

### Timing
- **Constructor**: ~1-2 seconds (sensor initialization)
- **loop() execution**: < 2ms (maintains real-time performance)
- **Read interval**: 10ms per sensor
- **Data transmission**: Every 10ms
- **I2C operations**: ~1-2ms per sensor read

### Memory Usage
- **RAM**: ~100 bytes + (40 bytes × enabled sensors)
- **Flash**: ~4KB (excluding VL53L8CX library)

## Safety Features

### Obstacle Detection
- **isUnsafe()**: Returns true when any sensor detects obstacle < 50mm
- **Integration**: Automatically checked by Module system
- **Customizable**: Modify threshold in `isUnsafe()` method

### Error Handling
- **Initialization Failures**: Sensors marked as uninitialized, NaN reported
- **I2C Timeouts**: Handled gracefully, NaN values for failed reads
- **Multiplexer Errors**: Individual sensor failures don't affect others

## Library Requirements

### VL53L8CX Library
The module requires a VL53L8CX Arduino library. You may need to:

1. **Install Library**: Add VL53L8CX library to Arduino/Teensy environment
2. **Update Includes**: Uncomment the `#include <VL53L8CX.h>` line
3. **Implement Methods**: Replace placeholder code with actual library calls

### Expected Library API
The module assumes the VL53L8CX library has methods similar to VL53L0X:

```cpp
class VL53L8CX {
public:
    bool init();
    void setTimeout(uint16_t timeout);
    void setMeasurementTimingBudget(uint32_t budget_us);
    void startContinuous();
    uint16_t readRangeContinuousMillimeters();
    bool timeoutOccurred();
};
```

## Troubleshooting

### Common Issues

1. **"Sensor X initialization failed"**
   - Check I2C wiring to multiplexer
   - Verify sensor power (3.3V)
   - Ensure VL53L8CX library is installed

2. **"NAN" distance values**
   - Sensor not initialized properly
   - I2C communication failure
   - Check multiplexer address (0x70)

3. **Performance Issues**
   - Reduce number of enabled sensors
   - Increase read intervals
   - Check I2C bus speed (400kHz recommended)

### Debug Output
Enable debug output by uncommenting Serial.printf statements in the source code.

## Integration with ROS

The serial output format is designed for easy ROS integration:

```python
# Example ROS parsing (Python)
if line.startswith("VL53L8CX_DISTANCE:"):
    parts = line.split(":")
    data = parts[1].split(",")
    sensor_id = int(data[0])
    distance = float(data[1]) if data[1] != "NAN" else float('nan')
    
    # Publish to ROS topic
    range_msg = Range()
    range_msg.header.frame_id = f"vl53l8cx_{sensor_id}"
    range_msg.range = distance / 1000.0  # Convert mm to meters
    publisher.publish(range_msg)
```

## Example Projects

- **VL53L8CX_Example.ino**: Basic usage demonstration
- **Multiple sensors**: Configure for 4-8 sensors for full coverage
- **Obstacle avoidance**: Use with motor control for autonomous navigation
- **Mapping**: Integrate with SLAM for environment mapping

## Support

- **Module System**: See `module.h` documentation for integration details
- **Serial Communication**: See `serial_manager.h` for output protocols
- **Hardware**: Consult VL53L8CX and TCA9548A datasheets for electrical specs
