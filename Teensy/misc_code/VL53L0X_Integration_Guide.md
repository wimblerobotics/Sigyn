# VL53L0X Time-of-Flight Distance Sensor Module

## Overview

The VL53L0X module provides single-zone time-of-flight distance sensing capabilities for the Teensy-based robotics system. The VL53L0X is a reliable ToF sensor that provides distance measurements in millimeters up to 2 meters. It supports up to 8 VL53L0X sensors connected through an I2C multiplexer, with automatic initialization, error handling, and real-time distance measurements.

## Features

- **Single-Zone ToF Sensor**: VL53L0X with reliable distance measurements
- **Multiple Sensor Support**: Up to 8 VL53L0X sensors via I2C multiplexer
- **Fast Operation**: 30Hz measurement frequency with 20ms timing budget
- **Non-blocking Operation**: Fast loop() execution (< 2ms) for real-time performance
- **Automatic Data Streaming**: Distance data sent every 10ms per sensor
- **Error Handling**: Robust initialization and NaN reporting for failed sensors
- **Safety Integration**: Obstacle detection for collision avoidance (< 50mm threshold)
- **Modular Architecture**: Integrates seamlessly with the Module system

## Prerequisites

### VL53L0X Library Installation

This module requires the Pololu VL53L0X library. Install it through:

1. **Arduino Library Manager**: Search for "VL53L0X" and install the Pololu library
2. **Manual Installation**: Download from [Pololu's VL53L0X GitHub repository](https://github.com/pololu/vl53l0x-arduino)

**Library Used**: `#include <VL53L0X.h>` (Pololu library)

## Hardware Setup

### I2C Multiplexer Connection
- **Multiplexer Address**: 0x70 (TCA9548A or similar)
- **I2C Pins**: Standard Teensy I2C (SDA/SCL)
- **VL53L0X Sensors**: Connect to multiplexer channels 0-7

### Wiring Example
```
Teensy 4.1        I2C Multiplexer (TCA9548A)        VL53L0X Sensors
SDA (18) -------- SDA                               
SCL (19) -------- SCL                               
3.3V     -------- VCC                               
GND      -------- GND                               
                  CH0 -------------------- Sensor 0 (SDA/SCL/VCC/GND)
                  CH1 -------------------- Sensor 1 (SDA/SCL/VCC/GND)
                  CH2 -------------------- Sensor 2 (SDA/SCL/VCC/GND)
                  ...                      ...
                  CH7 -------------------- Sensor 7 (SDA/SCL/VCC/GND)
```

**Important Notes:**
- VL53L0X operates at 3.3V (compatible with Teensy 4.1)
- All sensors start with the same I2C address (0x29), but the module automatically assigns unique addresses
- Use quality jumper wires for reliable I2C communication
- Keep sensor wiring short (< 30cm) for best performance

## Software Integration

### Basic Usage

```cpp
#include "vl53l0x_module.h"
#include "module.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize the module (automatic registration)
    VL53L0XModule::singleton();
    
    // Initialize all modules (including VL53L0X)
    Module::Setup();
}

void loop() {
    // Run all modules (including VL53L0X)
    Module::Loop();
    
    // Optional: Manual distance reading
    float distance = VL53L0XModule::singleton().getDistanceMm(0);
    if (!isnan(distance)) {
        Serial.printf("Distance: %.1f mm\n", distance);
    }
}
```

### Enabling Multiple Sensors

To enable more sensors, modify the header file:

```cpp
// In vl53l0x_module.h
static const uint8_t ENABLED_SENSORS = 4; // Enable 4 sensors instead of 1
```

### Serial Output Format

The module automatically sends distance data via SerialManager:

```
VL53L0X_DISTANCE:sensor_id,distance_mm
```

Examples:
```
VL53L0X_DISTANCE:0,123.4    # Sensor 0: 123.4mm
VL53L0X_DISTANCE:1,NAN      # Sensor 1: Not initialized or timeout
VL53L0X_DISTANCE:2,456.7    # Sensor 2: 456.7mm
```

## API Reference

### Public Methods

```cpp
class VL53L0XModule : public Module {
public:
    // Get singleton instance
    static VL53L0XModule& singleton();
    
    // Get distance from specific sensor (returns NaN if not initialized)
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
static const uint8_t MAX_SENSORS = 8;           // Maximum supported sensors
static const uint8_t ENABLED_SENSORS = 1;      // Currently enabled sensors (default: 1 for testing)
static const uint16_t SAFETY_THRESHOLD_MM = 50; // Safety threshold for obstacle detection
```

### VL53L0X Specific Features

- **Measurement Range**: 30mm to 2000mm (typical)
- **Measurement Frequency**: 30Hz for real-time performance
- **Timing Budget**: 20ms for good accuracy/speed balance
- **Timeout Handling**: 500ms timeout for reliable operation
- **Address Management**: Automatic unique address assignment per sensor

## Configuration Options

### Timing Budget Adjustment

To change measurement speed vs accuracy:

```cpp
// In initializeSensor() function, modify:
sensor->setMeasurementTimingBudget(33000); // 33ms for higher accuracy
// or
sensor->setMeasurementTimingBudget(15000); // 15ms for faster measurements
```

### Safety Threshold

To adjust obstacle detection threshold:

```cpp
// In vl53l0x_module.h, modify:
static const uint16_t SAFETY_THRESHOLD_MM = 100; // 100mm threshold
```

## Troubleshooting

### Common Issues

1. **"I2C multiplexer NOT found"**
   - Check TCA9548A wiring and power
   - Verify multiplexer address (0x70)
   - Test with I2C scanner

2. **"Failed to initialize sensor"**
   - Check VL53L0X wiring and power supply
   - Verify 3.3V power level
   - Check for loose connections

3. **Getting NAN distance values**
   - Check for I2C communication errors
   - Verify sensor is pointing at a valid target
   - Check for electromagnetic interference

4. **Inconsistent readings**
   - Ensure stable power supply
   - Use shorter, higher-quality I2C cables
   - Check for mechanical vibrations

### Performance Tips

- Keep I2C wiring short and use twisted pairs
- Use adequate power supply (stable 3.3V)
- Avoid pointing sensors at reflective or transparent surfaces
- Consider ambient light conditions for outdoor use
- Use appropriate timing budget for your speed requirements

## Integration with Main Code

Add to your main .ino file:

```cpp
#include "vl53l0x_module.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize VL53L0X module
    VL53L0XModule::singleton();
    
    // Initialize all modules
    Module::Setup();
}

void loop() {
    // Run all modules
    Module::Loop();
    
    // Handle safety checks
    if (VL53L0XModule::singleton().isUnsafe()) {
        // Take safety action (stop motors, etc.)
        Serial.println("SAFETY: Obstacle detected!");
    }
}
```

The module will automatically:
- Initialize all enabled sensors
- Send distance data every 10ms
- Provide safety status for obstacle avoidance
- Handle I2C multiplexer switching
- Manage sensor timeouts and errors
