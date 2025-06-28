#pragma once

#include <vl53l8cx_api.h>
#include "module.h"
#include "serial_manager.h"
#include <Wire.h>
#include <stdint.h>
#include <vector>

// Use the library's definitions from vl53l8cx_api.h
// VL53L8CX_STATUS_OK, VL53L8CX_RESOLUTION_4X4, VL53L8CX_RESOLUTION_8X8 are already defined
// VL53L8CX_ResultsData is already defined in the library

#ifndef VL53L8CX_TARGET_STATUS_RANGED
#define VL53L8CX_TARGET_STATUS_RANGED 5
#endif

/*
 * VL53L8CX Time-of-Flight Distance Sensor Module
 * 
 * Manages multiple VL53L8CX sensors connected through an I2C multiplexer.
 * The VL53L8CX is a multi-zone Time-of-Flight sensor that provides distance 
 * measurements in millimeters with 8x8 zone resolution.
 * 
 * Features:
 * - Supports up to 8 VL53L8CX sensors
 * - Uses I2C multiplexer (TCA9548A) for sensor selection
 * - Non-blocking operation in loop()
 * - 10ms update rate per sensor
 * - Automatic sensor initialization and error handling
 * - Multi-zone distance readings (using center zone for simplicity)
 * - Serial output for distance data via SerialManager
 */
class VL53L8CXModule : public Module {
public:
    // Maximum number of sensors supported
    static const uint8_t MAX_SENSORS = 8;
    
    // Number of sensors currently enabled (change this to enable more sensors)
    static const uint8_t ENABLED_SENSORS = 1;
    
    // Get singleton instance
    static VL53L8CXModule& singleton();
    
    // Get distance from specific sensor (returns NaN if sensor not initialized)
    float getDistanceMm(uint8_t sensor_index);
    
    // Check if sensor is initialized
    bool isSensorInitialized(uint8_t sensor_index);

protected:
    // Module interface implementation
    void setup() override;
    void loop() override;
    const char* name() override { return "VL53"; }
    
    // Safety interface (inherited from Module)
    bool isUnsafe() override;
    void resetSafetyFlags() override;

private:
    // Private constructor (singleton pattern)
    VL53L8CXModule();
    
    // Sensor data structure
    struct SensorData {
        VL53L8CX_Configuration* sensor;  // VL53L8CX configuration object pointer
        bool initialized;                // Initialization status
        uint32_t last_read_time;         // Last time sensor was read
        float last_distance_mm;          // Last measured distance (center zone)
        uint32_t read_interval_ms;       // Read interval (10ms)
        uint8_t mux_channel;             // Multiplexer channel
        uint8_t resolution;              // Sensor resolution (4x4 or 8x8)
    };
    
    // Static instance pointer
    static VL53L8CXModule* g_instance_;
    
    // Sensor array
    std::vector<SensorData> sensors_;
    
    // Timing for data transmission
    uint32_t last_data_send_time_;
    uint32_t data_send_interval_ms_;
    
    // Setup status
    bool setup_completed_;
    
    // I2C multiplexer configuration
    static const uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;
    
    // Helper functions
    void selectSensor(uint8_t sensor_index);
    bool initializeSensor(uint8_t sensor_index);
    void sendDistanceData();
    void readSensorDistance(uint8_t sensor_index);
};
