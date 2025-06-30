#pragma once

#include <VL53L0X.h>
#include "module.h"
#include "serial_manager.h"
#include <Wire.h>
#include <stdint.h>
#include <vector>

/*
 * VL53L0X Time-of-Flight Distance Sensor Module
 * 
 * Manages multiple VL53L0X sensors connected through an I2C multiplexer.
 * The VL53L0X is a single-zone Time-of-Flight sensor that provides distance 
 * measurements in millimeters.
 * 
 * Features:
 * - Supports up to 8 VL53L0X sensors
 * - Uses I2C multiplexer (TCA9548A) for sensor selection
 * - Non-blocking operation in loop()
 * 
 * - Automatic sensor initialization and error handling
 * - Single-zone distance readings
 * - Serial output for distance data via SerialManager
 */
class VL53L0XModule : public Module {
public:
    // Maximum number of sensors supported
    static const uint8_t MAX_SENSORS = 8;
    
    // Number of sensors currently enabled (change this to enable more sensors)
    static const uint8_t ENABLED_SENSORS = 1;
    
    // Get singleton instance
    static VL53L0XModule& singleton();
    
    // Get distance from specific sensor (returns NaN if sensor not initialized)
    float getDistanceMm(uint8_t sensor_index);
    
    // Check if sensor is initialized
    bool isSensorInitialized(uint8_t sensor_index);

protected:
    // Module interface implementation
    void setup() override;
    void loop() override;
    const char* name() override { return "VL53L0X"; }
    
    // Safety interface (inherited from Module)
    bool isUnsafe() override;
    void resetSafetyFlags() override;

private:
    // Private constructor (singleton pattern)
    VL53L0XModule();
    
    // Sensor data structure
    struct SensorData {
        VL53L0X* sensor;              // VL53L0X sensor object pointer
        bool initialized;             // Initialization status
        uint32_t last_read_time;      // Last time sensor was read
        float last_distance_mm;       // Last measured distance
        uint32_t read_interval_ms;    // Read interval (10ms)
        uint8_t mux_channel;          // Multiplexer channel
        uint8_t i2c_address;          // Sensor I2C address (after address change)
    };
    
    // Static instance pointer
    static VL53L0XModule* g_instance_;
    
    // Sensor array
    std::vector<SensorData> sensors_;
    
    // Timing for data transmission
    uint32_t last_data_send_time_;
    uint32_t data_send_interval_ms_;
    
    // Setup status
    bool setup_completed_;
    
    // Multiplexer availability
    bool multiplexer_available_;
    
    // I2C multiplexer configuration
    static const uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;
    
    // Default VL53L0X I2C address
    static const uint8_t VL53L0X_DEFAULT_ADDRESS = 0x29;
    
    // Safety threshold for obstacle detection (millimeters)
    static const uint16_t SAFETY_THRESHOLD_MM = 50;
    
    // Helper functions
    void selectSensor(uint8_t sensor_index);
    bool initializeSensor(uint8_t sensor_index);
    void sendDistanceData();
    void readSensorDistance(uint8_t sensor_index);
    bool testI2CMultiplexer();
};
