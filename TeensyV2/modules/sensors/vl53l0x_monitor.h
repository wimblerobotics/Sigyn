/**
 * @file vl53l0x_monitor.h
 * @brief VL53L0X Time-of-Flight sensor monitoring for TeensyV2 system
 *
 * Provides real-time monitoring of multiple VL53L0X Time-of-Flight distance
 * sensors through an I2C multiplexer. Supports up to 8 sensors with automatic
 * initialization, error handling, and performance monitoring.
 *
 * Features:
 * - Support for up to 8 VL53L0X sensors via I2C multiplexer
 * - Non-blocking sensor readings with state machine operation
 * - Automatic sensor initialization and error recovery
 * - Distance measurement in millimeters with configurable accuracy
 * - Safety integration for obstacle detection
 * - Comprehensive diagnostic and performance reporting
 *
 * Hardware Requirements:
 * - VL53L0X sensors connected via TCA9548A I2C multiplexer
 * - I2C communication at 400kHz for optimal performance
 * - Sensors should be mounted with clear line-of-sight
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <cstdint>
#include <cmath>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

/**
 * @brief Configuration parameters for VL53L0X sensor system.
 */
struct VL53L0XConfig {
  // Hardware configuration
  uint8_t i2c_multiplexer_address = 0x70;     ///< TCA9548A multiplexer I2C address
  uint8_t max_sensors = 8;                    ///< Maximum number of sensors supported
  uint8_t enabled_sensors = 8;               ///< Number of sensors currently enabled
  
  // Measurement configuration
  uint32_t measurement_timeout_ms = 50;       ///< Measurement timeout per sensor
  uint32_t timing_budget_us = 33000;          ///< Timing budget for measurements (33ms)
  uint16_t signal_rate_limit = 200;           ///< Signal rate limit (MCPS)
  
  // Safety thresholds
  uint16_t obstacle_threshold_mm = 200;       ///< Obstacle detection threshold (mm)
  uint16_t warning_threshold_mm = 500;        ///< Warning distance threshold (mm)
  uint16_t max_range_mm = 2000;               ///< Maximum measurement range (mm)
  
  // Operational parameters
  uint32_t sensor_cycle_time_ms = 100;        ///< Time between sensor cycles
  uint32_t initialization_retry_ms = 1000;    ///< Retry interval for failed sensors
  uint32_t error_recovery_time_ms = 5000;     ///< Time before attempting error recovery
  
  // Reporting intervals
  uint32_t status_report_interval_ms = 200;   ///< Status reporting interval
  uint32_t diagnostic_report_interval_ms = 2000; ///< Diagnostic reporting interval
};

/**
 * @brief Individual sensor status and measurement data.
 */
struct SensorStatus {
  // Measurement data
  uint16_t distance_mm = 0;                   ///< Current distance measurement (mm)
  uint8_t range_status = 255;                 ///< VL53L0X range status
  uint16_t signal_rate = 0;                   ///< Signal rate (MCPS)
  uint16_t ambient_rate = 0;                  ///< Ambient rate (MCPS)
  
  // Status flags
  bool initialized = false;                   ///< Sensor successfully initialized
  bool measurement_valid = false;             ///< Current measurement is valid
  bool communication_ok = false;              ///< Communication with sensor OK
  
  // Safety flags
  bool obstacle_detected = false;             ///< Obstacle within threshold
  bool warning_distance = false;              ///< Distance within warning threshold
  bool range_error = false;                   ///< Range measurement error
  
  // Performance tracking
  uint32_t total_measurements = 0;            ///< Total measurements taken
  uint32_t error_count = 0;                   ///< Number of measurement errors
  uint32_t last_measurement_time_ms = 0;      ///< Time of last measurement
  float measurement_frequency_hz = 0.0f;      ///< Current measurement frequency
};

/**
 * @brief System-wide sensor array status.
 */
struct SensorArrayStatus {
  uint8_t total_sensors = 0;                  ///< Total number of sensors
  uint8_t initialized_sensors = 0;            ///< Number of successfully initialized sensors
  uint8_t active_sensors = 0;                 ///< Number of currently active sensors
  uint8_t sensors_with_obstacles = 0;         ///< Number of sensors detecting obstacles
  
  uint16_t min_distance_mm = 65535;           ///< Minimum distance detected
  uint16_t max_distance_mm = 0;               ///< Maximum distance detected
  uint8_t min_distance_sensor = 255;          ///< Sensor with minimum distance
  
  bool multiplexer_ok = false;                ///< I2C multiplexer communication OK
  bool any_obstacles = false;                 ///< Any sensor detecting obstacles
  bool system_ready = false;                  ///< System ready for measurements
  
  uint32_t total_measurements = 0;            ///< Total measurements across all sensors
  uint32_t total_errors = 0;                  ///< Total errors across all sensors
  float system_measurement_rate_hz = 0.0f;    ///< Overall system measurement rate
};

/**
 * @brief VL53L0X Time-of-Flight sensor array monitor.
 * 
 * This module provides comprehensive monitoring and control of multiple VL53L0X
 * sensors through an I2C multiplexer, with real-time obstacle detection,
 * safety integration, and performance monitoring.
 */
class VL53L0XMonitor : public Module {
public:
  static VL53L0XMonitor& getInstance();
  
  // Sensor access interface
  float getDistanceMm(uint8_t sensor_index) const;
  bool isSensorReady(uint8_t sensor_index) const;
  bool isObstacleDetected(uint8_t sensor_index) const;
  const SensorStatus& getSensorStatus(uint8_t sensor_index) const;
  const SensorArrayStatus& getArrayStatus() const { return array_status_; }
  
  // Configuration interface
  void updateConfig(const VL53L0XConfig& config) { config_ = config; }
  const VL53L0XConfig& getConfig() const { return config_; }
  
  // Control interface
  void reinitializeSensor(uint8_t sensor_index);
  void reinitializeAll();
  void enableSensor(uint8_t sensor_index, bool enable);

protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() const override { return "VL53L0XMonitor"; }
  
  // Safety interface
  bool isUnsafe() override;
  void resetSafetyFlags() override;

private:
  // Singleton constructor
  VL53L0XMonitor();
  
  // State machine for sensor operations
  enum class SensorState {
    UNINITIALIZED,
    INITIALIZING,
    READY,
    MEASURING,
    ERROR_RECOVERY
  };
  
  // Core functionality
  void updateSensorCycle();
  void initializeSensors();
  void performMeasurements();
  void updateArrayStatus();
  void checkSafetyConditions();
  void sendStatusReports();
  void sendDiagnosticReports();
  
  // I2C multiplexer control
  void selectSensorChannel(uint8_t sensor_index);
  bool testMultiplexer();
  void resetMultiplexer();
  
  // Individual sensor management
  bool initializeSingleSensor(uint8_t sensor_index);
  bool measureSingleSensor(uint8_t sensor_index);
  void handleSensorError(uint8_t sensor_index);
  void updateSensorPerformance(uint8_t sensor_index);
  
  // Safety and error handling
  void detectObstacles();
  void handleSystemError();
  void recoverFromErrors();
  void recoverFailedSensors();
  
  // Configuration and state
  VL53L0XConfig config_;
  SensorArrayStatus array_status_;
  
  // Sensor hardware and state
  VL53L0X sensors_[8];                        ///< VL53L0X sensor objects
  SensorStatus sensor_status_[8];             ///< Status for each sensor
  SensorState sensor_states_[8];              ///< State machine state for each sensor
  bool sensor_enabled_[8];                    ///< Enable flag for each sensor
  
  // Operational state
  uint8_t current_sensor_index_;              ///< Currently active sensor in cycle
  uint32_t sensor_cycle_start_time_ms_;       ///< Start time of current sensor cycle
  bool multiplexer_available_;                ///< I2C multiplexer available
  bool system_initialized_;                   ///< System initialization complete
  
  // Timing for periodic operations
  uint32_t last_status_report_time_ms_;
  uint32_t last_diagnostic_report_time_ms_;
  uint32_t last_safety_check_time_ms_;
  uint32_t last_initialization_attempt_ms_;
  
  // Performance tracking
  uint32_t system_start_time_ms_;
  uint32_t last_measurement_cycle_time_ms_;
  uint32_t total_system_measurements_;
  uint32_t total_system_errors_;
};

} // namespace sigyn_teensy
