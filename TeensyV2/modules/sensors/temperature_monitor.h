/**
 * @file temperature_monitor.h
 * @brief Temperature monitoring system for TeensyV2
 *
 * Provides real-time temperature monitoring for motors and system components
 * using analog TMP36 temperature sensors. Integrates with the safety
 * system for thermal protection and provides comprehensive diagnostic reporting.
 *
 * Features:
 * - Support for multiple analog TMP36 temperature sensors
 * - Real-time temperature monitoring with configurable thresholds
 * - Safety integration with automatic thermal protection
 * - Temperature trend analysis and prediction
 * - Comprehensive error handling and sensor validation
 * - Performance optimization for minimal loop impact
 *
 * Safety Features:
 * - Critical temperature shutdown
 * - Warning temperature alerts
 * - Thermal runaway detection
 * - Automatic system protection
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <cmath>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

// Maximum number of temperature sensors supported
static constexpr uint8_t kMaxTemperatureSensors = 8;
// Default number of sensors configured (left and right motor)
static constexpr uint8_t kDefaultSensorsConfigured = 2;

/**
 * @brief Temperature sensor configuration and thresholds.
 */
struct TemperatureSensorConfig {
  // Sensor identification
  String sensor_name;                       ///< Human-readable sensor name
  String location;                          ///< Physical location description
  uint8_t analog_pin = 255;                 ///< Analog pin number for TMP36 sensors (255 = not configured)
  
  // Temperature thresholds (Celsius)
  float critical_high_temp = 85.0f;         ///< Critical high temperature (°C)
  float warning_high_temp = 70.0f;          ///< Warning high temperature (°C)
  float warning_low_temp = -10.0f;          ///< Warning low temperature (°C)
  float critical_low_temp = -20.0f;         ///< Critical low temperature (°C)
  
  // Operational parameters
  bool enabled = true;                      ///< Sensor enabled for monitoring
  uint32_t read_interval_ms = 1000;         ///< Reading interval (ms)
  uint8_t resolution_bits = 12;             ///< Temperature resolution (9-12 bits)
  
  // Safety parameters
  bool safety_critical = true;              ///< Sensor participates in safety system
  uint32_t fault_timeout_ms = 5000;         ///< Time before sensor fault triggers safety
  float thermal_runaway_rate = 5.0f;        ///< Temperature rise rate for runaway (°C/min)
};

/**
 * @brief Global temperature monitoring configuration.
 */
struct TemperatureMonitorConfig {
  // Hardware configuration
  uint8_t max_sensors = kMaxTemperatureSensors; ///< Maximum number of sensors
  
  // System thresholds
  float system_critical_temp = 80.0f;       ///< System-wide critical temperature
  float system_warning_temp = 65.0f;        ///< System-wide warning temperature
  
  // Operational parameters
  uint32_t scan_interval_ms = 5000;         ///< Sensor discovery scan interval
  
  // Reporting intervals
  uint32_t status_report_interval_ms = 2000; ///< Status reporting interval
  uint32_t diagnostic_report_interval_ms = 10000; ///< Diagnostic reporting interval
  
  // Safety settings
  bool enable_thermal_protection = true;    ///< Enable thermal protection
  uint32_t thermal_runaway_window_ms = 60000; ///< Time window for runaway detection
  uint8_t min_sensors_for_safety = 1;       ///< Minimum sensors required for safety
};

/**
 * @brief Individual temperature sensor status.
 */
struct TemperatureSensorStatus {
  // Current readings
  float temperature_c = NAN;                ///< Current temperature (°C)
  float temperature_f = NAN;                ///< Current temperature (°F)
  bool reading_valid = false;               ///< Current reading is valid
  
  // Sensor status
  bool sensor_present = false;              ///< Sensor detected on bus
  bool communication_ok = false;            ///< Communication working
  uint8_t resolution = 12;                  ///< Current resolution setting
  
  // Safety status
  bool critical_high = false;               ///< Critical high temperature
  bool warning_high = false;                ///< Warning high temperature
  bool warning_low = false;                 ///< Warning low temperature
  bool critical_low = false;                ///< Critical low temperature
  bool thermal_runaway = false;             ///< Thermal runaway detected
  
  // Performance tracking
  uint32_t total_readings = 0;              ///< Total successful readings
  uint32_t error_count = 0;                 ///< Number of read errors
  uint32_t last_reading_time_ms = 0;        ///< Time of last reading
  float reading_frequency_hz = 0.0f;        ///< Current reading frequency
  
  // Temperature history for trend analysis
  float temperature_history[10];            ///< Recent temperature history
  uint8_t history_index = 0;                ///< Current history index
  float temperature_trend = 0.0f;           ///< Temperature trend (°C/min)
  float max_temperature = -273.15f;         ///< Maximum recorded temperature
  float min_temperature = 1000.0f;          ///< Minimum recorded temperature
};

/**
 * @brief System-wide temperature monitoring status.
 */
struct TemperatureSystemStatus {
  // System overview
  uint8_t total_sensors = 0;                ///< Total configured sensors
  uint8_t active_sensors = 0;               ///< Currently active sensors
  uint8_t sensors_in_warning = 0;           ///< Sensors in warning state
  uint8_t sensors_in_critical = 0;          ///< Sensors in critical state
  
  // System temperatures
  float highest_temperature = -273.15f;     ///< Highest current temperature
  float lowest_temperature = 1000.0f;       ///< Lowest current temperature
  float average_temperature = NAN;          ///< Average system temperature
  uint8_t hottest_sensor = 255;             ///< Index of hottest sensor
  uint8_t coldest_sensor = 255;             ///< Index of coldest sensor
  
  // Safety status
  bool system_thermal_warning = false;      ///< System thermal warning active
  bool system_thermal_critical = false;     ///< System thermal critical active
  bool thermal_protection_active = false;   ///< Thermal protection engaged
  uint32_t time_in_warning_ms = 0;          ///< Time in warning state
  uint32_t time_in_critical_ms = 0;         ///< Time in critical state
  
  // Performance
  uint32_t total_readings = 0;              ///< Total system readings
  uint32_t total_errors = 0;                ///< Total system errors
  float system_reading_rate_hz = 0.0f;      ///< Overall system reading rate
};

/**
 * @brief Temperature monitoring module for TeensyV2 system.
 * 
 * This module provides comprehensive temperature monitoring using analog TMP36
 * sensors with real-time safety protection, trend analysis, and diagnostic
 * reporting. It follows the TeensyV2 module architecture with minimal
 * impact on real-time performance.
 */
class TemperatureMonitor : public Module {
public:
  static TemperatureMonitor& getInstance();
  
  // Sensor access interface
  float getTemperature(uint8_t sensor_index, bool fahrenheit = false) const;
  bool isSensorValid(uint8_t sensor_index) const;
  bool isTemperatureCritical(uint8_t sensor_index) const;
  uint8_t getSensorCount() const;
  const TemperatureSensorStatus& getSensorStatus(uint8_t sensor_index) const;
  const TemperatureSystemStatus& getSystemStatus() const { return system_status_; }
  
  // Configuration interface
  void updateConfig(const TemperatureMonitorConfig& config) { config_ = config; }
  const TemperatureMonitorConfig& getConfig() const { return config_; }
  void configureSensor(uint8_t sensor_index, const TemperatureSensorConfig& sensor_config);
  
  // Control interface
  void scanForSensors();
  void calibrateSensor(uint8_t sensor_index, float reference_temp);
  void resetSensorStatistics(uint8_t sensor_index);
  void resetSystemStatistics();

protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() const override { return "TemperatureMonitor"; }
  
  // Safety interface
  bool isUnsafe() override;
  void resetSafetyFlags() override;

private:
  // Singleton constructor
  TemperatureMonitor();
  
  // State machine for temperature operations
  enum class TempState {
    UNINITIALIZED,
    SCANNING,
    CONVERTING,
    READING,
    IDLE
  };
  
  // Core functionality
  void updateTemperatureReadings();
  void updateSystemStatus();
  void checkSafetyConditions();
  void analyzeTemperatureTrends();
  void sendStatusReports();
  void sendDiagnosticReports();
  
  // Sensor management
  bool readSingleSensor(uint8_t sensor_index);
  void handleSensorError(uint8_t sensor_index);
  
  // Safety and thermal protection
  void checkThermalLimits();
  void detectThermalRunaway();
  void engageThermalProtection();
  void disengageThermalProtection();
  
  // Trend analysis
  void updateTemperatureHistory(uint8_t sensor_index, float temperature);
  float calculateTemperatureTrend(uint8_t sensor_index);
  void updatePerformanceStatistics();
  
  // Configuration and state
  TemperatureMonitorConfig config_;
  TemperatureSystemStatus system_status_;
  TempState temp_state_;
  
  // Sensor management
  TemperatureSensorConfig sensor_configs_[kMaxTemperatureSensors];
  TemperatureSensorStatus sensor_status_[kMaxTemperatureSensors];
  bool sensor_configured_[kMaxTemperatureSensors];
  
  // Safety state tracking
  uint32_t warning_start_time_ms_;
  uint32_t critical_start_time_ms_;
  bool thermal_protection_engaged_;
  
  // Timing for periodic operations
  uint32_t last_status_report_time_ms_;
  uint32_t last_diagnostic_report_time_ms_;
  uint32_t last_sensor_scan_time_ms_;
  uint32_t last_safety_check_time_ms_;
  
  // Performance tracking
  uint32_t system_start_time_ms_;
  uint32_t total_system_readings_;
  uint32_t total_system_errors_;
  uint32_t last_performance_update_ms_;
};

} // namespace sigyn_teensy
