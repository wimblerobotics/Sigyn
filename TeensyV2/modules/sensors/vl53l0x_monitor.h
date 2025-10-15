// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

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
#include <cstdint>
#include <cmath>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"
#include <VL53L0X.h>  // Use Arduino library directly

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
    uint32_t measurement_timing_budget_us = 20000;   ///< Timing budget per measurement (default 20ms)
    float signal_rate_limit_mcps = 0.25f;            ///< Minimum signal rate for valid return (MCPS)
    uint32_t poll_interval_us = 20000;               ///< Desired polling cadence per sensor (microseconds)
    uint32_t initialization_retry_ms = 1000;         ///< Retry interval for failed sensors (milliseconds)
    uint32_t reinitialize_backoff_ms = 5000;         ///< Backoff after forced reset (milliseconds)

    uint16_t obstacle_threshold_mm = 200;            ///< Critical obstacle distance (mm)
    uint16_t warning_threshold_mm = 500;             ///< Warning distance (mm)
    uint16_t max_range_mm = 2000;                    ///< Maximum trusted range (mm)

    // Reporting intervals
    uint32_t status_report_interval_ms = 100;        ///< Interval for publishing status messages
    uint32_t diagnostic_report_interval_ms = 1000;   ///< Interval for publishing diagnostic statistics
    uint32_t statistics_window_ms = 1000;            ///< Rolling statistics window length

    // Filtering parameters
    float filter_slow_alpha = 0.25f;                 ///< Low-frequency smoothing factor (0-1)
    float filter_fast_alpha = 0.7f;                  ///< High-frequency smoothing factor (0-1)
    float filter_fast_delta_mm = 60.0f;              ///< Delta threshold to engage fast response (mm)

    // Error handling
    uint8_t max_consecutive_failures = 5;            ///< Consecutive invalid readings before reset
  };

  /**
   * @brief Individual sensor status and measurement data.
   */
  struct SensorStatus {
    uint16_t raw_distance_mm = 0;                    ///< Latest raw distance sample (mm)
    uint16_t filtered_distance_mm = 0;               ///< Filtered distance estimate (mm)

    // Status flags
    bool initialized = false;                        ///< Sensor passed initialization
    bool measurement_valid = false;                  ///< Latest reading is considered usable
    bool degraded_quality = false;                   ///< Reading is usable but degraded (e.g., out of range)
    bool communication_ok = false;                   ///< I2C communication healthy
    bool filtered_valid = false;                     ///< Filter has a valid estimate

    // Timing and performance
    uint32_t last_measurement_us = 0;                ///< Timestamp (µs) when sample was captured
    uint32_t last_update_us = 0;                     ///< Timestamp (µs) when monitor processed the sample
    uint32_t last_successful_read_us = 0;            ///< Timestamp (µs) of last usable measurement
    uint32_t measurement_age_us = 0;                 ///< Age (µs) of measurement at time of last publish

    // Counters
    uint32_t total_measurements = 0;                 ///< Lifetime total measurements processed
    uint32_t total_valid_measurements = 0;           ///< Lifetime usable measurements
    uint32_t total_errors = 0;                       ///< Lifetime invalid/error readings
    uint8_t consecutive_failures = 0;                ///< Consecutive invalid or error readings
  };

  /**
   * @brief System-wide sensor array status.
   */
  struct SensorArrayStatus {
    uint8_t total_sensors = 0;
    uint8_t initialized_sensors = 0;
    uint8_t active_sensors = 0;
    uint8_t sensors_with_obstacles = 0;

    uint16_t min_distance_mm = 65535;
    uint16_t max_distance_mm = 0;
    uint8_t min_distance_sensor = 255;
    bool any_obstacles = false;

    bool multiplexer_ok = false;
    bool system_ready = false;

    uint32_t total_measurements = 0;
    uint32_t total_valid_measurements = 0;
    uint32_t total_errors = 0;
    float system_measurement_rate_hz = 0.0f;
  };

  /**
   * @brief Sensor diagnostic counters for reporting and analysis.
   */
  struct SensorDiagnostics {
    uint32_t total_reads = 0;
    uint32_t ok = 0;
    uint32_t sigma_fail = 0;
    uint32_t signal_fail = 0;
    uint32_t min_range_fail = 0;
    uint32_t phase_fail = 0;
    uint32_t hw_fail = 0;
    uint32_t none = 0;
    uint32_t timeouts = 0;
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
    const VL53L0XConfig& getConfig() const { return config_; }

    // Configuration interface
    void updateConfig(const VL53L0XConfig& config);

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
      ERROR_RECOVERY
    };

    struct FilterState {
      float estimate = 0.0f;
      bool initialized = false;
    };

    // Core functionality
    void updateSensorSchedule();
    void attemptSensorInitialization(uint8_t sensor_index);
    bool initializeSingleSensor(uint8_t sensor_index);
    void updateArrayStatus(uint32_t now_us);
    void sendStatusReports();
    void sendDiagnosticReports();
    void detectObstacles();
    float applyAdaptiveFilter(uint8_t sensor_index, float sample_mm);

    // I2C multiplexer control
    void selectSensorChannel(uint8_t sensor_index);
    bool testMultiplexer();
    void resetMultiplexer();

    // Recovery and error handling
    void scheduleRecovery(uint8_t sensor_index, uint32_t now_ms);

    // Configuration and state
    VL53L0XConfig config_;
    SensorArrayStatus array_status_;

    // Sensor hardware and state
    SensorStatus sensor_status_[8];
    SensorState sensor_states_[8];
    bool sensor_enabled_[8];

    VL53L0X sensors_[8];  // Use Arduino library directly - it works!
    FilterState filter_state_[8];

    // Operational state
    uint32_t poll_cadence_us_[8];
    uint32_t next_poll_due_us_[8];
    uint32_t next_reinit_attempt_ms_[8];

    // Timing for periodic operations
    uint32_t last_status_report_time_ms_;
    uint32_t last_diagnostic_report_time_ms_;
    uint32_t last_array_status_update_us_;
    uint32_t system_start_time_us_;

    // Performance tracking
    uint32_t total_system_measurements_;
    uint32_t total_system_valid_measurements_;
    uint32_t total_system_errors_;

    bool multiplexer_available_;
    bool system_initialized_;
  };

}  // namespace sigyn_teensy
