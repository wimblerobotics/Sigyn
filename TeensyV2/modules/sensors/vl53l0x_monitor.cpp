// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file vl53l0x_monitor.cpp
 * @brief VL53L0X Time-of-Flight sensor monitoring implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "vl53l0x_monitor.h"

#include <cstdarg>

namespace {
  constexpr uint8_t kMaxSensors = 8;
  constexpr uint8_t kI2CMultiplexerEnablePin = 8;
  constexpr uint32_t kI2CClockFrequency = 400000;

  void snappend(char* buf, size_t buf_size, size_t& offset, const char* fmt, ...) {
    if (!buf || buf_size == 0 || offset >= buf_size) {
      return;
    }

    va_list args;
    va_start(args, fmt);
    const int written = vsnprintf(buf + offset, buf_size - offset, fmt, args);
    va_end(args);

    if (written <= 0) {
      return;
    }

    const size_t remaining = buf_size - offset;
    const size_t advance = (static_cast<size_t>(written) < remaining) ? static_cast<size_t>(written) : (remaining - 1);
    offset += advance;
  }

  inline uint32_t deltaMicros(uint32_t now, uint32_t earlier) {
    return now - earlier;
  }

  inline uint32_t deltaMillis(uint32_t now, uint32_t earlier) {
    return now - earlier;
  }

  inline float clampAlpha(float alpha) {
    if (alpha < 0.0f) return 0.0f;
    if (alpha > 1.0f) return 1.0f;
    return alpha;
  }
}  // namespace

namespace sigyn_teensy {

  VL53L0XMonitor& VL53L0XMonitor::getInstance() {
    static VL53L0XMonitor instance;
    return instance;
  }

  VL53L0XMonitor::VL53L0XMonitor()
    : Module(),
    config_(),
    array_status_(),
    last_status_report_time_ms_(0),
    last_diagnostic_report_time_ms_(0),
    last_array_status_update_us_(0),
    system_start_time_us_(0),
    total_system_measurements_(0),
    total_system_valid_measurements_(0),
    total_system_errors_(0),
    multiplexer_available_(false),
    system_initialized_(false) {

    array_status_.total_sensors = config_.enabled_sensors;
    array_status_.multiplexer_ok = false;

    for (uint8_t i = 0; i < kMaxSensors; ++i) {
      sensor_status_[i] = SensorStatus();
      sensor_states_[i] = (i < config_.enabled_sensors) ? SensorState::UNINITIALIZED : SensorState::ERROR_RECOVERY;
      sensor_enabled_[i] = (i < config_.enabled_sensors);
      filter_state_[i] = FilterState();
      poll_cadence_us_[i] = config_.poll_interval_us;
      next_poll_due_us_[i] = 0;
      next_reinit_attempt_ms_[i] = 0;
    }
  }

  void VL53L0XMonitor::updateConfig(const VL53L0XConfig& config) {
    config_ = config;
    if (config_.max_sensors > kMaxSensors) {
      config_.max_sensors = kMaxSensors;
    }
    if (config_.enabled_sensors > kMaxSensors) {
      config_.enabled_sensors = kMaxSensors;
    }
    array_status_.total_sensors = config_.enabled_sensors;

    const uint32_t now_us = micros();
    for (uint8_t i = 0; i < kMaxSensors; ++i) {
      sensor_enabled_[i] = (i < config_.enabled_sensors);
      poll_cadence_us_[i] = config_.poll_interval_us;
      next_poll_due_us_[i] = now_us;
      filter_state_[i] = FilterState();
      next_reinit_attempt_ms_[i] = 0;
    }
  }

  void VL53L0XMonitor::setup() {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Starting VL53L0X monitor initialization");

    pinMode(kI2CMultiplexerEnablePin, OUTPUT);
    digitalWrite(kI2CMultiplexerEnablePin, HIGH);

    Wire.begin();
    Wire.setClock(kI2CClockFrequency);

    multiplexer_available_ = testMultiplexer();
    array_status_.multiplexer_ok = multiplexer_available_;

    if (!multiplexer_available_) {
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "I2C multiplexer not detected");
      return;
    }

    updateSensorSchedule();

    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      if (!sensor_enabled_[i]) {
        sensor_states_[i] = SensorState::ERROR_RECOVERY;
        continue;
      }
      attemptSensorInitialization(i);
      delay(40);  // Stagger configuration to avoid bus contention
    }

    last_status_report_time_ms_ = millis();
    last_diagnostic_report_time_ms_ = millis();
    last_array_status_update_us_ = micros();
    system_start_time_us_ = micros();
    system_initialized_ = true;

    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "VL53L0X monitor initialization complete");
  }

  void VL53L0XMonitor::loop() {
    if (!system_initialized_) {
      return;
    }

    const uint32_t now_us = micros();
    const uint32_t now_ms = millis();
    bool updated_measurement = false;

    for (uint8_t i = 0; i < config_.enabled_sensors && i < kMaxSensors; ++i) {
      if (!sensor_enabled_[i]) {
        continue;
      }

      if (!sensor_status_[i].initialized) {
        if (now_ms >= next_reinit_attempt_ms_[i]) {
          attemptSensorInitialization(i);
        }
        continue;
      }

      // Read ALL sensors every cycle - continuous mode has data ready
      // Check if data is ready (non-blocking) before reading
      selectSensorChannel(i);

      VL53L0X& sensor = sensors_[i];

      // Check interrupt status to see if new data is ready (non-blocking)
      uint8_t interrupt_status = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);
      if ((interrupt_status & 0x07) == 0) {
        // No new data ready yet, skip this sensor for now
        continue;
      }

      // Data is ready - read the range
      uint16_t distance_mm = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);

      // Clear the interrupt
      sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

      // Got a valid reading - process it
      sensor_status_[i].communication_ok = true;
      sensor_status_[i].last_update_us = now_us;
      sensor_status_[i].last_measurement_us = now_us;
      sensor_status_[i].raw_distance_mm = distance_mm;
      sensor_status_[i].total_measurements++;
      total_system_measurements_++;

      // VL53L0X returns 8190-8191 when target is out of range - this is VALID
      // It means "no target detected" or "target beyond max range"
      bool is_out_of_range = (distance_mm >= 8190);

      // ALL readings from the sensor are valid, even out-of-range ones
      bool measurement_valid = true;

      sensor_status_[i].measurement_valid = measurement_valid;
      sensor_status_[i].degraded_quality = is_out_of_range;

      if (!is_out_of_range) {
        // Normal in-range reading - apply filter
        uint16_t sample_mm = distance_mm;
        const float filtered = applyAdaptiveFilter(i, static_cast<float>(sample_mm));
        sensor_status_[i].filtered_distance_mm = static_cast<uint16_t>(filtered + 0.5f);
        sensor_status_[i].filtered_valid = filter_state_[i].initialized;
        sensor_status_[i].last_successful_read_us = now_us;
        sensor_status_[i].total_valid_measurements++;
        total_system_valid_measurements_++;
        sensor_status_[i].consecutive_failures = 0;
      }
      else {
        // Out of range reading (8190-8191) - valid but no target detected
        // Don't filter these, just mark as valid with no filtered value
        sensor_status_[i].filtered_valid = false;
        sensor_status_[i].last_successful_read_us = now_us;
        sensor_status_[i].total_valid_measurements++;
        total_system_valid_measurements_++;
        sensor_status_[i].consecutive_failures = 0;
      }

      // Move to next sensor - no need to schedule next poll time
      // since we read all sensors every loop cycle
      updated_measurement = true;
    }

    if (updated_measurement || deltaMicros(now_us, last_array_status_update_us_) >= config_.poll_interval_us) {
      updateArrayStatus(now_us);
      last_array_status_update_us_ = now_us;
    }

    detectObstacles();

    if (deltaMillis(now_ms, last_status_report_time_ms_) >= config_.status_report_interval_ms) {
      sendStatusReports();
      last_status_report_time_ms_ = now_ms;
    }

    if (deltaMillis(now_ms, last_diagnostic_report_time_ms_) >= config_.diagnostic_report_interval_ms) {
      sendDiagnosticReports();
      last_diagnostic_report_time_ms_ = now_ms;
    }
  }

  bool VL53L0XMonitor::isUnsafe() {
    return false; //#####return array_status_.any_obstacles && (array_status_.min_distance_mm < config_.obstacle_threshold_mm);
  }

  void VL53L0XMonitor::resetSafetyFlags() {
    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      sensor_status_[i].degraded_quality = false;
    }
    array_status_.any_obstacles = false;
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Safety flags reset");
  }

  float VL53L0XMonitor::getDistanceMm(uint8_t sensor_index) const {
    if (sensor_index >= config_.enabled_sensors) {
      return NAN;
    }

    const SensorStatus& status = sensor_status_[sensor_index];
    if (status.measurement_valid && status.filtered_valid) {
      return static_cast<float>(status.filtered_distance_mm);
    }

    return NAN;
  }

  bool VL53L0XMonitor::isSensorReady(uint8_t sensor_index) const {
    if (sensor_index >= config_.enabled_sensors) {
      return false;
    }
    const SensorStatus& status = sensor_status_[sensor_index];
    return status.initialized && status.communication_ok;
  }

  bool VL53L0XMonitor::isObstacleDetected(uint8_t sensor_index) const {
    if (sensor_index >= config_.enabled_sensors) {
      return false;
    }

    const SensorStatus& status = sensor_status_[sensor_index];
    if (!status.filtered_valid) {
      return false;
    }

    return status.filtered_distance_mm < config_.obstacle_threshold_mm;
  }

  const SensorStatus& VL53L0XMonitor::getSensorStatus(uint8_t sensor_index) const {
    static SensorStatus invalid_status;
    if (sensor_index >= config_.enabled_sensors) {
      return invalid_status;
    }
    return sensor_status_[sensor_index];
  }

  void VL53L0XMonitor::reinitializeSensor(uint8_t sensor_index) {
    if (sensor_index >= config_.enabled_sensors) {
      return;
    }

    sensor_status_[sensor_index] = SensorStatus();
    filter_state_[sensor_index] = FilterState();
    sensors_[sensor_index].stopContinuous();
    sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
    next_reinit_attempt_ms_[sensor_index] = millis();
  }

  void VL53L0XMonitor::reinitializeAll() {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Reinitializing all VL53L0X sensors");
    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      reinitializeSensor(i);
      attemptSensorInitialization(i);
    }
  }

  void VL53L0XMonitor::enableSensor(uint8_t sensor_index, bool enable) {
    if (sensor_index >= kMaxSensors) {
      return;
    }

    sensor_enabled_[sensor_index] = enable;
    if (enable) {
      sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
      next_reinit_attempt_ms_[sensor_index] = millis();
    }
    else {
      sensors_[sensor_index].stopContinuous();
      sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
      sensor_status_[sensor_index].initialized = false;
      sensor_status_[sensor_index].communication_ok = false;
      filter_state_[sensor_index] = FilterState();
    }
  }

  void VL53L0XMonitor::updateSensorSchedule() {
    const uint32_t now_us = micros();
    for (uint8_t i = 0; i < kMaxSensors; ++i) {
      poll_cadence_us_[i] = config_.poll_interval_us;
      next_poll_due_us_[i] = now_us;
    }
  }

  void VL53L0XMonitor::attemptSensorInitialization(uint8_t sensor_index) {
    sensor_states_[sensor_index] = SensorState::INITIALIZING;
    if (!initializeSingleSensor(sensor_index)) {
      scheduleRecovery(sensor_index, millis());
    }
  }

  bool VL53L0XMonitor::initializeSingleSensor(uint8_t sensor_index) {
    if (!multiplexer_available_) {
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Cannot initialize sensor, multiplexer unavailable");
      return false;
    }

    // Select sensor channel on multiplexer
    selectSensorChannel(sensor_index);

    // Set up the sensor using Arduino library - PROVEN TO WORK
    VL53L0X& sensor = sensors_[sensor_index];
    sensor.setBus(&Wire);
    sensor.setTimeout(500);  // 500ms timeout

    // Initialize sensor
    if (!sensor.init()) {
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "init failed sensor=%u", static_cast<unsigned int>(sensor_index));
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
      sensor_status_[sensor_index].initialized = false;
      sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
      return false;
    }

    // Configure for good accuracy (from working code)
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

    // Set timing budget
    if (!sensor.setMeasurementTimingBudget(config_.measurement_timing_budget_us)) {
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "Failed to set timing budget for sensor %u", static_cast<unsigned int>(sensor_index));
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      // Continue anyway - not fatal
    }

    // Start continuous back-to-back measurements
    sensor.startContinuous();

    // Wait for sensor to stabilize
    delay(50);

    // Test reading
    uint16_t test_distance = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred()) {
      char msg[128] = {0};
      snprintf(msg, sizeof(msg), "Sensor %u test reading failed - timeout", static_cast<unsigned int>(sensor_index));
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      sensor_status_[sensor_index].initialized = false;
      sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
      return false;
    }

    // Initialize filter
    filter_state_[sensor_index].estimate = static_cast<float>(test_distance);
    filter_state_[sensor_index].initialized = true;

    // Mark sensor as ready
    sensor_status_[sensor_index].initialized = true;
    sensor_status_[sensor_index].communication_ok = true;
    sensor_status_[sensor_index].raw_distance_mm = test_distance;
    sensor_states_[sensor_index] = SensorState::READY;
    next_poll_due_us_[sensor_index] = micros() + config_.poll_interval_us;
    next_reinit_attempt_ms_[sensor_index] = millis() + config_.reinitialize_backoff_ms;

    char msg[128] = {0};
    snprintf(msg, sizeof(msg), "sensor=%u,init=success,test_dist=%u", static_cast<unsigned int>(sensor_index),
             static_cast<unsigned int>(test_distance));
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
    return true;
  }

  void VL53L0XMonitor::updateArrayStatus(uint32_t now_us) {
    array_status_.initialized_sensors = 0;
    array_status_.active_sensors = 0;
    array_status_.sensors_with_obstacles = 0;
    array_status_.min_distance_mm = 65535;
    array_status_.max_distance_mm = 0;
    array_status_.min_distance_sensor = 255;
    array_status_.any_obstacles = false;

    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      if (!sensor_enabled_[i]) {
        continue;
      }

      const SensorStatus& status = sensor_status_[i];
      if (status.initialized) {
        array_status_.initialized_sensors++;
      }
      if (status.communication_ok) {
        array_status_.active_sensors++;
      }

      if (status.measurement_valid && status.filtered_valid) {
        const uint16_t distance = status.filtered_distance_mm;

        if (distance < array_status_.min_distance_mm) {
          array_status_.min_distance_mm = distance;
          array_status_.min_distance_sensor = i;
        }
        if (distance > array_status_.max_distance_mm) {
          array_status_.max_distance_mm = distance;
        }

        if (distance < config_.obstacle_threshold_mm) {
          array_status_.sensors_with_obstacles++;
          array_status_.any_obstacles = true;
        }
      }
    }

    array_status_.system_ready = multiplexer_available_ && (array_status_.initialized_sensors > 0);
    array_status_.total_sensors = config_.enabled_sensors;
    array_status_.total_measurements = total_system_measurements_;
    array_status_.total_valid_measurements = total_system_valid_measurements_;
    array_status_.total_errors = total_system_errors_;

    if (system_start_time_us_ == 0) {
      system_start_time_us_ = now_us;
    }
    const float elapsed_s = (now_us - system_start_time_us_) / 1'000'000.0f;
    if (elapsed_s > 0.1f) {
      array_status_.system_measurement_rate_hz = total_system_valid_measurements_ / elapsed_s;
    }
  }

  void VL53L0XMonitor::sendStatusReports() {
    const uint32_t now_us = micros();

    char payload[1024] = {0};
    size_t off = 0;

    snappend(payload, sizeof(payload), off,
             "{\"total_sensors\":%u,\"active_sensors\":%u,\"min_distance\":%u,\"max_distance\":%u,\"obstacles\":%s,\"distances\":[",
             static_cast<unsigned int>(array_status_.total_sensors), static_cast<unsigned int>(array_status_.active_sensors),
             static_cast<unsigned int>(array_status_.min_distance_mm), static_cast<unsigned int>(array_status_.max_distance_mm),
             array_status_.any_obstacles ? "true" : "false");

    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      if (i > 0) {
        snappend(payload, sizeof(payload), off, ",");
      }

      const SensorStatus& status = sensor_status_[i];
      snappend(payload, sizeof(payload), off, "{\"id\":%u,\"mm\":", static_cast<unsigned int>(i));

      if (status.measurement_valid && status.filtered_valid) {
        snappend(payload, sizeof(payload), off, "%u", static_cast<unsigned int>(status.filtered_distance_mm));
      } else {
        snappend(payload, sizeof(payload), off, "null");
      }

      snappend(payload, sizeof(payload), off, ",\"raw\":");
      if (status.measurement_valid || status.degraded_quality) {
        snappend(payload, sizeof(payload), off, "%u", static_cast<unsigned int>(status.raw_distance_mm));
      } else {
        snappend(payload, sizeof(payload), off, "null");
      }

      snappend(payload, sizeof(payload), off, ",\"age_us\":%lu,\"degraded\":%s}",
               static_cast<unsigned long>(deltaMicros(now_us, status.last_measurement_us)),
               status.degraded_quality ? "true" : "false");
    }

    snappend(payload, sizeof(payload), off, "]}");

    SerialManager::getInstance().sendMessage("VL53L0X", payload);
  }

  void VL53L0XMonitor::sendDiagnosticReports() {
    char payload[1024] = {0};
    size_t off = 0;

    snappend(payload, sizeof(payload), off, "{\"window_ms\":%lu,\"multiplexer_ok\":%s,\"sensors\":[",
             static_cast<unsigned long>(config_.diagnostic_report_interval_ms), multiplexer_available_ ? "true" : "false");

    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      if (i > 0) {
        snappend(payload, sizeof(payload), off, ",");
      }

      snappend(payload, sizeof(payload), off,
               "{\"id\":%u,\"total_measurements\":%lu,\"valid_measurements\":%lu,\"errors\":%lu,\"consecutive_failures\":%u,\"initialized\":%s}",
               static_cast<unsigned int>(i), static_cast<unsigned long>(sensor_status_[i].total_measurements),
               static_cast<unsigned long>(sensor_status_[i].total_valid_measurements),
               static_cast<unsigned long>(sensor_status_[i].total_errors),
               static_cast<unsigned int>(sensor_status_[i].consecutive_failures),
               sensor_status_[i].initialized ? "true" : "false");
    }

    snappend(payload, sizeof(payload), off, "]}");
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), payload);
  }

  void VL53L0XMonitor::detectObstacles() {
    static uint32_t last_alert_us = 0;
    const uint32_t now_us = micros();

    for (uint8_t i = 0; i < config_.enabled_sensors; ++i) {
      if (!sensor_enabled_[i]) {
        continue;
      }

      const SensorStatus& status = sensor_status_[i];
      if (!status.filtered_valid) {
        continue;
      }

      if (status.filtered_distance_mm < config_.obstacle_threshold_mm) {
        if (deltaMicros(now_us, last_alert_us) > 200000U) {
          char message[160] = {0};
          snprintf(message, sizeof(message), "active=true,source=VL53L0X_OBSTACLE,sensor=%u,distance_mm=%u,degraded=%s",
                   static_cast<unsigned int>(i), static_cast<unsigned int>(status.filtered_distance_mm),
                   status.degraded_quality ? "true" : "false");
          SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), message);
          last_alert_us = now_us;
        }
      }
    }
  }

  float VL53L0XMonitor::applyAdaptiveFilter(uint8_t sensor_index, float sample_mm) {
    FilterState& state = filter_state_[sensor_index];

    const float slow_alpha = clampAlpha(config_.filter_slow_alpha);
    const float fast_alpha = clampAlpha(config_.filter_fast_alpha);
    const float delta_threshold = (config_.filter_fast_delta_mm <= 0.0f) ? 1.0f : config_.filter_fast_delta_mm;

    if (!state.initialized) {
      state.estimate = sample_mm;
      state.initialized = true;
      return state.estimate;
    }

    const float delta = fabsf(sample_mm - state.estimate);
    const float alpha = (delta >= delta_threshold) ? fast_alpha : slow_alpha;
    state.estimate += alpha * (sample_mm - state.estimate);
    return state.estimate;
  }

  void VL53L0XMonitor::selectSensorChannel(uint8_t sensor_index) {
    Wire.beginTransmission(config_.i2c_multiplexer_address);
    Wire.write(1 << sensor_index);
    Wire.endTransmission();
    delayMicroseconds(100);
  }

  bool VL53L0XMonitor::testMultiplexer() {
    Wire.beginTransmission(config_.i2c_multiplexer_address);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
  }

  void VL53L0XMonitor::scheduleRecovery(uint8_t sensor_index, uint32_t now_ms) {
    sensors_[sensor_index].stopContinuous();
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
    sensor_status_[sensor_index].initialized = false;
    sensor_status_[sensor_index].communication_ok = false;
    next_reinit_attempt_ms_[sensor_index] = now_ms + config_.reinitialize_backoff_ms;
    char msg[96] = {0};
    snprintf(msg, sizeof(msg), "sensor=%u,recovery_scheduled=true", static_cast<unsigned int>(sensor_index));
    SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
  }

}  // namespace sigyn_teensy
