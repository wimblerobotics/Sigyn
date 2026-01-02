// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file temperature_monitor.cpp
 * @brief Temperature monitoring implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "temperature_monitor.h"
#include "analog_reader_hw.h"
#include "../../common/core/serial_manager.h"
#include "../../modules/safety/safety_coordinator.h"

namespace sigyn_teensy {

// Hardware configuration constants
constexpr uint8_t ANALOG_RESOLUTION = 12;  // 12-bit ADC resolution for better precision
constexpr float REFERENCE_VOLTAGE = 3.3f;  // 3.3V reference voltage (Teensy 4.1 default)
constexpr uint32_t ADC_MAX_VALUE = 4096;   // 2^12 for 12-bit resolution

// TMP36 sensor constants
constexpr float TMP36_OFFSET_MV = 500.0f;      // TMP36 offset in millivolts (500mV at 0°C)
constexpr float TMP36_SCALE_MV_PER_C = 10.0f;  // TMP36 scale factor (10mV/°C)

// Static hardware reader for production use (no heap allocation)
static AnalogReaderHW hardware_reader;

TemperatureMonitor& TemperatureMonitor::getInstance() {
  static TemperatureMonitor instance;
  return instance;
}

TemperatureMonitor::TemperatureMonitor()
    : Module(),
      config_(),
      system_status_(),
      temp_state_(TempState::UNINITIALIZED),
      warning_start_time_ms_(0),
      critical_start_time_ms_(0),
      thermal_protection_engaged_(false),
      last_status_report_time_ms_(0),
      last_diagnostic_report_time_ms_(0),
      last_sensor_scan_time_ms_(0),
      last_safety_check_time_ms_(0),
      system_start_time_ms_(0),
      total_system_readings_(0),
      total_system_errors_(0),
      last_performance_update_ms_(0),
      analog_reader_(&hardware_reader) {  // Initialize with hardware reader
  // Configure for analog sensors instead of OneWire
  config_.max_sensors = kMaxTemperatureSensors;  // Support up to max analog temperature sensors

  // Initialize sensor configurations for left and right motor
  sensor_configs_[0].sensor_name = "LeftMotor";
  sensor_configs_[0].location = "Left Motor";
  sensor_configs_[0].analog_pin = 25;  // Left motor temperature pin
  sensor_configs_[0].critical_high_temp = 85.0f;
  sensor_configs_[0].warning_high_temp = 70.0f;
  sensor_configs_[0].warning_low_temp = 5.0f;
  sensor_configs_[0].critical_low_temp = 0.0f;
  sensor_configs_[0].enabled = true;
  sensor_configs_[0].safety_critical = true;
  sensor_configured_[0] = true;

  sensor_configs_[1].sensor_name = "RightMotor";
  sensor_configs_[1].location = "Right Motor";
  sensor_configs_[1].analog_pin = 26;  // Right motor temperature pin
  sensor_configs_[1].critical_high_temp = 85.0f;
  sensor_configs_[1].warning_high_temp = 70.0f;
  sensor_configs_[1].warning_low_temp = 5.0f;
  sensor_configs_[1].critical_low_temp = 0.0f;
  sensor_configs_[1].enabled = true;
  sensor_configs_[1].safety_critical = true;
  sensor_configured_[1] = true;

  // Mark other sensors as unconfigured
  for (uint8_t i = kDefaultSensorsConfigured; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    sensor_configured_[i] = false;
  }

  // Initialize system status
  system_status_.total_sensors = kDefaultSensorsConfigured;
}

void TemperatureMonitor::setup() {
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Starting initialization");
  SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "Module registered and setup() called");

  system_start_time_ms_ = millis();

  // Set analog resolution for all analog pins
  analog_reader_->setAnalogResolution(ANALOG_RESOLUTION);

  // Configure temperature sensor pins as inputs
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (sensor_configured_[i] && sensor_configs_[i].analog_pin != 255) {
      pinMode(sensor_configs_[i].analog_pin, INPUT);

      String msg =
          "Configured pin " + String(sensor_configs_[i].analog_pin) + " for sensor " + sensor_configs_[i].sensor_name;
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());
    }
  }

  // Initialize sensor status
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (sensor_configured_[i]) {
      sensor_status_[i].sensor_present = true;
      sensor_status_[i].communication_ok = true;
      sensor_status_[i].resolution = ANALOG_RESOLUTION;
      sensor_status_[i].last_valid_reading_time_ms = 0;

      // Initialize temperature history
      for (uint8_t j = 0; j < 50; j++) {
        sensor_status_[i].temperature_history[j] = NAN;
      }
      sensor_status_[i].history_index = 0;
    }
  }

  system_status_.active_sensors = getSensorCount();

  temp_state_ = TempState::IDLE;

  // Initialize timing
  uint32_t now = millis();
  last_status_report_time_ms_ = now;
  last_diagnostic_report_time_ms_ = now;
  last_safety_check_time_ms_ = now;

  SerialManager::getInstance().sendDiagnosticMessage(
      "INFO", name(), ("Setup complete - " + String(getSensorCount()) + " analog sensors ready").c_str());
}

void TemperatureMonitor::loop() {
  uint32_t now = millis();

  // Update temperature readings
  updateTemperatureReadings();

  // Update system status
  updateSystemStatus();

  // Periodic safety checks
  if (now - last_safety_check_time_ms_ >= 100) {  // 10Hz safety checks
    checkSafetyConditions();
    last_safety_check_time_ms_ = now;
  }

  // Periodic status reporting
  if (now - last_status_report_time_ms_ >= config_.status_report_interval_ms) {
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }

  // Periodic diagnostic reporting
  if (now - last_diagnostic_report_time_ms_ >= config_.diagnostic_report_interval_ms) {
    sendDiagnosticReports();
    last_diagnostic_report_time_ms_ = now;
  }

  // Update performance statistics
  if (now - last_performance_update_ms_ >= 1000) {  // Update every second
    updatePerformanceStatistics();
    last_performance_update_ms_ = now;
  }
}

uint8_t TemperatureMonitor::getSensorCount() const {
  uint8_t count = 0;
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (sensor_configured_[i]) {
      count++;
    }
  }
  return count;
}

bool TemperatureMonitor::isUnsafe() {
  // Check if any sensor is in critical state
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (sensor_configured_[i] && sensor_configs_[i].safety_critical) {
      if (sensor_status_[i].critical_high || sensor_status_[i].critical_low || sensor_status_[i].thermal_runaway) {
        return true;
      }
    }
  }

  return system_status_.system_thermal_critical || thermal_protection_engaged_;
}

void TemperatureMonitor::resetSafetyFlags() {
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    sensor_status_[i].critical_high = false;
    sensor_status_[i].critical_low = false;
    sensor_status_[i].warning_high = false;
    sensor_status_[i].warning_low = false;
    sensor_status_[i].thermal_runaway = false;
  }

  system_status_.system_thermal_warning = false;
  system_status_.system_thermal_critical = false;
  thermal_protection_engaged_ = false;
  warning_start_time_ms_ = 0;
  critical_start_time_ms_ = 0;

  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Safety flags reset");
}

float TemperatureMonitor::getTemperature(uint8_t sensor_index, bool fahrenheit) const {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors ||
      !sensor_configured_[sensor_index]) {
    return NAN;
  }

  if (!sensor_status_[sensor_index].reading_valid) {
    return NAN;
  }

  if (fahrenheit) {
    return sensor_status_[sensor_index].temperature_f;
  } else {
    return sensor_status_[sensor_index].temperature_c;
  }
}

bool TemperatureMonitor::isSensorValid(uint8_t sensor_index) const {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return false;
  }

  return sensor_status_[sensor_index].sensor_present && sensor_status_[sensor_index].communication_ok &&
         sensor_status_[sensor_index].reading_valid;
}

bool TemperatureMonitor::isTemperatureCritical(uint8_t sensor_index) const {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors ||
      !sensor_configured_[sensor_index]) {
    return false;
  }

  return sensor_status_[sensor_index].critical_high || sensor_status_[sensor_index].critical_low ||
         sensor_status_[sensor_index].thermal_runaway;
}

const TemperatureSensorStatus& TemperatureMonitor::getSensorStatus(uint8_t sensor_index) const {
  static TemperatureSensorStatus invalid_status;

  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return invalid_status;
  }

  return sensor_status_[sensor_index];
}

void TemperatureMonitor::configureSensor(uint8_t sensor_index, const TemperatureSensorConfig& sensor_config) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return;
  }

  sensor_configs_[sensor_index] = sensor_config;
  sensor_configured_[sensor_index] = true;

  String msg = "Configured sensor " + String(sensor_index) + " (" + sensor_config.sensor_name + ")";
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());
}

void TemperatureMonitor::scanForSensors() {
  // For analog sensors, this is always successful since they're directly connected
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Analog sensors always present - scan complete");
}

void TemperatureMonitor::calibrateSensor(uint8_t sensor_index, float reference_temp) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return;
  }

  // For TMP36 sensors, calibration would involve adjusting the conversion formula
  // This is a placeholder for future calibration implementation
  String msg = "Calibration requested for sensor " + String(sensor_index) + " with reference " +
               String(reference_temp, 1) + "°C";
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());
}

void TemperatureMonitor::resetSensorStatistics(uint8_t sensor_index) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return;
  }

  sensor_status_[sensor_index].total_readings = 0;
  sensor_status_[sensor_index].error_count = 0;
  sensor_status_[sensor_index].max_temperature = -273.15f;
  sensor_status_[sensor_index].min_temperature = 1000.0f;
  sensor_status_[sensor_index].reading_frequency_hz = 0.0f;
  sensor_status_[sensor_index].last_valid_reading_time_ms = 0;

  // Clear temperature history
  for (uint8_t i = 0; i < 50; i++) {
    sensor_status_[sensor_index].temperature_history[i] = NAN;
  }
  sensor_status_[sensor_index].history_index = 0;
  sensor_status_[sensor_index].temperature_trend = 0.0f;
}

void TemperatureMonitor::resetSystemStatistics() {
  total_system_readings_ = 0;
  total_system_errors_ = 0;
  system_status_.total_readings = 0;
  system_status_.total_errors = 0;
  system_status_.system_reading_rate_hz = 0.0f;
  system_status_.time_in_warning_ms = 0;
  system_status_.time_in_critical_ms = 0;

  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    resetSensorStatistics(i);
  }

  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "All statistics reset");
}

void TemperatureMonitor::updateTemperatureReadings() {
  uint32_t now = millis();

  // Read all configured temperature sensors
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i] || !sensor_configs_[i].enabled) {
      continue;
    }

    // Check if it's time to read this sensor
    if (now - sensor_status_[i].last_reading_time_ms < sensor_configs_[i].read_interval_ms) {
      continue;
    }

    if (readSingleSensor(i)) {
      updateTemperatureHistory(i, sensor_status_[i].temperature_c);
      total_system_readings_++;
      sensor_status_[i].total_readings++;
      sensor_status_[i].last_valid_reading_time_ms = now;
    } else {
      total_system_errors_++;
      sensor_status_[i].error_count++;
    }

    sensor_status_[i].last_reading_time_ms = now;
  }
}

bool TemperatureMonitor::readSingleSensor(uint8_t sensor_index) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return false;
  }

  uint8_t pin = sensor_configs_[sensor_index].analog_pin;
  if (pin == 255) {
    // Pin not configured
    sensor_status_[sensor_index].reading_valid = false;
    sensor_status_[sensor_index].communication_ok = false;
    return false;
  }

  // Take multiple readings and average them to reduce noise
  uint32_t raw_sum = 0;
  const uint8_t num_samples = 8;

  for (uint8_t sample = 0; sample < num_samples; sample++) {
    raw_sum += analog_reader_->readAnalog(pin);
    delayMicroseconds(100);  // Small delay between samples
  }

  int16_t raw_value = raw_sum / num_samples;

  // Convert to voltage (in millivolts)
  float voltage_mv = (raw_value * REFERENCE_VOLTAGE * 1000.0f) / ADC_MAX_VALUE;

  // Convert voltage to temperature using TMP36 formula
  // TMP36: Temperature (°C) = (Voltage_mV - 500mV) / 10mV/°C
  float temperature_c = (voltage_mv - TMP36_OFFSET_MV) / TMP36_SCALE_MV_PER_C;

  // Validate temperature reading (reasonable range for motor temperatures)
  if (temperature_c < -40.0f || temperature_c > 150.0f) {
    sensor_status_[sensor_index].reading_valid = false;
    sensor_status_[sensor_index].communication_ok = false;
    return false;
  }

  // Apply simple moving average filter to smooth readings
  float filtered_temp = temperature_c;
  if (sensor_status_[sensor_index].reading_valid) {
    // Simple exponential moving average: new_value = alpha * new_reading + (1 - alpha) * old_value
    // Alpha = 0.7 for fast response (~2 seconds to converge at 1Hz sampling)
    float alpha = 0.7f;
    filtered_temp = alpha * temperature_c + (1.0f - alpha) * sensor_status_[sensor_index].temperature_c;
  }

  // Update sensor status
  sensor_status_[sensor_index].temperature_c = filtered_temp;
  sensor_status_[sensor_index].temperature_f = (filtered_temp * 9.0f / 5.0f) + 32.0f;
  sensor_status_[sensor_index].reading_valid = true;
  sensor_status_[sensor_index].communication_ok = true;

  // Update min/max temperatures
  if (filtered_temp > sensor_status_[sensor_index].max_temperature) {
    sensor_status_[sensor_index].max_temperature = filtered_temp;
  }
  if (filtered_temp < sensor_status_[sensor_index].min_temperature) {
    sensor_status_[sensor_index].min_temperature = filtered_temp;
  }

  return true;
}

void TemperatureMonitor::updateSystemStatus() {
  uint8_t active_count = 0;
  uint8_t warning_count = 0;
  uint8_t critical_count = 0;
  float total_temp = 0.0f;
  float highest_temp = -273.15f;
  float lowest_temp = 1000.0f;
  uint8_t hottest_sensor = 255;
  uint8_t coldest_sensor = 255;

  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }

    active_count++;
    float temp = sensor_status_[i].temperature_c;
    total_temp += temp;

    if (temp > highest_temp) {
      highest_temp = temp;
      hottest_sensor = i;
    }
    if (temp < lowest_temp) {
      lowest_temp = temp;
      coldest_sensor = i;
    }

    if (sensor_status_[i].warning_high || sensor_status_[i].warning_low) {
      warning_count++;
    }
    if (sensor_status_[i].critical_high || sensor_status_[i].critical_low) {
      critical_count++;
    }
  }

  system_status_.active_sensors = active_count;
  system_status_.sensors_in_warning = warning_count;
  system_status_.sensors_in_critical = critical_count;

  if (active_count > 0) {
    system_status_.average_temperature = total_temp / active_count;
    system_status_.highest_temperature = highest_temp;
    system_status_.lowest_temperature = lowest_temp;
    system_status_.hottest_sensor = hottest_sensor;
    system_status_.coldest_sensor = coldest_sensor;

    // Update system thermal status
    system_status_.system_thermal_warning = (highest_temp > config_.system_warning_temp);
    system_status_.system_thermal_critical = (highest_temp > config_.system_critical_temp);
  }

  system_status_.total_readings = total_system_readings_;
  system_status_.total_errors = total_system_errors_;
}

void TemperatureMonitor::checkSafetyConditions() {
  static uint32_t last_safety_diag_time_ms = 0;
  const uint32_t now = millis();
  const bool can_log = (now - last_safety_diag_time_ms) >= 1000;
  if (can_log) {
    last_safety_diag_time_ms = now;
  }

  bool any_warning = false;
  bool any_critical = false;
  bool any_degraded = false;
  bool thermal_runaway_detected = false;

  int8_t first_warning_idx = -1;
  int8_t first_critical_idx = -1;
  int8_t first_degraded_idx = -1;

  // Sensor timeout / invalid handling: temperature sensor failures are DEGRADED (not E-stop).
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i]) {
      continue;
    }
    if (!sensor_configs_[i].enabled || !sensor_configs_[i].safety_critical) {
      continue;
    }
    const uint32_t timeout_ms = sensor_configs_[i].fault_timeout_ms;
    if (timeout_ms == 0) {
      continue;
    }
    const uint32_t last_valid = sensor_status_[i].last_valid_reading_time_ms;
    const bool never_valid = (last_valid == 0);
    const uint32_t age_ms = never_valid ? (now - system_start_time_ms_) : (now - last_valid);
    if (age_ms >= timeout_ms) {
      any_degraded = true;
      if (first_degraded_idx < 0) {
        first_degraded_idx = static_cast<int8_t>(i);
      }
      if (can_log) {
        char msg[256];
        snprintf(msg, sizeof(msg), "DEGRADED temp sensor: sensor=%s idx=%u age_ms=%lu timeout_ms=%lu", 
                 sensor_configs_[i].sensor_name.c_str(), i, (unsigned long)age_ms, (unsigned long)timeout_ms);
        SerialManager::getInstance().sendDiagnosticMessage("WARNING", name(), msg);
      }
    }
  }

  // Threshold checks (only for valid readings)
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }

    float temp = sensor_status_[i].temperature_c;
    const TemperatureSensorConfig& config = sensor_configs_[i];

    // Check temperature thresholds
    sensor_status_[i].critical_high = (temp >= config.critical_high_temp);
    sensor_status_[i].warning_high = (temp >= config.warning_high_temp && temp < config.critical_high_temp);
    sensor_status_[i].warning_low = (temp <= config.warning_low_temp && temp > config.critical_low_temp);
    sensor_status_[i].critical_low = (temp <= config.critical_low_temp);

    if (sensor_status_[i].warning_high || sensor_status_[i].warning_low) {
      any_warning = true;
      if (first_warning_idx < 0) {
        first_warning_idx = static_cast<int8_t>(i);
      }
    }
    if (sensor_status_[i].critical_high || sensor_status_[i].critical_low) {
      any_critical = true;
      if (first_critical_idx < 0) {
        first_critical_idx = static_cast<int8_t>(i);
      }
    }

    // Log safety violations (rate-limited)
    if (can_log && (sensor_status_[i].critical_high || sensor_status_[i].critical_low)) {
      char msg[256];
      snprintf(msg, sizeof(msg), "CRITICAL temp: sensor=%s idx=%u temp=%.1fC (crit_high>=%.1fC crit_low<=%.1fC)",
               config.sensor_name.c_str(), i, temp, config.critical_high_temp, config.critical_low_temp);
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
    }

    if (can_log && (sensor_status_[i].warning_high || sensor_status_[i].warning_low) &&
        !(sensor_status_[i].critical_high || sensor_status_[i].critical_low)) {
      char msg[256];
      snprintf(msg, sizeof(msg), "WARN temp: sensor=%s idx=%u temp=%.1fC (warn_high>=%.1fC warn_low<=%.1fC)",
               config.sensor_name.c_str(), i, temp, config.warning_high_temp, config.warning_low_temp);
      SerialManager::getInstance().sendDiagnosticMessage("WARNING", name(), msg);
    }
  }

  // Detect thermal runaway (E-stop level)
  detectThermalRunaway();
  int8_t runaway_idx = -1;
  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (sensor_configured_[i] && sensor_status_[i].thermal_runaway) {
      thermal_runaway_detected = true;
      runaway_idx = static_cast<int8_t>(i);
      break;
    }
  }

  system_status_.system_thermal_warning = any_warning;
  system_status_.system_thermal_critical = any_critical || thermal_runaway_detected;

  FaultSeverity desired_severity = FaultSeverity::NORMAL;
  String desired_description;

  if (thermal_runaway_detected || any_critical) {
    desired_severity = FaultSeverity::EMERGENCY_STOP;
    const int8_t idx = (runaway_idx >= 0) ? runaway_idx : first_critical_idx;
    if (idx >= 0) {
      const TemperatureSensorConfig& config = sensor_configs_[idx];
      const TemperatureSensorStatus& status = sensor_status_[idx];
      if (runaway_idx >= 0) {
        desired_description = "Thermal runaway: sensor=" + config.sensor_name + " idx=" + String(idx) +
                              " temp=" + String(status.temperature_c, 1) + "C trend=" +
                              String(status.temperature_trend, 1) + "C/min thr=" +
                              String(config.thermal_runaway_rate, 1) + "C/min";
      } else {
        desired_description = "Temp critical: sensor=" + config.sensor_name + " idx=" + String(idx) +
                              " temp=" + String(status.temperature_c, 1) + "C (crit_high>=" +
                              String(config.critical_high_temp, 1) + "C crit_low<=" +
                              String(config.critical_low_temp, 1) + "C)";
      }
    } else {
      desired_description = "Temperature critical";
    }
    if (can_log) {
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), "System thermal CRITICAL (E-stop) active");
    }
  } else if (any_warning) {
    desired_severity = FaultSeverity::WARNING;
    const int8_t idx = first_warning_idx;
    if (idx >= 0) {
      const TemperatureSensorConfig& config = sensor_configs_[idx];
      const TemperatureSensorStatus& status = sensor_status_[idx];
      desired_description = "Temp warning: sensor=" + config.sensor_name + " idx=" + String(idx) +
                            " temp=" + String(status.temperature_c, 1) + "C (warn_high>=" +
                            String(config.warning_high_temp, 1) + "C warn_low<=" +
                            String(config.warning_low_temp, 1) + "C)";
    } else {
      desired_description = "Temperature warning";
    }
    if (can_log) {
      SerialManager::getInstance().sendDiagnosticMessage("WARNING", name(), "System thermal WARNING active");
    }
  } else if (any_degraded) {
    desired_severity = FaultSeverity::DEGRADED;
    const int8_t idx = first_degraded_idx;
    if (idx >= 0) {
      const TemperatureSensorConfig& config = sensor_configs_[idx];
      const uint32_t timeout_ms = config.fault_timeout_ms;
      const uint32_t last_valid = sensor_status_[idx].last_valid_reading_time_ms;
      const bool never_valid = (last_valid == 0);
      const uint32_t age_ms = never_valid ? (now - system_start_time_ms_) : (now - last_valid);
      desired_description = "Temp sensor stale: sensor=" + config.sensor_name + " idx=" + String(idx) +
                            " age_ms=" + String(age_ms) + " timeout_ms=" + String(timeout_ms);
    } else {
      desired_description = "Temperature sensor stale";
    }
    if (can_log) {
      SerialManager::getInstance().sendDiagnosticMessage("WARNING", name(), "System thermal DEGRADED (sensor timeout) active");
    }
  }

  // Avoid spamming SafetyCoordinator: only update when state/description changes.
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  const Fault& current = safety.getFault(FaultSource::TEMPERATURE_FAULT);
  if (desired_severity == FaultSeverity::NORMAL) {
    if (current.active) {
      safety.deactivateFault(FaultSource::TEMPERATURE_FAULT);
    }
  } else {
    if (!current.active || current.severity != desired_severity || current.description != desired_description) {
      safety.activateFault(desired_severity, FaultSource::TEMPERATURE_FAULT, desired_description);
    }
  }

  // Update timing for warning/critical states
  if (system_status_.system_thermal_warning && warning_start_time_ms_ == 0) {
    warning_start_time_ms_ = now;
  } else if (!system_status_.system_thermal_warning) {
    warning_start_time_ms_ = 0;
  }

  if (system_status_.system_thermal_critical && critical_start_time_ms_ == 0) {
    critical_start_time_ms_ = now;
  } else if (!system_status_.system_thermal_critical) {
    critical_start_time_ms_ = 0;
  }

  // Update time in states
  if (warning_start_time_ms_ > 0) {
    system_status_.time_in_warning_ms = now - warning_start_time_ms_;
  }
  if (critical_start_time_ms_ > 0) {
    system_status_.time_in_critical_ms = now - critical_start_time_ms_;
  }
}

void TemperatureMonitor::detectThermalRunaway() {
  static uint32_t last_thermal_message_time = 0;
  uint32_t now = millis();

  for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }

    float trend = calculateTemperatureTrend(i);
    sensor_status_[i].temperature_trend = trend;

    // Check for thermal runaway (rapid temperature rise)
    if (trend > sensor_configs_[i].thermal_runaway_rate) {
      if (!sensor_status_[i].thermal_runaway) {
        sensor_status_[i].thermal_runaway = true;

        // Rate limit thermal runaway messages (max once per 1s)
        if (now - last_thermal_message_time >= 1000) {
          String msg = "active:true,source:THERMAL_RUNAWAY,reason:" + sensor_configs_[i].sensor_name +
                       " thermal runaway detected,value:" + String(sensor_status_[i].temperature_c, 1) +
                       ",rate:" + String(trend, 1) + "C_per_min" + ",manual_reset:false,time:" + String(millis());
          SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg.c_str());
          last_thermal_message_time = now;
        }
      }
    } else if (sensor_status_[i].thermal_runaway) {
      // Self-healing: Clear thermal runaway when trend returns to safe levels
      sensor_status_[i].thermal_runaway = false;
      
      if (now - last_thermal_message_time >= 1000) {
        String msg = "active:false,source:THERMAL_RUNAWAY,reason:" + sensor_configs_[i].sensor_name +
                     " thermal runaway cleared,value:" + String(sensor_status_[i].temperature_c, 1) +
                     ",rate:" + String(trend, 1) + "C_per_min" + ",time:" + String(millis());
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg.c_str());
        last_thermal_message_time = now;
      }
    }
  }
}

void TemperatureMonitor::updateTemperatureHistory(uint8_t sensor_index, float temperature) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return;
  }

  TemperatureSensorStatus& status = sensor_status_[sensor_index];
  status.temperature_history[status.history_index] = temperature;
  status.history_index = (status.history_index + 1) % 50;
}

float TemperatureMonitor::calculateTemperatureTrend(uint8_t sensor_index) {
  if (sensor_index >= config_.max_sensors || sensor_index >= kMaxTemperatureSensors) {
    return 0.0f;
  }

  const TemperatureSensorStatus& status = sensor_status_[sensor_index];

  // Determine if buffer is full or filling
  bool buffer_full = !isnan(status.temperature_history[status.history_index]);
  uint32_t count = buffer_full ? 50 : status.history_index;

  // Need at least 10 data points (1 second at 10Hz) to calculate trend reliably
  if (count < 10) {
    return 0.0f;
  }

  float oldest_temp;
  float newest_temp;

  if (buffer_full) {
    // Buffer is full, oldest is at history_index
    oldest_temp = status.temperature_history[status.history_index];
    // Newest is at (history_index - 1) wrapped
    uint8_t newest_idx = (status.history_index + 49) % 50;
    newest_temp = status.temperature_history[newest_idx];
  } else {
    // Buffer filling, oldest is at 0
    oldest_temp = status.temperature_history[0];
    // Newest is at history_index - 1
    newest_temp = status.temperature_history[status.history_index - 1];
  }

  float time_span_minutes = (count * sensor_configs_[sensor_index].read_interval_ms) / 60000.0f;

  if (time_span_minutes > 0) {
    return (newest_temp - oldest_temp) / time_span_minutes;  // °C per minute
  }

  return 0.0f;
}

void TemperatureMonitor::updatePerformanceStatistics() {
  uint32_t now = millis();
  if (now > system_start_time_ms_) {
    float time_diff_s = (now - system_start_time_ms_) / 1000.0f;
    system_status_.system_reading_rate_hz = total_system_readings_ / time_diff_s;

    // Update individual sensor frequencies
    for (uint8_t i = 0; i < config_.max_sensors && i < kMaxTemperatureSensors; i++) {
      if (sensor_status_[i].total_readings > 0 && time_diff_s > 0.0f) {
        sensor_status_[i].reading_frequency_hz = sensor_status_[i].total_readings / time_diff_s;
      }
    }
  }
}

void TemperatureMonitor::sendStatusReports() {
  // Send aggregate TEMPERATURE message in JSON format (like VL53L0X)
  String json = "{";
  json += "\"total_sensors\":" + String(kMaxTemperatureSensors);
  json += ",\"active_sensors\":" + String(system_status_.active_sensors);

  // Add temperatures array
  json += ",\"temperatures\":[";
  for (int i = 0; i < kMaxTemperatureSensors; i++) {
    if (i > 0) json += ",";
    if (sensor_configured_[i] && sensor_status_[i].reading_valid) {
      json += String(sensor_status_[i].temperature_c, 1);
    } else {
      json += "null";
    }
  }
  json += "]";

  // Add system statistics
  json += ",\"avg_temp\":" + String(system_status_.average_temperature, 1);
  json += ",\"max_temp\":" + String(system_status_.highest_temperature, 1);
  json += ",\"min_temp\":" + String(system_status_.lowest_temperature, 1);
  json += ",\"hottest_sensor\":" + String(system_status_.hottest_sensor);
  json += ",\"system_warning\":" + String(system_status_.system_thermal_warning ? "true" : "false");
  json += ",\"system_critical\":" + String(system_status_.system_thermal_critical ? "true" : "false");
  json += ",\"rate_hz\":" + String(system_status_.system_reading_rate_hz, 1);
  json += ",\"readings\":" + String(system_status_.total_readings);
  json += ",\"errors\":" + String(system_status_.total_errors);
  json += "}";

  SerialManager::getInstance().sendMessage("TEMPERATURE", json.c_str());
}

void TemperatureMonitor::sendDiagnosticReports() {
  // Send diagnostic information
  String diag_msg = "active:" + String(system_status_.active_sensors);
  diag_msg += ",readings:" + String(system_status_.total_readings);
  diag_msg += ",errors:" + String(system_status_.total_errors);
  diag_msg += ",rate_hz:" + String(system_status_.system_reading_rate_hz, 1);
  diag_msg += ",warning_time:" + String(system_status_.time_in_warning_ms);
  diag_msg += ",critical_time:" + String(system_status_.time_in_critical_ms);

  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), ("Diagnostic report,details:" + diag_msg).c_str());
}

}  // namespace sigyn_teensy
