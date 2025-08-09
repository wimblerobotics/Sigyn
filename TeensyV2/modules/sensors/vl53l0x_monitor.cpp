/**
 * @file vl53l0x_monitor.cpp
 * @brief VL53L0X Time-of-Flight sensor monitoring implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "vl53l0x_monitor.h"

namespace sigyn_teensy {

// Hardware configuration constants
constexpr uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;
constexpr uint8_t I2C_MULTIPLEXER_ENABLE_PIN = 8;
constexpr uint32_t I2C_CLOCK_FREQUENCY = 400000;

VL53L0XMonitor::VL53L0XMonitor()
    : Module(),
      config_(),
      array_status_(),
      current_sensor_index_(0),
      sensor_cycle_start_time_ms_(0),
      multiplexer_available_(false),
      system_initialized_(false),
      last_status_report_time_ms_(0),
      last_diagnostic_report_time_ms_(0),
      last_safety_check_time_ms_(0),
      last_initialization_attempt_ms_(0),
      system_start_time_ms_(0),
      last_measurement_cycle_time_ms_(0),
      total_system_measurements_(0),
      total_system_errors_(0) {
  // Initialize configuration with sensible defaults
  config_.i2c_multiplexer_address = I2C_MULTIPLEXER_ADDRESS;
  config_.max_sensors = 8;
  config_.enabled_sensors = 8;

  // Initialize sensor arrays
  for (uint8_t i = 0; i < 8; i++) {
    sensor_status_[i] = SensorStatus();
    sensor_status_[i].last_successful_read_time_ms = 0; // Initialize timing
    sensor_states_[i] = SensorState::UNINITIALIZED;
    sensor_enabled_[i] = (i < config_.enabled_sensors);
  }

  // Initialize array status
  array_status_.total_sensors = config_.enabled_sensors;
}

VL53L0XMonitor& VL53L0XMonitor::getInstance() {
  static VL53L0XMonitor instance;
  return instance;
}

void VL53L0XMonitor::setup() {
  SerialManager::getInstance().sendMessage(
      "INFO", "VL53L0XMonitor: Starting initialization");

  system_start_time_ms_ = millis();

  // Initialize I2C communication
  pinMode(I2C_MULTIPLEXER_ENABLE_PIN, OUTPUT);
  digitalWrite(I2C_MULTIPLEXER_ENABLE_PIN, HIGH);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQUENCY);

  // Test I2C multiplexer
  multiplexer_available_ = testMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::getInstance().sendMessage(
        "ERROR", "VL53L0XMonitor: I2C multiplexer not found");
    return;
  }

  array_status_.multiplexer_ok = true;

  // Initialize sensors
  initializeSensors();

  // Initialize timing
  uint32_t now = millis();
  last_status_report_time_ms_ = now;
  last_diagnostic_report_time_ms_ = now;
  last_safety_check_time_ms_ = now;
  sensor_cycle_start_time_ms_ = now;

  system_initialized_ = true;
  SerialManager::getInstance().sendMessage(
      "INFO", ("VL53L0XMonitor: Initialization complete, " +
               String(array_status_.initialized_sensors) + "/" +
               String(array_status_.total_sensors) + " sensors ready")
                  .c_str());
}

void VL53L0XMonitor::loop() {
  if (!system_initialized_) {
    return;
  }

  uint32_t now = millis();
  
  // Debug: Add periodic debug output to verify module is running
  static uint32_t last_debug_time = 0;
  if (now - last_debug_time > 10000) { // Every 10 seconds
    SerialManager::getInstance().sendMessage("DEBUG", "VL53L0XMonitor::loop() executing");
    last_debug_time = now;
  }
  
  // Update sensor measurement cycle
  updateSensorCycle();

  // Update array status
  updateArrayStatus();

  // Periodic safety checks
  if (now - last_safety_check_time_ms_ >= 50) {  // 20Hz safety checks
    checkSafetyConditions();
    last_safety_check_time_ms_ = now;
  }

  // Periodic status reporting
  if (now - last_status_report_time_ms_ >= config_.status_report_interval_ms) {
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }

  // Periodic diagnostic reporting
  if (now - last_diagnostic_report_time_ms_ >=
      config_.diagnostic_report_interval_ms) {
    sendDiagnosticReports();
    last_diagnostic_report_time_ms_ = now;
  }

  // Attempt to recover failed sensors
  if (now - last_initialization_attempt_ms_ >=
      config_.initialization_retry_ms) {
    // First try to recover individual failed sensors
    for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
      if (sensor_enabled_[i] && !sensor_status_[i].initialized &&
          sensor_status_[i].error_count < 10) {
        initializeSingleSensor(i);
      }
    }

    // If we have persistently failed sensors, try full recovery
    if (array_status_.initialized_sensors < config_.enabled_sensors) {
      recoverFailedSensors();
    }

    last_initialization_attempt_ms_ = now;
  }
}

bool VL53L0XMonitor::isUnsafe() {
  // Consider system unsafe if critical obstacles are detected
  return array_status_.any_obstacles && (array_status_.min_distance_mm < 100);
}

void VL53L0XMonitor::resetSafetyFlags() {
  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    sensor_status_[i].obstacle_detected = false;
    sensor_status_[i].warning_distance = false;
    sensor_status_[i].range_error = false;
  }
  array_status_.any_obstacles = false;

  SerialManager::getInstance().sendMessage(
      "INFO", "VL53L0XMonitor: Safety flags reset");
}

float VL53L0XMonitor::getDistanceMm(uint8_t sensor_index) const {
  if (sensor_index >= config_.enabled_sensors) {
    return NAN;
  }

  if (!sensor_status_[sensor_index].measurement_valid) {
    return NAN;
  }

  return static_cast<float>(sensor_status_[sensor_index].distance_mm);
}

bool VL53L0XMonitor::isSensorReady(uint8_t sensor_index) const {
  if (sensor_index >= config_.enabled_sensors) {
    return false;
  }

  return sensor_status_[sensor_index].initialized &&
         sensor_status_[sensor_index].communication_ok;
}

bool VL53L0XMonitor::isObstacleDetected(uint8_t sensor_index) const {
  if (sensor_index >= config_.enabled_sensors) {
    return false;
  }

  return sensor_status_[sensor_index].obstacle_detected;
}

const SensorStatus& VL53L0XMonitor::getSensorStatus(
    uint8_t sensor_index) const {
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

  sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
  sensor_status_[sensor_index].initialized = false;

  SerialManager::getInstance().sendMessage(
      "INFO", ("VL53L0XMonitor: Reinitializing sensor " + String(sensor_index))
                  .c_str());
}

void VL53L0XMonitor::reinitializeAll() {
  SerialManager::getInstance().sendMessage(
      "INFO", "VL53L0XMonitor: Reinitializing all sensors");

  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    reinitializeSensor(i);
  }

  initializeSensors();
}

void VL53L0XMonitor::enableSensor(uint8_t sensor_index, bool enable) {
  if (sensor_index >= config_.max_sensors) {
    return;
  }

  sensor_enabled_[sensor_index] = enable;

  if (enable && sensor_index < config_.enabled_sensors) {
    initializeSingleSensor(sensor_index);
  } else if (!enable) {
    sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
    sensor_status_[sensor_index].initialized = false;
  }
}

void VL53L0XMonitor::updateSensorCycle() {
  uint32_t now = millis();

  // Check if it's time to start a new sensor cycle
  if (now - sensor_cycle_start_time_ms_ >= config_.sensor_cycle_time_ms) {
    sensor_cycle_start_time_ms_ = now;
    current_sensor_index_ = 0;
    last_measurement_cycle_time_ms_ = now;
  }

  // Process sensors in round-robin fashion, but respect per-sensor timing
  if (current_sensor_index_ < config_.enabled_sensors) {
    bool sensor_processed = false;
    
    // Find next sensor that's ready to be read (respecting 50ms minimum interval)
    for (uint8_t attempts = 0; attempts < config_.enabled_sensors && !sensor_processed; attempts++) {
      uint8_t sensor_idx = (current_sensor_index_ + attempts) % config_.enabled_sensors;
      
      if (sensor_enabled_[sensor_idx] && 
          sensor_status_[sensor_idx].initialized) {
        
        // Check if enough time has passed since last read (50ms minimum)
        uint32_t time_since_last_read = now - sensor_status_[sensor_idx].last_successful_read_time_ms;
        
        if (time_since_last_read >= 50 || sensor_status_[sensor_idx].last_successful_read_time_ms == 0) {
          // Sensor is ready to be read
          if (measureSingleSensor(sensor_idx)) {
            updateSensorPerformance(sensor_idx);
            total_system_measurements_++;
            sensor_status_[sensor_idx].last_successful_read_time_ms = now;
          } else {
            total_system_errors_++;
          }
          
          // Move to next sensor for next cycle
          current_sensor_index_ = (sensor_idx + 1) % config_.enabled_sensors;
          sensor_processed = true;
        }
        // If sensor not ready (< 50ms), skip to next sensor without reading
      }
    }
    
    // If no sensor was ready, advance to next sensor anyway to prevent getting stuck
    if (!sensor_processed) {
      current_sensor_index_ = (current_sensor_index_ + 1) % config_.enabled_sensors;
    }
  }

  // Reset cycle when we've processed all sensors
  if (current_sensor_index_ >= config_.enabled_sensors) {
    current_sensor_index_ = 0;
  }
}

void VL53L0XMonitor::initializeSensors() {
  uint8_t initialized_count = 0;

  SerialManager::getInstance().sendMessage(
      "INFO", "VL53L0XMonitor: Initializing sensors...");

  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (sensor_enabled_[i]) {
      String msg = "VL53L0XMonitor: Initializing sensor " + String(i) + "...";
      SerialManager::getInstance().sendMessage("INFO", msg.c_str());

      if (initializeSingleSensor(i)) {
        initialized_count++;
      } else {
        String error_msg =
            "VL53L0XMonitor: Failed to initialize sensor " + String(i);
        SerialManager::getInstance().sendMessage("WARN", error_msg.c_str());
      }

      // Delay between sensor initializations to prevent I2C issues
      delay(100);
    }
  }

  array_status_.initialized_sensors = initialized_count;
  array_status_.system_ready = (initialized_count > 0);

  String msg = "VL53L0XMonitor: " + String(initialized_count) + "/" +
               String(config_.enabled_sensors) +
               " sensors initialized successfully";
  SerialManager::getInstance().sendMessage("INFO", msg.c_str());
}

bool VL53L0XMonitor::initializeSingleSensor(uint8_t sensor_index) {
  if (sensor_index >= config_.enabled_sensors || !multiplexer_available_) {
    SerialManager::getInstance().sendMessage(
        "ERROR",
        "VL53L0XMonitor: Invalid sensor index or multiplexer not available");
    return false;
  }

  // Select sensor channel
  selectSensorChannel(sensor_index);
  sensors_[sensor_index].setBus(&Wire);
  // Initialize VL53L0X sensor with proper error checking
  if (!sensors_[sensor_index].init()) {
    sensor_status_[sensor_index].initialized = false;
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
    handleSensorError(sensor_index);

    String msg =
        "VL53L0XMonitor: Failed to initialize sensor " + String(sensor_index);
    SerialManager::getInstance().sendMessage("ERROR", msg.c_str());
    return false;
  }

  // Configure the sensor for good balance of speed and accuracy
  // Use faster timing budget for reduced execution time
  sensors_[sensor_index].setTimeout(50); // Reduced from 70ms
  sensors_[sensor_index].setSignalRateLimit(0.25); // Increased for faster reads
  sensors_[sensor_index].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14); // Reduced for speed
  sensors_[sensor_index].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange,
                                             10); // Reduced for speed

  // Set faster timing budget for quicker measurements (reduced from 33ms)
  if (!sensors_[sensor_index].setMeasurementTimingBudget(20000)) { // 20ms instead of 33ms
    String msg = "VL53L0XMonitor: Failed to set timing budget for sensor " +
                 String(sensor_index);
    SerialManager::getInstance().sendMessage("WARN", msg.c_str());
    // Continue anyway - this is not a fatal error
  }


  // delay(10);                                // Wait for reset

  // Set a unique I2C address for this sensor (0x30 + sensor_index)
  // uint8_t new_address = 0x30 + sensor_index;
  // sensors_[sensor_index].setAddress(new_address);
  // delay(1);  // Short delay for address change to take effect

  // Configure sensor settings with error checking
  // sensors_[sensor_index].setTimeout(config_.measurement_timeout_ms);

  // if (!sensors_[sensor_index].setSignalRateLimit(config_.signal_rate_limit /
  //                                                128.0f)) {
  //   String msg = "VL53L0XMonitor: Failed to set signal rate limit for sensor
  //   " +
  //                String(sensor_index);
  //   SerialManager::getInstance().sendMessage("WARN", msg.c_str());
  //   // Continue anyway - this is not a fatal error
  // }

  // Start continuous back-to-back mode for faster readings
  sensors_[sensor_index].startContinuous();

  // Wait for sensor to stabilize
  delay(50);

  // Test reading to ensure sensor is working
  uint16_t test_distance =
      sensors_[sensor_index].readRangeContinuousMillimeters();
  if (sensors_[sensor_index].timeoutOccurred()) {
    String msg = "VL53L0XMonitor: Sensor " + String(sensor_index) +
                 " test reading failed - timeout";
    SerialManager::getInstance().sendMessage("WARN", msg.c_str());
    sensor_status_[sensor_index].initialized = false;
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
    return false;
  }

  // Mark sensor as initialized
  sensor_status_[sensor_index].initialized = true;
  sensor_status_[sensor_index].communication_ok = true;
  sensor_status_[sensor_index].distance_mm = test_distance;
  sensor_status_[sensor_index].measurement_valid = true;
  sensor_states_[sensor_index] = SensorState::READY;

  String msg =
      "VL53L0XMonitor: Sensor " + String(sensor_index) +
      " initialized successfully, test reading: " + String(test_distance) +
      "mm";
  SerialManager::getInstance().sendMessage("INFO", msg.c_str());

  return true;
}

bool VL53L0XMonitor::measureSingleSensor(uint8_t sensor_index) {
  if (sensor_index >= config_.enabled_sensors ||
      !sensor_status_[sensor_index].initialized) {
    return false;
  }

  // Select sensor channel
  selectSensorChannel(sensor_index);

  // Perform measurement using continuous mode
  uint16_t distance = sensors_[sensor_index].readRangeContinuousMillimeters();

  // Check for timeout
  if (sensors_[sensor_index].timeoutOccurred()) {
    sensor_status_[sensor_index].measurement_valid = false;
    sensor_status_[sensor_index].communication_ok = false;
    sensor_status_[sensor_index].error_count++;

    String msg = "VL53L0XMonitor: Sensor " + String(sensor_index) +
                 " measurement timeout";
    SerialManager::getInstance().sendMessage("WARN", msg.c_str());

    handleSensorError(sensor_index);
    return false;
  }

  // Validate measurement range
  if (distance == 65535 || distance > config_.max_range_mm) {
    // Reading is out of range or invalid
    sensor_status_[sensor_index].distance_mm = distance;
    sensor_status_[sensor_index].measurement_valid = false;
    sensor_status_[sensor_index].range_error = true;
  } else {
    // Valid measurement
    sensor_status_[sensor_index].distance_mm = distance;
    sensor_status_[sensor_index].measurement_valid = true;
    sensor_status_[sensor_index].range_error = false;
  }

  sensor_status_[sensor_index].communication_ok = true;
  sensor_status_[sensor_index].last_measurement_time_ms = millis();
  sensor_status_[sensor_index].total_measurements++;

  // Check for obstacles and warnings only if measurement is valid
  if (sensor_status_[sensor_index].measurement_valid) {
    if (distance < config_.obstacle_threshold_mm) {
      sensor_status_[sensor_index].obstacle_detected = true;
    } else {
      sensor_status_[sensor_index].obstacle_detected = false;
    }

    if (distance < config_.warning_threshold_mm) {
      sensor_status_[sensor_index].warning_distance = true;
    } else {
      sensor_status_[sensor_index].warning_distance = false;
    }
  } else {
    // Clear warning flags for invalid readings
    sensor_status_[sensor_index].obstacle_detected = false;
    sensor_status_[sensor_index].warning_distance = false;
  }

  return true;
}

void VL53L0XMonitor::handleSensorError(uint8_t sensor_index) {
  sensor_status_[sensor_index].error_count++;

  String msg =
      "VL53L0XMonitor: Error on sensor " + String(sensor_index) +
      ", error count: " + String(sensor_status_[sensor_index].error_count);
  SerialManager::getInstance().sendMessage("WARN", msg.c_str());

  // Escalated error handling based on error count
  if (sensor_status_[sensor_index].error_count >= 10) {
    // Too many errors - disable sensor temporarily
    sensor_status_[sensor_index].initialized = false;
    sensor_status_[sensor_index].communication_ok = false;
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;

    String error_msg = "VL53L0XMonitor: Sensor " + String(sensor_index) +
                       " disabled due to excessive errors (" +
                       String(sensor_status_[sensor_index].error_count) + ")";
    SerialManager::getInstance().sendMessage("ERROR", error_msg.c_str());

  } else if (sensor_status_[sensor_index].error_count >= 5) {
    // Multiple errors - try to reinitialize sensor
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;

    String warn_msg = "VL53L0XMonitor: Sensor " + String(sensor_index) +
                      " experiencing errors, attempting recovery";
    SerialManager::getInstance().sendMessage("WARN", warn_msg.c_str());

    // Attempt immediate recovery
    if (initializeSingleSensor(sensor_index)) {
      String recovery_msg = "VL53L0XMonitor: Sensor " + String(sensor_index) +
                            " recovered successfully";
      SerialManager::getInstance().sendMessage("INFO", recovery_msg.c_str());
      sensor_status_[sensor_index].error_count =
          0;  // Reset error count on successful recovery
    }
  } else {
    // Minor error - just mark for monitoring
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
  }
}

void VL53L0XMonitor::updateSensorPerformance(uint8_t sensor_index) {
  uint32_t now = millis();
  uint32_t last_time = sensor_status_[sensor_index].last_measurement_time_ms;

  if (last_time > 0 && now > last_time) {
    float time_diff_s = (now - last_time) / 1000.0f;
    sensor_status_[sensor_index].measurement_frequency_hz = 1.0f / time_diff_s;
  }
}

void VL53L0XMonitor::updateArrayStatus() {
  array_status_.initialized_sensors = 0;
  array_status_.active_sensors = 0;
  array_status_.sensors_with_obstacles = 0;
  array_status_.min_distance_mm = 65535;
  array_status_.max_distance_mm = 0;
  array_status_.min_distance_sensor = 255;
  array_status_.any_obstacles = false;

  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (!sensor_enabled_[i]) continue;

    if (sensor_status_[i].initialized) {
      array_status_.initialized_sensors++;
    }

    if (sensor_status_[i].communication_ok) {
      array_status_.active_sensors++;
    }

    if (sensor_status_[i].obstacle_detected) {
      array_status_.sensors_with_obstacles++;
      array_status_.any_obstacles = true;
    }

    if (sensor_status_[i].measurement_valid) {
      uint16_t distance = sensor_status_[i].distance_mm;

      if (distance < array_status_.min_distance_mm) {
        array_status_.min_distance_mm = distance;
        array_status_.min_distance_sensor = i;
      }

      if (distance > array_status_.max_distance_mm) {
        array_status_.max_distance_mm = distance;
      }
    }
  }

  // Calculate system measurement rate
  uint32_t now = millis();
  if (now > system_start_time_ms_) {
    float time_diff_s = (now - system_start_time_ms_) / 1000.0f;
    array_status_.system_measurement_rate_hz =
        total_system_measurements_ / time_diff_s;
  }

  array_status_.system_ready = (array_status_.initialized_sensors > 0);
  array_status_.total_measurements = total_system_measurements_;
  array_status_.total_errors = total_system_errors_;
}

void VL53L0XMonitor::checkSafetyConditions() {
  detectObstacles();

  // Additional safety checks can be added here
}

void VL53L0XMonitor::detectObstacles() {
  bool critical_obstacle = false;
  static uint32_t last_estop_message_time = 0;
  uint32_t now = millis();

  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (!sensor_enabled_[i] || !sensor_status_[i].measurement_valid) {
      continue;
    }

    if (sensor_status_[i].distance_mm < 100) {  // Critical distance
      critical_obstacle = true;

      // Rate limit ESTOP messages to reduce spam (max once per 200ms)
      if (now - last_estop_message_time >= 200) {
        String msg =
            "active:true,source:VL53L0X_OBSTACLE,reason:Critical obstacle "
            "detected," +
            String("value:") + String(sensor_status_[i].distance_mm) +
            ",sensor:" + String(i) +
            ",manual_reset:false,time:" + String(millis());
        SerialManager::getInstance().sendMessage("ESTOP", msg.c_str());
        last_estop_message_time = now;
      }
    }
  }

  // Use the critical_obstacle flag to update system status
  (void)critical_obstacle;  // Suppress unused variable warning until fully
                            // implemented
}

void VL53L0XMonitor::selectSensorChannel(uint8_t sensor_index) {
  Wire.beginTransmission(
      I2C_MULTIPLEXER_ADDRESS);   // I2C_MULTIPLEXER_ADDRESS, adjust as needed
  Wire.write(1 << sensor_index);  // Select channel
  Wire.endTransmission();
  delayMicroseconds(100);
}

bool VL53L0XMonitor::testMultiplexer() {
  Wire.beginTransmission(config_.i2c_multiplexer_address);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    String msg = "VL53L0XMonitor: I2C multiplexer found at address 0x" +
                 String(config_.i2c_multiplexer_address, HEX);
    SerialManager::getInstance().sendMessage("INFO", msg.c_str());
    return true;
  } else {
    String msg = "VL53L0XMonitor: I2C multiplexer not found at address 0x" +
                 String(config_.i2c_multiplexer_address, HEX) +
                 ", error: " + String(error);
    SerialManager::getInstance().sendMessage("ERROR", msg.c_str());
    return false;
  }
}

void VL53L0XMonitor::resetMultiplexer() {
  digitalWrite(I2C_MULTIPLEXER_ENABLE_PIN, LOW);
  delay(10);
  digitalWrite(I2C_MULTIPLEXER_ENABLE_PIN, HIGH);
  delay(10);

  multiplexer_available_ = testMultiplexer();

  if (multiplexer_available_) {
    SerialManager::getInstance().sendMessage(
        "INFO", "VL53L0XMonitor: Multiplexer reset successful");
  } else {
    SerialManager::getInstance().sendMessage(
        "ERROR", "VL53L0XMonitor: Multiplexer reset failed");
  }
}

void VL53L0XMonitor::recoverFailedSensors() {
  // Attempt to recover sensors that have been disabled due to errors
  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (sensor_enabled_[i] && !sensor_status_[i].initialized &&
        sensor_status_[i].error_count >= 10) {
      String msg =
          "VL53L0XMonitor: Attempting recovery of failed sensor " + String(i);
      SerialManager::getInstance().sendMessage("INFO", msg.c_str());

      // Reset error count to allow retry
      sensor_status_[i].error_count = 0;
      sensor_states_[i] = SensorState::UNINITIALIZED;

      // Try to reinitialize
      if (initializeSingleSensor(i)) {
        String success_msg =
            "VL53L0XMonitor: Sensor " + String(i) + " recovery successful";
        SerialManager::getInstance().sendMessage("INFO", success_msg.c_str());
      } else {
        String fail_msg =
            "VL53L0XMonitor: Sensor " + String(i) + " recovery failed";
        SerialManager::getInstance().sendMessage("WARN", fail_msg.c_str());
      }

      // Delay between recovery attempts
      delay(100);
    }
  }
}

void VL53L0XMonitor::sendStatusReports() {
  // Send VL53L0X status message (JSON format)
  String status_msg = "{";
  status_msg += "\"total_sensors\":" + String(array_status_.total_sensors);
  status_msg += ",\"active_sensors\":" + String(array_status_.active_sensors);
  status_msg += ",\"min_distance\":" + String(array_status_.min_distance_mm);
  status_msg += ",\"max_distance\":" + String(array_status_.max_distance_mm);
  status_msg += ",\"obstacles\":" +
                String(array_status_.any_obstacles ? "true" : "false");
  status_msg += ",\"distances\":[";

  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (i > 0) status_msg += ",";
    if (sensor_status_[i].measurement_valid) {
      status_msg += String(sensor_status_[i].distance_mm);
    } else {
      status_msg += "null";
    }
  }

  status_msg += "]}";

  SerialManager::getInstance().sendMessage("VL53L0X", status_msg.c_str());
}

void VL53L0XMonitor::sendDiagnosticReports() {
  // Send diagnostic information
  String diag_msg = "initialized:" + String(array_status_.initialized_sensors);
  diag_msg += ",active:" + String(array_status_.active_sensors);
  diag_msg += ",measurements:" + String(total_system_measurements_);
  diag_msg += ",errors:" + String(total_system_errors_);
  diag_msg += ",rate_hz:" + String(array_status_.system_measurement_rate_hz, 1);
  diag_msg +=
      ",multiplexer_ok:" + String(multiplexer_available_ ? "true" : "false");
  
  // Add per-sensor read timing info for performance monitoring
  diag_msg += ",sensor_timing:[";
  uint32_t now = millis();
  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (i > 0) diag_msg += ",";
    uint32_t time_since_read = (sensor_status_[i].last_successful_read_time_ms > 0) ? 
                               now - sensor_status_[i].last_successful_read_time_ms : 9999;
    diag_msg += String(time_since_read);
  }
  diag_msg += "]";

  // Convert to JSON format for diagnostic message
  String json_msg = "{\"level\":\"INFO\",\"module\":\"VL53L0XMonitor\",\"message\":\"Diagnostic report\",\"details\":\"" + diag_msg + "\"}";
  SerialManager::getInstance().sendMessage("DIAG", json_msg.c_str());
}

}  // namespace sigyn_teensy
