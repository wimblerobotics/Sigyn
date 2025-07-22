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
  SerialManager::getInstance().sendMessage("INFO", "VL53L0XMonitor: Starting initialization");
  
  system_start_time_ms_ = millis();
  
  // Initialize I2C communication
  pinMode(I2C_MULTIPLEXER_ENABLE_PIN, OUTPUT);
  digitalWrite(I2C_MULTIPLEXER_ENABLE_PIN, HIGH);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQUENCY);
  
  // Test I2C multiplexer
  multiplexer_available_ = testMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::getInstance().sendMessage("ERROR", 
      "VL53L0XMonitor: I2C multiplexer not found");
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
  
  system_initialized_ = true;  SerialManager::getInstance().sendMessage("INFO", 
    ("VL53L0XMonitor: Initialization complete, " +
    String(array_status_.initialized_sensors) + "/" +
    String(array_status_.total_sensors) + " sensors ready").c_str());
}

void VL53L0XMonitor::loop() {
  if (!system_initialized_) {
    return;
  }
  
  uint32_t now = millis();
  
  // Update sensor measurement cycle
  updateSensorCycle();
  
  // Update array status
  updateArrayStatus();
  
  // Periodic safety checks
  if (now - last_safety_check_time_ms_ >= 50) { // 20Hz safety checks
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
  
  // Attempt to recover failed sensors
  if (now - last_initialization_attempt_ms_ >= config_.initialization_retry_ms) {
    for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
      if (sensor_enabled_[i] && !sensor_status_[i].initialized) {
        initializeSingleSensor(i);
      }
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
  
  SerialManager::getInstance().sendMessage("INFO", "VL53L0XMonitor: Safety flags reset");
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
  
  sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
  sensor_status_[sensor_index].initialized = false;
  
  SerialManager::getInstance().sendMessage("INFO", 
    ("VL53L0XMonitor: Reinitializing sensor " + String(sensor_index)).c_str());
}

void VL53L0XMonitor::reinitializeAll() {
  SerialManager::getInstance().sendMessage("INFO", "VL53L0XMonitor: Reinitializing all sensors");
  
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
  
  // Process current sensor if enabled and initialized
  if (current_sensor_index_ < config_.enabled_sensors &&
      sensor_enabled_[current_sensor_index_] &&
      sensor_status_[current_sensor_index_].initialized) {
    
    if (measureSingleSensor(current_sensor_index_)) {
      updateSensorPerformance(current_sensor_index_);
      total_system_measurements_++;
    } else {
      total_system_errors_++;
    }
  }
  
  // Move to next sensor
  current_sensor_index_++;
  if (current_sensor_index_ >= config_.enabled_sensors) {
    current_sensor_index_ = 0;
  }
}

void VL53L0XMonitor::initializeSensors() {
  uint8_t initialized_count = 0;
  
  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (sensor_enabled_[i] && initializeSingleSensor(i)) {
      initialized_count++;
    }
    
    // Small delay between initializations
    delay(1);
  }
  
  array_status_.initialized_sensors = initialized_count;
  array_status_.system_ready = (initialized_count > 0);
  
  String msg = "VL53L0XMonitor: " + String(initialized_count) + "/" + 
               String(config_.enabled_sensors) + " sensors initialized";
  SerialManager::getInstance().sendMessage("INFO", msg.c_str());
}

bool VL53L0XMonitor::initializeSingleSensor(uint8_t sensor_index) {
  if (sensor_index >= config_.enabled_sensors || !multiplexer_available_) {
    return false;
  }
  
  // Select sensor channel
  if (!selectSensorChannel(sensor_index)) {
    return false;
  }
  
  // Initialize VL53L0X sensor
  if (!sensors_[sensor_index].init()) {
    sensor_status_[sensor_index].initialized = false;
    sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
    handleSensorError(sensor_index);
    return false;
  }
  
  // Configure sensor settings
  sensors_[sensor_index].setTimeout(config_.measurement_timeout_ms);
  
  // Set timing budget for faster measurements
  if (!sensors_[sensor_index].setMeasurementTimingBudget(config_.timing_budget_us)) {
    String msg = "VL53L0XMonitor: Failed to set timing budget for sensor " + String(sensor_index);
    SerialManager::getInstance().sendMessage("WARN", msg.c_str());
  }
  
  // Set signal rate limit
  if (!sensors_[sensor_index].setSignalRateLimit(config_.signal_rate_limit / 128.0f)) {
    String msg = "VL53L0XMonitor: Failed to set signal rate limit for sensor " + String(sensor_index);
    SerialManager::getInstance().sendMessage("WARN", msg.c_str());
  }
  
  // Mark sensor as initialized
  sensor_status_[sensor_index].initialized = true;
  sensor_status_[sensor_index].communication_ok = true;
  sensor_states_[sensor_index] = SensorState::READY;
  
  return true;
}

bool VL53L0XMonitor::measureSingleSensor(uint8_t sensor_index) {
  if (sensor_index >= config_.enabled_sensors || 
      !sensor_status_[sensor_index].initialized) {
    return false;
  }
  
  // Select sensor channel
  if (!selectSensorChannel(sensor_index)) {
    return false;
  }
  
  // Perform measurement
  uint16_t distance = sensors_[sensor_index].readRangeSingleMillimeters();
  
  // Check for timeout
  if (sensors_[sensor_index].timeoutOccurred()) {
    sensor_status_[sensor_index].measurement_valid = false;
    sensor_status_[sensor_index].communication_ok = false;
    sensor_status_[sensor_index].error_count++;
    handleSensorError(sensor_index);
    return false;
  }
  
  // Update sensor status
  sensor_status_[sensor_index].distance_mm = distance;
  sensor_status_[sensor_index].measurement_valid = true;
  sensor_status_[sensor_index].communication_ok = true;
  sensor_status_[sensor_index].last_measurement_time_ms = millis();
  sensor_status_[sensor_index].total_measurements++;
  
  // Check for obstacles and warnings
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
  
  // Check for range errors
  if (distance >= config_.max_range_mm) {
    sensor_status_[sensor_index].range_error = true;
  } else {
    sensor_status_[sensor_index].range_error = false;
  }
  
  return true;
}

void VL53L0XMonitor::handleSensorError(uint8_t sensor_index) {
  sensor_status_[sensor_index].error_count++;
  sensor_states_[sensor_index] = SensorState::ERROR_RECOVERY;
  
  String msg = "VL53L0XMonitor: Error on sensor " + String(sensor_index) + 
               ", error count: " + String(sensor_status_[sensor_index].error_count);
  SerialManager::getInstance().sendMessage("WARN", msg.c_str());
  
  // If too many errors, mark sensor as failed
  if (sensor_status_[sensor_index].error_count > 10) {
    sensor_status_[sensor_index].initialized = false;
    sensor_states_[sensor_index] = SensorState::UNINITIALIZED;
    
    String msg = "VL53L0XMonitor: Sensor " + String(sensor_index) + " marked as failed";
    SerialManager::getInstance().sendMessage("ERROR", msg.c_str());
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
    array_status_.system_measurement_rate_hz = total_system_measurements_ / time_diff_s;
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
  
  for (uint8_t i = 0; i < config_.enabled_sensors; i++) {
    if (!sensor_enabled_[i] || !sensor_status_[i].measurement_valid) {
      continue;
    }
    
    if (sensor_status_[i].distance_mm < 100) { // Critical distance
      critical_obstacle = true;
      
      String msg = "active:true,source:VL53L0X_OBSTACLE,reason:Critical obstacle detected," +
                   String("value:") + String(sensor_status_[i].distance_mm) + 
                   ",sensor:" + String(i) + ",manual_reset:false,time:" + String(millis());
      SerialManager::getInstance().sendMessage("ESTOP", msg.c_str());
    }
  }
  
  // Use the critical_obstacle flag to update system status
  (void)critical_obstacle; // Suppress unused variable warning until fully implemented
}

bool VL53L0XMonitor::selectSensorChannel(uint8_t sensor_index) {
  if (!multiplexer_available_ || sensor_index >= 8) {
    return false;
  }
  
  Wire.beginTransmission(config_.i2c_multiplexer_address);
  Wire.write(1 << sensor_index);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    String msg = "VL53L0XMonitor: Failed to select sensor channel " + String(sensor_index);
    SerialManager::getInstance().sendMessage("ERROR", msg.c_str());
    return false;
  }
  
  // Small delay for multiplexer settling
  delayMicroseconds(100);
  return true;
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
                 String(config_.i2c_multiplexer_address, HEX) + ", error: " + String(error);
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
}

void VL53L0XMonitor::sendStatusReports() {
  // Send VL53L0X status message (JSON format)
  String status_msg = "{";
  status_msg += "\"total_sensors\":" + String(array_status_.total_sensors);
  status_msg += ",\"active_sensors\":" + String(array_status_.active_sensors);
  status_msg += ",\"min_distance\":" + String(array_status_.min_distance_mm);
  status_msg += ",\"max_distance\":" + String(array_status_.max_distance_mm);
  status_msg += ",\"obstacles\":" + String(array_status_.any_obstacles ? "true" : "false");
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
  diag_msg += ",multiplexer_ok:" + String(multiplexer_available_ ? "true" : "false");
  
  String full_msg = "level:INFO,module:VL53L0XMonitor,msg:Diagnostic report,details:" + diag_msg;
  SerialManager::getInstance().sendMessage("DIAG", full_msg.c_str());
}

} // namespace sigyn_teensy
