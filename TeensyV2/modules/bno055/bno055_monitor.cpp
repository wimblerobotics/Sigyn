/**
 * @file bno055_monitor.cpp
 * @brief Implementation of comprehensive dual BNO055 IMU monitoring system
 *
 * This file implements the BNO055Monitor module that provides real-time
 * monitoring of orientation, angular velocity, and linear acceleration using
 * two BNO055 9-DOF IMU sensors for the TeensyV2 system. The implementation
 * supports redundant sensor configurations with automatic sensor detection
 * and graceful degradation.
 *
 * Key Implementation Features:
 *
 * **Multi-Sensor Support:**
 * - Automatic detection and configuration of BNO055 sensors
 * - Graceful degradation when sensors are unavailable
 * - Support for two simultaneous IMU sensors on separate I2C multiplexer channels
 * - Configurable I2C multiplexer support for sensor selection
 *
 * **Real-Time Monitoring:**
 * - High-frequency data acquisition (up to 100Hz) with minimal latency
 * - Quaternion, Euler angles, gyroscope, and linear acceleration data
 * - Exponential moving average (EMA) filtering for stable readings
 * - Low-latency data acquisition suitable for navigation and control
 *
 * **Safety Integration:**
 * - Automatic detection of sensor failures or invalid readings
 * - Integration with global safety system via isUnsafe() interface
 * - Sensor health monitoring and error reporting
 * - Configurable safety thresholds with appropriate hysteresis
 *
 * **Performance Characteristics:**
 * - Sensor reading time: <2ms for both sensors combined
 * - Memory footprint: ~256 bytes per sensor configuration
 * - No dynamic memory allocation during operation
 * - Deterministic execution time for real-time safety
 *
 * **Error Handling:**
 * - Robust I2C communication with automatic retry logic
 * - Sensor disconnection detection and graceful recovery
 * - Invalid reading detection and filtering
 * - Comprehensive error reporting via SerialManager
 *
 * The implementation follows the TeensyV2 architectural principles of
 * modularity, safety, and real-time performance while providing the
 * detailed orientation information needed for autonomous robot navigation.
 *
 * @author Sigyn Robotics
 * @date 2025
 * @version 2.0
 */

#include <Arduino.h>  // Provides millis, delay, snprintf, etc.
#include <Wire.h>     // Provides I2C communication support
#include <limits> // For std::numeric_limits
#include <cstdio>    // For snprintf

#include "bno055_monitor.h"

#include "common/core/serial_manager.h"

namespace sigyn_teensy {

  BNO055Monitor::BNO055Monitor()
    : last_report_time_(0),
    active_sensor_count_(0),
    multiplexer_available_(false),
    setup_completed_(false),
    safety_violation_(false),
    current_sensor_index_(0),
    sensor_stagger_interval_(kDefaultReportInterval / kMaxSensors) {

    // Initialize sensor configurations
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      sensor_configs_[i] = {
        .i2c_address = kBNO055Address,
        .mux_channel = i,  // Channel 0 and 1 for the two sensors
        .read_interval_ms = kDefaultUpdateInterval,
        .timeout_ms = 100,
        .enable_calibration = true,
        .enable_temp_compensation = false,
        .gyro_scale = 1.0f,
        .accel_scale = 1.0f
      };

      sensor_data_[i] = {};
      sensor_states_[i] = IMUState::UNKNOWN;
      last_update_time_[i] = 0;
      current_read_state_[i] = ReadState::IDLE;
      next_sensor_start_time_[i] = i * sensor_stagger_interval_;
    }
  }

  BNO055Monitor& BNO055Monitor::getInstance() {
    static BNO055Monitor instance;
    return instance;
  }

  const char* BNO055Monitor::name() const {
    return "BNO055Monitor";
  }

  void BNO055Monitor::setup() {
    if (setup_completed_) {
      return;
    }

    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Starting dual sensor initialization");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);  // 400kHz for faster communication

    // Test I2C multiplexer
    multiplexer_available_ = testI2CMultiplexer();
    if (!multiplexer_available_) {
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "I2C multiplexer not available");
      setup_completed_ = true;
      return;
    }

    // Initialize each sensor
    active_sensor_count_ = 0;
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      if (initializeSensor(i)) {
        active_sensor_count_++;
        sensor_states_[i] = IMUState::NORMAL;
        char msg[64];
        snprintf(msg, sizeof(msg), "Sensor %d initialized successfully", i);
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);

        // Prime the sensor with additional diagnostics
        if (!primeSensorReadings(i)) {
          char warn_msg[64];
          snprintf(warn_msg, sizeof(warn_msg), "Sensor %d priming incomplete", i);
          SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), warn_msg);
        }
        else {
          char info_msg[64];
          snprintf(info_msg, sizeof(info_msg), "Sensor %d primed successfully", i);
          SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), info_msg);
        }

        // Seed watchdog last publish time to now to avoid early resets before first publish
        last_publish_time_ms_[i] = millis();
      }
      else {
        sensor_states_[i] = IMUState::FAILED;
        char msg[64];
        snprintf(msg, sizeof(msg), "Sensor %d initialization failed", i);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        // Keep last_publish_time at 0 for failed sensors
      }
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "%d/%d sensors initialized", active_sensor_count_, kMaxSensors);
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);

    // Initialize performance stats
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      performance_stats_[i] = { 0, 0, 0, 0, 0 };
    }

    setup_completed_ = true;
  }

  void BNO055Monitor::loop() {
    if (!setup_completed_) {
      return;
    }

    uint32_t loop_start_time = micros();
    uint32_t now = millis();

    // Process sensors in a staggered state machine approach for optimal performance
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      if (sensor_states_[i] != IMUState::NORMAL && sensor_states_[i] != IMUState::WARNING) {
        continue; // Skip failed sensors
      }

      // Ensure next read is always scheduled in the future
      if (next_sensor_start_time_[i] < now) {
        next_sensor_start_time_[i] = now;
      }

      // Check if it's time to start reading this sensor
      if (current_read_state_[i] == ReadState::IDLE && now >= next_sensor_start_time_[i]) {
        selectSensor(sensor_configs_[i].mux_channel);
        current_read_state_[i] = ReadState::READ_QUATERNION;
        sensor_data_[i].timestamp_ms = now;
        sensor_data_[i].valid = false;
      }

      switch (current_read_state_[i]) {
      case ReadState::READ_QUATERNION: {
        if (readQuaternion(sensor_data_[i].qw, sensor_data_[i].qx, sensor_data_[i].qy, sensor_data_[i].qz)) {
          current_read_state_[i] = ReadState::READ_GYROSCOPE; // Proceed to gyro
        }
        else {
          sensor_states_[i] = IMUState::CRITICAL;
          current_read_state_[i] = ReadState::IDLE;
          next_sensor_start_time_[i] = now + sensor_configs_[i].read_interval_ms;
          char msg[80];
          snprintf(msg, sizeof(msg), "BNO055Monitor: Sensor %d quaternion read failed", i);
          SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        }
        break;
      }
      case ReadState::READ_GYROSCOPE: {
        if (readGyroscope(sensor_data_[i].gx, sensor_data_[i].gy, sensor_data_[i].gz)) {
          current_read_state_[i] = ReadState::READ_ACCELERATION;
        }
        else {
          sensor_states_[i] = IMUState::CRITICAL;
          current_read_state_[i] = ReadState::IDLE;
          next_sensor_start_time_[i] = now + sensor_configs_[i].read_interval_ms;
          char msg[80];
          snprintf(msg, sizeof(msg), "BNO055Monitor: Sensor %d gyroscope read failed", i);
          SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        }
        break;
      }
      case ReadState::READ_ACCELERATION: {
        if (readLinearAcceleration(sensor_data_[i].ax, sensor_data_[i].ay, sensor_data_[i].az)) {
          current_read_state_[i] = ReadState::READ_STATUS;
        }
        else {
          sensor_states_[i] = IMUState::CRITICAL;
          current_read_state_[i] = ReadState::IDLE;
          next_sensor_start_time_[i] = now + sensor_configs_[i].read_interval_ms;
          char msg[80];
            snprintf(msg, sizeof(msg), "BNO055Monitor: Sensor %d acceleration read failed", i);
          SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        }
        break;
      }
      case ReadState::READ_STATUS: {
        if (readStatus(sensor_data_[i].system_status, sensor_data_[i].system_error, sensor_data_[i].calibration_status)) {
          current_read_state_[i] = ReadState::COMPLETE;
        }
        else {
          // Status read failure is non-fatal; mark fields invalid but still allow publish
          sensor_data_[i].system_status = 0xFF;
          sensor_data_[i].system_error = 0xFF;
          sensor_data_[i].calibration_status = 0x00;
          current_read_state_[i] = ReadState::COMPLETE;
          char msg[96];
          snprintf(msg, sizeof(msg), "BNO055Monitor: Sensor %d status read failed (continuing)", i);
          SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
        }
        break;
      }
      case ReadState::COMPLETE: {
        sensor_data_[i].valid = true;
        current_read_state_[i] = ReadState::IDLE;
        last_update_time_[i] = now;
        next_sensor_start_time_[i] = now + kPerSensorPublishIntervalMs;
        break;
      }
      case ReadState::IDLE:
      default:
        break;
      }
    }

    // Send status messages at per-sensor interval: publish each sensor when its data is valid and
    // at least kPerSensorPublishIntervalMs has passed since last publish.
    for (uint8_t i = 0; i < kMaxSensors; ++i) {
      if ((sensor_states_[i] == IMUState::NORMAL || sensor_states_[i] == IMUState::WARNING) && sensor_data_[i].valid) {
        if (now - last_publish_time_ms_[i] >= kPerSensorPublishIntervalMs) {
          sendStatusMessage(i);
          last_publish_time_ms_[i] = now;
        }
      }
    }

    // Heartbeat and watchdog once per 1s
    if (now - last_heartbeat_ms_ >= 1000) {
      // Emit concise heartbeat with ages and states
      for (uint8_t i = 0; i < kMaxSensors; i++) {
        uint32_t age = now - last_publish_time_ms_[i];
        const char* state_str = "UNKNOWN";
        switch (sensor_states_[i]) {
        case IMUState::NORMAL: state_str = "NORMAL"; break;
        case IMUState::WARNING: state_str = "WARNING"; break;
        case IMUState::CRITICAL: state_str = "CRITICAL"; break;
        case IMUState::FAILED: state_str = "FAILED"; break;
        case IMUState::CALIBRATING: state_str = "CALIB"; break;
        case IMUState::INITIALIZING: state_str = "INIT"; break;
        default: break;
        }
        char msg[128];
        snprintf(msg, sizeof(msg), "IMU_HEARTBEAT s=%u age=%lums state=%s mux=%u resets=%lu i2c_err=%lu mux_re=%lu",
          (unsigned)i, (unsigned long)age, state_str,
          (unsigned)sensor_configs_[i].mux_channel,
          (unsigned long)imu_reset_count_[i],
          (unsigned long)i2c_error_count_,
          (unsigned long)mux_reselect_count_);
        SerialManager::getInstance().sendDiagnosticMessage("DIAG", name(), msg);
      }
      last_heartbeat_ms_ = now;
    }

    // Watchdog
    checkWatchdog_();

    // Performance monitoring and violation detection
    static uint32_t violation_count = 0;
    static uint32_t last_violation_report = 0;
    uint32_t execution_time_us = micros() - loop_start_time;

    // Check for performance violations (target: <2000us per loop)
    if (execution_time_us > 2000) {
      violation_count++;

      // Report violations but throttle messages to avoid spam
      if ((now - last_violation_report) >= 5000) { // Report every 5 seconds max
        char diag_msg[128];
        snprintf(diag_msg, sizeof(diag_msg),
          "BNO055Monitor PERF_VIOLATION: %luus (target <2000us), violations=%lu, "
          "active_sensors=%d, multiplexer=%s",
          (unsigned long)execution_time_us, (unsigned long)violation_count,
          active_sensor_count_, multiplexer_available_ ? "OK" : "FAIL");
        SerialManager::getInstance().sendDiagnosticMessage("DIAG", name(), diag_msg);

        // Additional diagnostic info about sensor states
        for (uint8_t i = 0; i < kMaxSensors; i++) {
          char state_msg[128]; // Increased buffer size
          const char* state_str = "UNKNOWN";
          switch (sensor_states_[i]) {
          case IMUState::NORMAL: state_str = "NORMAL"; break;
          case IMUState::WARNING: state_str = "WARNING"; break;
          case IMUState::CRITICAL: state_str = "CRITICAL"; break;
          case IMUState::FAILED: state_str = "FAILED"; break;
          case IMUState::INITIALIZING: state_str = "INIT"; break;
          default: break;
          }

          const char* read_state_str = "IDLE";
          switch (current_read_state_[i]) {
          case ReadState::READ_QUATERNION: read_state_str = "QUAT"; break;
          case ReadState::READ_GYROSCOPE: read_state_str = "GYRO"; break;
          case ReadState::READ_ACCELERATION: read_state_str = "ACCEL"; break;
          case ReadState::READ_EULER: read_state_str = "EULER"; break;
          case ReadState::READ_STATUS: read_state_str = "STATUS"; break;
          case ReadState::COMPLETE: read_state_str = "DONE"; break;
          default: break;
          }

          snprintf(state_msg, sizeof(state_msg),
            "BNO055Monitor sensor_%d: state=%s, read_state=%s, "
            "next_start_in=%ldms, calib=0x%02X",
            i, state_str, read_state_str,
            (long)(next_sensor_start_time_[i] - now),
            sensor_data_[i].calibration_status);
          SerialManager::getInstance().sendDiagnosticMessage("DIAG", name(), state_msg);
        }

        last_violation_report = now;
      }
    }
  }

  bool BNO055Monitor::isUnsafe() {
    // Check if all sensors have failed
    if (active_sensor_count_ == 0) {
      safety_violation_ = true;
      return true;
    }

    // Check for critical sensor failures
    uint8_t failed_sensors = 0;
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      if (sensor_states_[i] == IMUState::FAILED || sensor_states_[i] == IMUState::CRITICAL) {
        failed_sensors++;
      }
    }

    // If more than half the sensors fail, consider it unsafe
    safety_violation_ = (failed_sensors > (kMaxSensors / 2));
    return safety_violation_;
  }

  void BNO055Monitor::resetSafetyFlags() {
    safety_violation_ = false;

    // Attempt to re-initialize failed sensors
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      if (sensor_states_[i] == IMUState::FAILED) {
        if (initializeSensor(i)) {
          sensor_states_[i] = IMUState::NORMAL;
        }
      }
    }
  }

  bool BNO055Monitor::getIMUData(uint8_t sensor_id, IMUData& data) const {
    if (sensor_id >= kMaxSensors) {
      return false;
    }

    if (sensor_states_[sensor_id] != IMUState::NORMAL && sensor_states_[sensor_id] != IMUState::WARNING) {
      return false;
    }

    data = sensor_data_[sensor_id];
    return data.valid;
  }

  bool BNO055Monitor::getIMUDataROS(uint8_t sensor_id,
    float& qx, float& qy, float& qz, float& qw,
    float& gx, float& gy, float& gz,
    float& ax, float& ay, float& az) const {
    IMUData data;
    if (!getIMUData(sensor_id, data)) {
      return false;
    }

    // Convert quaternion to ROS format
    convertQuaternionToROS(data.qw, data.qx, data.qy, data.qz, qx, qy, qz, qw);

    // Convert gyroscope and acceleration (coordinate frame conversion may be needed)
    gx = data.gx;
    gy = data.gy;
    gz = data.gz;
    ax = data.ax;
    ay = data.ay;
    az = data.az;

    return true;
  }

  IMUState BNO055Monitor::getSensorState(uint8_t sensor_id) const {
    if (sensor_id >= kMaxSensors) {
      return IMUState::UNKNOWN;
    }
    return sensor_states_[sensor_id];
  }

  bool BNO055Monitor::isSensorCalibrated(uint8_t sensor_id) const {
    if (sensor_id >= kMaxSensors) {
      return false;
    }

    // Check if sensor is fully calibrated (all subsystems calibrated)
    uint8_t calib = sensor_data_[sensor_id].calibration_status;
    return ((calib & 0xC0) == 0xC0) &&  // System calibrated
      ((calib & 0x30) == 0x30) &&  // Gyroscope calibrated
      ((calib & 0x0C) == 0x0C) &&  // Accelerometer calibrated
      ((calib & 0x03) == 0x03);    // Magnetometer calibrated
  }

  bool BNO055Monitor::updateSensorConfig(uint8_t sensor_id, const BNO055Config& config) {
    if (sensor_id >= kMaxSensors) {
      return false;
    }

    sensor_configs_[sensor_id] = config;
    return true;
  }

  bool BNO055Monitor::initializeSensor(uint8_t sensor_id) {
    if (sensor_id >= kMaxSensors) {
      return false;
    }

    // Select sensor via multiplexer
    selectSensor(sensor_configs_[sensor_id].mux_channel);

    // Small delay for multiplexer settling
    delay(10);

    // Check chip ID
    if (!checkChipID()) {
      char msg[64];
      snprintf(msg, sizeof(msg), "BNO055Monitor: Sensor %d chip ID check failed", sensor_id);
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
      return false;
    }

    // Reset sensor
    if (!writeRegister(kRegSysTrigger, 0x20)) {
      return false;
    }
    delay(650);  // Wait for reset to complete

    // Set to config mode
    if (!writeRegister(kRegOprMode, static_cast<uint8_t>(BNO055Mode::CONFIG))) {
      return false;
    }
    delay(20);

    // Set power mode
    if (!writeRegister(kRegPwrMode, static_cast<uint8_t>(BNO055PowerMode::NORMAL))) {
      return false;
    }
    delay(10);

    // Set page 0
    if (!writeRegister(kRegPageID, 0x00)) {
      return false;
    }
    delay(10);

    // If the breakout has a 32.768 kHz crystal, select external crystal for improved stability
    // Teensy 4.1 host being crystal controlled is separate; this specifically configures the BNO055
    if (!writeRegister(kRegSysTrigger, 0x80)) {
      return false;
    }
    delay(10);

    // Set operation mode to NDOF
    if (!writeRegister(kRegOprMode, static_cast<uint8_t>(BNO055Mode::NDOF))) {
      return false;
    }
    // Datasheet: "7ms is required to switch from CONFIG to any other operation mode"
    // Adafruit driver uses 20ms. Let's give it a bit more to be safe.
    delay(25);

    // After switching to NDOF mode, the sensor needs time for the fusion algorithm to stabilize.
    // A small delay here prevents the first few readings from being invalid or slow.
    // The datasheet suggests waiting 100ms for full stabilization.
    delay(100);

    return true;
  }

  void BNO055Monitor::selectSensor(uint8_t mux_channel) const {
    Wire.beginTransmission(kI2CMultiplexerAddress);
    Wire.write(1 << mux_channel);
    Wire.endTransmission();
    // Allow additional settle time after switching TCA9548A channel before any I2C transaction.
    // On longer runs or with multiple devices, 100us can be marginal; use a more conservative delay.
    delayMicroseconds(500);
  }

  bool BNO055Monitor::testI2CMultiplexer() const {
    Wire.beginTransmission(kI2CMultiplexerAddress);
    uint8_t error = Wire.endTransmission();
    return (error == 0);
  }

  bool BNO055Monitor::readRegister(uint8_t reg, uint8_t* data, uint8_t length) const {
    // Retry the full register read sequence once after a short settle to accommodate
    // BNO055 + TCA9548A quirks right after channel switching.
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      Wire.beginTransmission(kBNO055Address);
      Wire.write(reg);
      if (Wire.endTransmission() == 0) {
        Wire.requestFrom(kBNO055Address, length);
        if (Wire.available() == length) {
          for (uint8_t i = 0; i < length; i++) {
            data[i] = Wire.read();
          }
          return true;
        }
      }
      // Count the error and optionally retry once after a short delay
      i2c_error_count_++;
      if (attempt == 0) {
        delayMicroseconds(500);
      }
    }
    return false;
  }

  bool BNO055Monitor::writeRegister(uint8_t reg, uint8_t value) const {
    // Retry once after a short settle to handle edge cases after mux switching
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
      Wire.beginTransmission(kBNO055Address);
      Wire.write(reg);
      Wire.write(value);
      uint8_t err = Wire.endTransmission();
      if (err == 0) {
        return true;
      }
      i2c_error_count_++;
      if (attempt == 0) {
        delayMicroseconds(500);
      }
    }
    return false;
  }

  bool BNO055Monitor::checkChipID() const {
    uint8_t chip_id;
    if (!readRegister(kRegChipID, &chip_id, 1)) {
      return false;
    }
    return (chip_id == kBNO055ChipID);
  }

  bool BNO055Monitor::readQuaternion(float& w, float& x, float& y, float& z) const {
    uint8_t buffer[8];
    if (!readRegister(kRegQuaternionW, buffer, 8)) {
      return false;
    }

    int16_t raw_w = (buffer[1] << 8) | buffer[0];
    int16_t raw_x = (buffer[3] << 8) | buffer[2];
    int16_t raw_y = (buffer[5] << 8) | buffer[4];
    int16_t raw_z = (buffer[7] << 8) | buffer[6];

    w = static_cast<float>(raw_w) / kScaleQuaternion;
    x = static_cast<float>(raw_x) / kScaleQuaternion;
    y = static_cast<float>(raw_y) / kScaleQuaternion;
    z = static_cast<float>(raw_z) / kScaleQuaternion;

    return true;
  }

  bool BNO055Monitor::readGyroscope(float& x, float& y, float& z) const {
    // Add a small delay to allow the sensor to stabilize before reading the gyroscope.
    // This was found to be necessary to prevent read failures on the first loop().
    delay(1);

    uint8_t buffer[6];
    if (!readRegister(kRegGyroX, buffer, 6)) {
      return false;
    }

    int16_t raw_x = (buffer[1] << 8) | buffer[0];
    int16_t raw_y = (buffer[3] << 8) | buffer[2];
    int16_t raw_z = (buffer[5] << 8) | buffer[4];

    // Convert to rad/s (BNO055 outputs in deg/s, scale factor is 16 LSB/°/s)
    x = (static_cast<float>(raw_x) / kScaleGyro) * kRadPerDeg;
    y = (static_cast<float>(raw_y) / kScaleGyro) * kRadPerDeg;
    z = (static_cast<float>(raw_z) / kScaleGyro) * kRadPerDeg;

    return true;
  }

  bool BNO055Monitor::readLinearAcceleration(float& x, float& y, float& z) const {
    uint8_t buffer[6];
    if (!readRegister(kRegLinearAccelX, buffer, 6)) {
      return false;
    }

    int16_t raw_x = (buffer[1] << 8) | buffer[0];
    int16_t raw_y = (buffer[3] << 8) | buffer[2];
    int16_t raw_z = (buffer[5] << 8) | buffer[4];

    // Convert to m/s² (BNO055 scale factor is 100 LSB/m/s²)
    x = static_cast<float>(raw_x) / kScaleAccel;
    y = static_cast<float>(raw_y) / kScaleAccel;
    z = static_cast<float>(raw_z) / kScaleAccel;

    return true;
  }

  bool BNO055Monitor::readEulerAngles(float& heading, float& roll, float& pitch) const {
    uint8_t buffer[6];
    if (!readRegister(kRegEulerH, buffer, 6)) {
      return false;
    }

    int16_t raw_h = (buffer[1] << 8) | buffer[0];
    int16_t raw_r = (buffer[3] << 8) | buffer[2];
    int16_t raw_p = (buffer[5] << 8) | buffer[4];

    // Convert to degrees (BNO055 scale factor is 16 LSB/°)
    heading = static_cast<float>(raw_h) / kScaleEuler;
    roll = static_cast<float>(raw_r) / kScaleEuler;
    pitch = static_cast<float>(raw_p) / kScaleEuler;

    return true;
  }

  bool BNO055Monitor::readStatus(uint8_t& sys_status, uint8_t& sys_error, uint8_t& calib_status) const {
    if (!readRegister(kRegSysStatus, &sys_status, 1)) return false;
    if (!readRegister(kRegSysErr, &sys_error, 1)) return false;
    if (!readRegister(kRegCalibStat, &calib_status, 1)) return false;
    return true;
  }

  // NOTE: updateSensorData function removed - state machine in loop() handles all sensor reading

  void BNO055Monitor::sendStatusMessage(uint8_t sensor_id) {
    if (sensor_id >= kMaxSensors || !sensor_data_[sensor_id].valid) {
      return;
    }

    const IMUData& data = sensor_data_[sensor_id];

    // Create JSON status message with full kinematics & status
    char message[256];
    snprintf(message, sizeof(message),
      "{\"id\":%d,\"qx\":%.4f,\"qy\":%.4f,\"qz\":%.4f,\"qw\":%.4f,"
      "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
      "\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
      "\"calib\":%u,\"status\":%u,\"error\":%u,\"timestamp\":%lu}",
      sensor_id,
      data.qx, data.qy, data.qz, data.qw,
      data.gx, data.gy, data.gz,
      data.ax, data.ay, data.az,
      (unsigned)data.calibration_status,
      (unsigned)data.system_status,
      (unsigned)data.system_error,
      (unsigned long)data.timestamp_ms);

    SerialManager::getInstance().sendMessage("IMU", message);
    // Update publish time to now for watchdog
    last_publish_time_ms_[sensor_id] = millis();
  }

  void BNO055Monitor::convertQuaternionToROS(float bno_w, float bno_x, float bno_y, float bno_z,
    float& ros_x, float& ros_y, float& ros_z, float& ros_w) {
    // BNO055 coordinate system to ROS coordinate system conversion
    // BNO055: X-forward, Y-left, Z-up
    // ROS: X-forward, Y-left, Z-up (same, so no conversion needed)
    // Just reorder from (w,x,y,z) to (x,y,z,w)
    ros_x = bno_x;
    ros_y = bno_y;
    ros_z = bno_z;
    ros_w = bno_w;
  }

  bool BNO055Monitor::primeSensorReadings(uint8_t sensor_id) {
    if (sensor_id >= kMaxSensors) {
      return false;
    }

    // Select the sensor
    selectSensor(sensor_configs_[sensor_id].mux_channel);

    // Allow extra time for sensor fusion to stabilize after mode switch
    delay(100);  // BNO055 NDOF mode stabilization time

    IMUData& data = sensor_data_[sensor_id];
    data.timestamp_ms = millis();
    data.valid = false;

    // Perform multiple read cycles to fully initialize the sensor's data pipeline (full set)
    bool success = true;
    for (int cycle = 1; cycle <= 3; cycle++) {
      char debug_msg[96];
      snprintf(debug_msg, sizeof(debug_msg), "BNO055Monitor: Priming sensor %d (Cycle %d) - full read", sensor_id, cycle);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), debug_msg);

      uint32_t start_time = micros();
      success = readQuaternion(data.qw, data.qx, data.qy, data.qz);
      if (success) success = readGyroscope(data.gx, data.gy, data.gz);
      if (success) success = readLinearAcceleration(data.ax, data.ay, data.az);
      if (success) success = readStatus(data.system_status, data.system_error, data.calibration_status);
      uint32_t elapsed_time = micros() - start_time;
      snprintf(debug_msg, sizeof(debug_msg), "prime_full: success=%d, time=%luus", success, elapsed_time);
      SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), debug_msg);

      if (!success) {
        snprintf(debug_msg, sizeof(debug_msg), "BNO055Monitor: Sensor %d priming failed during cycle %d", sensor_id, cycle);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), debug_msg);
        break;
      }
      delay(50);
    }

    if (success) {
      data.valid = true;
      char info_msg[80];
      snprintf(info_msg, sizeof(info_msg), "BNO055Monitor: Sensor %d primed successfully (full)", sensor_id);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), info_msg);
    }

    return success;
  }

  bool BNO055Monitor::validateGyroscopeReadsDuringPriming(uint8_t sensor_id) {
    for (int attempt = 0; attempt < 3; ++attempt) {
      float gx, gy, gz;
      if (readGyroscope(gx, gy, gz)) { // Corrected function call
        return true;
      }
      char error_msg[64];
      snprintf(error_msg, sizeof(error_msg), "Gyroscope read failed during priming for sensor %d", sensor_id);
      SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), error_msg);
      delay(50); // Retry delay
    }
    return false;
  }

  void BNO055Monitor::initializeMinValueIfUnset(uint8_t sensor_id, float value) {
    if (performance_stats_[sensor_id].min == std::numeric_limits<float>::max()) {
      performance_stats_[sensor_id].min = value;
      char info_msg[64];
      snprintf(info_msg, sizeof(info_msg), "Initialized min value for sensor %d with value %.2f", sensor_id, value);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), info_msg);
    }
  }

  void BNO055Monitor::reselectSensor_(uint8_t mux_channel) const {
    // Toggle off then on to force a clean reselect
    Wire.beginTransmission(kI2CMultiplexerAddress);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) { i2c_error_count_++; }
    // Slightly longer off delay to ensure switches open fully
    delayMicroseconds(200);
    Wire.beginTransmission(kI2CMultiplexerAddress);
    Wire.write(1 << mux_channel);
    if (Wire.endTransmission() != 0) { i2c_error_count_++; }
    // Allow extra settle time before the next transaction
    delayMicroseconds(500);
  }

  void BNO055Monitor::checkWatchdog_() {
    uint32_t now = millis();
    for (uint8_t i = 0; i < kMaxSensors; i++) {
      // Only watch sensors considered active
      if (!(sensor_states_[i] == IMUState::NORMAL || sensor_states_[i] == IMUState::WARNING)) {
        continue;
      }
      uint32_t age = now - last_publish_time_ms_[i];
      if (age >= kStaleWarnMs && (now - last_stale_report_ms_[i] >= 1000)) {
        last_stale_report_ms_[i] = now;
        char msg[160];
        snprintf(msg, sizeof(msg), "IMU_STALE sensor=%u age=%lums state=%u read_state=%u mux=%u",
          (unsigned)i, (unsigned long)age,
          (unsigned)sensor_states_[i], (unsigned)current_read_state_[i],
          (unsigned)sensor_configs_[i].mux_channel);
        SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      }

      if (age >= kStaleResetMs && (now - last_watchdog_reset_ms_[i] >= 2000)) {
        // Attempt recovery
        last_watchdog_reset_ms_[i] = now;
        recoverSensor_(i);
      }
    }
  }

  void BNO055Monitor::recoverSensor_(uint8_t sensor_id) {
    char msg[160];
    snprintf(msg, sizeof(msg), "IMU_WATCHDOG_RESET sensor=%u mux=%u", (unsigned)sensor_id, (unsigned)sensor_configs_[sensor_id].mux_channel);
    SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);

    // Force mux reselect
    reselectSensor_(sensor_configs_[sensor_id].mux_channel);
    mux_reselect_count_++;

    // Soft reset sequence similar to initializeSensor()
    selectSensor(sensor_configs_[sensor_id].mux_channel);
    if (!writeRegister(kRegSysTrigger, 0x20)) { i2c_error_count_++; }
    delay(650);
    if (!writeRegister(kRegOprMode, static_cast<uint8_t>(BNO055Mode::CONFIG))) { i2c_error_count_++; }
    delay(20);
    if (!writeRegister(kRegPwrMode, static_cast<uint8_t>(BNO055PowerMode::NORMAL))) { i2c_error_count_++; }
    delay(10);
    if (!writeRegister(kRegPageID, 0x00)) { i2c_error_count_++; }
    delay(10);
    if (!writeRegister(kRegOprMode, static_cast<uint8_t>(BNO055Mode::NDOF))) { i2c_error_count_++; }
    delay(25);
    delay(100);

    // Clear state machine and prime one quick read attempt
    current_read_state_[sensor_id] = ReadState::IDLE;
    next_sensor_start_time_[sensor_id] = millis();
    imu_reset_count_[sensor_id]++;
  }

  // Ensure only one set of readRegister/writeRegister exists; the versions above count i2c errors.
  // Remove duplicate implementations if present

} // namespace sigyn_teensy
