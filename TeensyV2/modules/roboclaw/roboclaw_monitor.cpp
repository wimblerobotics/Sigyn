/**
 * @file roboclaw_monitor.cpp
 * @brief RoboClaw motor controller monitoring implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "roboclaw_monitor.h"
#include <cstring>

namespace sigyn_teensy {

// Configuration constants (ported from legacy config.h)
constexpr float WHEEL_DIAMETER_M = 0.102224144529039f;
constexpr float WHEEL_BASE_M = 0.3906f;
constexpr uint32_t QUADRATURE_PULSES_PER_REVOLUTION = 1000;
constexpr uint32_t MAX_MOTOR_SPEED_QPPS = 1392;
constexpr uint32_t MAX_ACCELERATION_QPPS2 = 3000;
constexpr float MAX_SECONDS_COMMANDED_TRAVEL = 0.05f;
constexpr uint32_t MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS = 200;

// Hardware configuration
constexpr uint8_t ROBOCLAW_ADDRESS = 0x80;
constexpr uint32_t ROBOCLAW_TIMEOUT_US = 10000;
constexpr uint32_t ROBOCLAW_BAUD_RATE = 38400;
constexpr const char* ROBOCLAW_SOFTWARE_VERSION = "USB Roboclaw 2x15a v4.2.8\n";

// Serial port (define this based on your hardware setup)
#define ROBOCLAW_SERIAL Serial7

RoboClawMonitor::RoboClawMonitor()
    : Module(),
      config_(),
      roboclaw_(&ROBOCLAW_SERIAL, config_.timeout_us),
      connection_state_(ConnectionState::DISCONNECTED),
      reading_state_(ReadingState::READ_ENCODER_M1),
      last_reading_time_ms_(0),
      motor1_status_(),
      motor2_status_(),
      system_status_(),
      last_velocity_command_(),
      last_commanded_m1_qpps_(0),
      last_commanded_m2_qpps_(0),
      last_command_time_ms_(0),
      emergency_stop_active_(false),
      runaway_detection_initialized_(false),
      last_runaway_check_time_ms_(0),
      last_runaway_encoder_m1_(0),
      last_runaway_encoder_m2_(0),
      last_status_report_time_ms_(0),
      last_diagnostic_report_time_ms_(0),
      last_safety_check_time_ms_(0),
      total_commands_sent_(0),
      total_communication_errors_(0),
      total_safety_violations_(0) {
  
  // Initialize configuration with sensible defaults
  config_.address = ROBOCLAW_ADDRESS;
  config_.timeout_us = ROBOCLAW_TIMEOUT_US;
  config_.baud_rate = ROBOCLAW_BAUD_RATE;
  
  // Set hardware-specific defaults
  config_.max_speed_qpps = MAX_MOTOR_SPEED_QPPS;
  config_.default_acceleration = MAX_ACCELERATION_QPPS2;
}

RoboClawMonitor& RoboClawMonitor::getInstance() {
  static RoboClawMonitor instance;
  return instance;
}

void RoboClawMonitor::setup() {
  SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Starting initialization");
  
  // Initialize serial communication
  roboclaw_.begin(config_.baud_rate);
  
  // Initialize hardware pins if needed
  // pinMode(E_STOP_PIN, OUTPUT);
  // digitalWrite(E_STOP_PIN, HIGH); // HIGH = E-Stop not active
  
  connection_state_ = ConnectionState::CONNECTING;
  
  if (initializeRoboClaw()) {
    connection_state_ = ConnectionState::CONNECTED;
    SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Initialization successful");
  } else {
    connection_state_ = ConnectionState::ERROR_RECOVERY;
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Initialization failed");
  }
  
  // Initialize timing
  uint32_t now = millis();
  last_status_report_time_ms_ = now;
  last_diagnostic_report_time_ms_ = now;
  last_safety_check_time_ms_ = now;
  last_runaway_check_time_ms_ = now;
}

void RoboClawMonitor::loop() {
  uint32_t now = millis();
  
  // Handle connection state machine
  switch (connection_state_) {
    case ConnectionState::DISCONNECTED:
    case ConnectionState::ERROR_RECOVERY:
      // Attempt reconnection
      if (initializeRoboClaw()) {
        connection_state_ = ConnectionState::CONNECTED;
        SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Reconnected");
      }
      break;
      
    case ConnectionState::CONNECTING:
      // Wait for initialization to complete
      break;
      
    case ConnectionState::CONNECTED:
      // Normal operation
      updateMotorStatus();  // Now uses state machine, much faster
      
      // Periodic safety checks
      if (now - last_safety_check_time_ms_ >= 50) { // 20Hz safety checks
        checkSafetyConditions();
        last_safety_check_time_ms_ = now;
      }
      
      // Handle command timeouts
      if (now - last_velocity_command_.timestamp_ms > MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS) {
        // Timeout - stop motors
        setMotorSpeeds(0, 0);
      }
      
      break;
  }
  
  // Periodic reporting (regardless of connection state)
  if (now - last_status_report_time_ms_ >= config_.status_report_interval_ms) {
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }
  
  if (now - last_diagnostic_report_time_ms_ >= config_.diagnostic_report_interval_ms) {
    sendDiagnosticReports();
    last_diagnostic_report_time_ms_ = now;
  }
}

bool RoboClawMonitor::isUnsafe() {
  String safety_reasons = "";
  bool unsafe = false;
  
  // Check local safety flags
  if (emergency_stop_active_) {
    safety_reasons += "emergency_stop_active ";
    unsafe = true;
  }
  
  if (motor1_status_.runaway_detected) {
    safety_reasons += "motor1_runaway ";
    unsafe = true;
  }
  
  if (motor2_status_.runaway_detected) {
    safety_reasons += "motor2_runaway ";
    unsafe = true;
  }
  
  if (motor1_status_.overcurrent) {
    safety_reasons += "motor1_overcurrent ";
    unsafe = true;
  }
  
  if (motor2_status_.overcurrent) {
    safety_reasons += "motor2_overcurrent ";
    unsafe = true;
  }
  
  if (!motor1_status_.communication_ok) {
    safety_reasons += "motor1_comm_fail ";
    unsafe = true;
  }
  
  if (!motor2_status_.communication_ok) {
    safety_reasons += "motor2_comm_fail ";
    unsafe = true;
  }
  
  // Check RoboClaw error status for safety-critical errors
  uint32_t error = system_status_.error_status;
  bool roboclaw_unsafe = (error & static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT)) ||
                        (error & static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT)) ||
                        (error & static_cast<uint32_t>(RoboClawError::E_STOP)) ||
                        (error & static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR)) ||
                        (error & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT)) ||
                        (error & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT)) ||
                        (error & static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_LOW));
  
  if (roboclaw_unsafe) {
    String error_details = decodeErrorStatus(error);
    safety_reasons += "roboclaw_hw_error[" + error_details + "] ";
    unsafe = true;
  }
  
  // Log safety status if unsafe condition detected
  if (unsafe && safety_reasons.length() > 0) {
    SerialManager::getInstance().sendMessage("WARN", 
      ("RoboClawMonitor unsafe - reasons: " + safety_reasons).c_str());
  }
  
  return unsafe;
}

void RoboClawMonitor::resetSafetyFlags() {
  emergency_stop_active_ = false;
  motor1_status_.runaway_detected = false;
  motor2_status_.runaway_detected = false;
  motor1_status_.overcurrent = false;
  motor2_status_.overcurrent = false;
  motor1_status_.timeout_error = false;
  motor2_status_.timeout_error = false;
  
  SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Safety flags reset");
}

void RoboClawMonitor::setVelocityCommand(float linear_x, float angular_z) {
  last_velocity_command_.linear_x = linear_x;
  last_velocity_command_.angular_z = angular_z;
  last_velocity_command_.timestamp_ms = millis();
  
  // Convert velocity to motor speeds
  int32_t m1_qpps, m2_qpps;
  velocityToMotorSpeeds(linear_x, angular_z, m1_qpps, m2_qpps);
  
  setMotorSpeeds(m1_qpps, m2_qpps);
}

void RoboClawMonitor::setMotorSpeeds(int32_t m1_qpps, int32_t m2_qpps) {
  if (emergency_stop_active_) {
    m1_qpps = 0;
    m2_qpps = 0;
  }
  
  // Constrain to maximum speeds
  m1_qpps = constrain(m1_qpps, -static_cast<int32_t>(config_.max_speed_qpps), 
                      static_cast<int32_t>(config_.max_speed_qpps));
  m2_qpps = constrain(m2_qpps, -static_cast<int32_t>(config_.max_speed_qpps), 
                      static_cast<int32_t>(config_.max_speed_qpps));
  
  executeMotorCommand(m1_qpps, m2_qpps);
  
  // Update command tracking
  last_commanded_m1_qpps_ = m1_qpps;
  last_commanded_m2_qpps_ = m2_qpps;
  last_command_time_ms_ = millis();
  total_commands_sent_++;
}

void RoboClawMonitor::emergencyStop() {
  emergency_stop_active_ = true;
  setMotorSpeeds(0, 0);
   SerialManager::getInstance().sendMessage("ESTOP", 
    ("active:true,source:ROBOCLAW,reason:Emergency stop activated,manual_reset:false,time:" +
    String(millis())).c_str());
}

void RoboClawMonitor::resetErrors() {
  resetSafetyFlags();
  total_communication_errors_ = 0;
  total_safety_violations_ = 0;
  
  // Attempt to clear RoboClaw errors if connected
  if (connection_state_ == ConnectionState::CONNECTED) {
    // RoboClaw doesn't have a specific error reset command, 
    // but stopping motors can help clear some error conditions
    setMotorSpeeds(0, 0);
  }
}

void RoboClawMonitor::handleTwistMessage(const String& data) {
  // Parse twist message: "linear_x:<value>,angular_z:<value>"
  float linear_x = 0.0f, angular_z = 0.0f;
  
  int linear_start = data.indexOf("linear_x:") + 9;
  int linear_end = data.indexOf(",", linear_start);
  if (linear_start > 8 && linear_end > linear_start) {
    linear_x = data.substring(linear_start, linear_end).toFloat();
  }
  
  int angular_start = data.indexOf("angular_z:") + 10;
  if (angular_start > 9) {
    angular_z = data.substring(angular_start).toFloat();
  }
  
  setVelocityCommand(linear_x, angular_z);
}

bool RoboClawMonitor::initializeRoboClaw() {
  // Test communication
  if (!testCommunication()) {
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Communication test failed");
    return false;
  }
  
  // Set PID values (ported from legacy code)
  if (!roboclaw_.SetM1VelocityPID(config_.address, 7.26239f, 2.43f, 0.0f, 2437)) {
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Failed to set M1 PID");
    return false;
  }
  
  if (!roboclaw_.SetM2VelocityPID(config_.address, 7.26239f, 2.43f, 0.0f, 2437)) {
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Failed to set M2 PID");
    return false;
  }
  
  // Initialize encoder readings for runaway detection
  bool valid1, valid2;
  last_runaway_encoder_m1_ = roboclaw_.ReadEncM1(config_.address, nullptr, &valid1);
  last_runaway_encoder_m2_ = roboclaw_.ReadEncM2(config_.address, nullptr, &valid2);
  
  if (valid1 && valid2) {
    runaway_detection_initialized_ = true;
  } else {
    SerialManager::getInstance().sendMessage("WARN", "RoboClawMonitor: Failed to read initial encoder values");
  }
  
  return true;
}

bool RoboClawMonitor::testCommunication() {
  char version[64];
  bool success = roboclaw_.ReadVersion(config_.address, version);
  
  if (success) {
    // Check if version matches expected
    if (strstr(version, "Roboclaw") != nullptr) {
      String msg = "RoboClawMonitor: Version check passed: " + String(version);
      SerialManager::getInstance().sendMessage("INFO", msg.c_str());
      return true;
    } else {
      String msg = "RoboClawMonitor: Unexpected version: " + String(version);
      SerialManager::getInstance().sendMessage("WARN", msg.c_str());
      return false;
    }
  } else {
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Failed to read version");
    return false;
  }
}

void RoboClawMonitor::handleCommunicationError() {
  total_communication_errors_++;
  connection_state_ = ConnectionState::ERROR_RECOVERY;
  
  // Clear status flags
  motor1_status_.communication_ok = false;
  motor2_status_.communication_ok = false;
  
  SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Communication error detected");
}

void RoboClawMonitor::velocityToMotorSpeeds(float linear_x, float angular_z, 
                                           int32_t& m1_qpps, int32_t& m2_qpps) {
  // Convert twist to differential drive wheel speeds
  float v_left_mps = linear_x - (angular_z * WHEEL_BASE_M / 2.0f);
  float v_right_mps = linear_x + (angular_z * WHEEL_BASE_M / 2.0f);
  
  // Convert m/s to RPM
  float rpm_left = (v_left_mps * 60.0f) / (M_PI * WHEEL_DIAMETER_M);
  float rpm_right = (v_right_mps * 60.0f) / (M_PI * WHEEL_DIAMETER_M);
  
  // Convert RPM to QPPS
  m1_qpps = static_cast<int32_t>((rpm_left / 60.0f) * QUADRATURE_PULSES_PER_REVOLUTION);
  m2_qpps = static_cast<int32_t>((rpm_right / 60.0f) * QUADRATURE_PULSES_PER_REVOLUTION);
}

void RoboClawMonitor::executeMotorCommand(int32_t m1_qpps, int32_t m2_qpps) {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }
  
  // Calculate distances for move command
  uint32_t m1_max_distance = abs(m1_qpps * MAX_SECONDS_COMMANDED_TRAVEL);
  uint32_t m2_max_distance = abs(m2_qpps * MAX_SECONDS_COMMANDED_TRAVEL);
  
  // Send command to RoboClaw
  bool success = roboclaw_.SpeedAccelDistanceM1M2(
    config_.address,
    config_.default_acceleration,
    m1_qpps, m1_max_distance,
    m2_qpps, m2_max_distance,
    1  // Flag
  );
  
  if (!success) {
    handleCommunicationError();
  }
  
  // Reset safety flags if stopping
  if (m1_qpps == 0 && m2_qpps == 0) {
    motor1_status_.runaway_detected = false;
    motor2_status_.runaway_detected = false;
  }
}

void RoboClawMonitor::updateMotorStatus() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }
  
  uint32_t now = millis();
  
  // Limit reading frequency to prevent overloading serial communication
  // Spread reads across multiple cycles: each state should take ~1-2ms
  if (now - last_reading_time_ms_ < 10) { // Minimum 10ms between reads
    return;
  }
  
  bool valid;
  uint8_t status;
  
  // State machine to spread serial reads across multiple loop cycles
  switch (reading_state_) {
    case ReadingState::READ_ENCODER_M1:
      motor1_status_.encoder_count = roboclaw_.ReadEncM1(config_.address, &status, &valid);
      motor1_status_.encoder_valid = valid;
      if (!valid) {
        handleCommunicationError();
        return;
      }
      reading_state_ = ReadingState::READ_SPEED_M1;
      break;
      
    case ReadingState::READ_SPEED_M1:
      motor1_status_.speed_qpps = roboclaw_.ReadSpeedM1(config_.address, &status, &valid);
      motor1_status_.speed_valid = valid;
      if (!valid) {
        handleCommunicationError();
        return;
      }
      reading_state_ = ReadingState::READ_ENCODER_M2;
      break;
      
    case ReadingState::READ_ENCODER_M2:
      motor2_status_.encoder_count = roboclaw_.ReadEncM2(config_.address, &status, &valid);
      motor2_status_.encoder_valid = valid;
      if (!valid) {
        handleCommunicationError();
        return;
      }
      reading_state_ = ReadingState::READ_SPEED_M2;
      break;
      
    case ReadingState::READ_SPEED_M2:
      motor2_status_.speed_qpps = roboclaw_.ReadSpeedM2(config_.address, &status, &valid);
      motor2_status_.speed_valid = valid;
      if (!valid) {
        handleCommunicationError();
        return;
      }
      reading_state_ = ReadingState::READ_CURRENTS;
      break;
      
    case ReadingState::READ_CURRENTS:
      {
        int16_t current1, current2;
        bool current_valid = roboclaw_.ReadCurrents(config_.address, current1, current2);
        
        if (current_valid) {
          motor1_status_.current_amps = current1 / 100.0f;
          motor2_status_.current_amps = current2 / 100.0f;
          motor1_status_.current_valid = true;
          motor2_status_.current_valid = true;
        } else {
          motor1_status_.current_valid = false;
          motor2_status_.current_valid = false;
          handleCommunicationError();
          return;
        }
        reading_state_ = ReadingState::READ_VOLTAGES;
      }
      break;
      
    case ReadingState::READ_VOLTAGES:
      // This could be split further if needed, but voltages are typically fast
      updateSystemStatus();  // This contains voltage and error reads
      reading_state_ = ReadingState::COMPLETE;
      break;
      
    case ReadingState::COMPLETE:
      // All readings complete for this cycle
      motor1_status_.communication_ok = true;
      motor2_status_.communication_ok = true;
      reading_state_ = ReadingState::READ_ENCODER_M1; // Start next cycle
      break;
  }
  
  last_reading_time_ms_ = now;
}

void RoboClawMonitor::updateSystemStatus() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }
  
  bool valid;
  
  // Only read one system parameter per call to keep it fast
  // This will be called from the reading state machine
  static uint8_t system_read_counter = 0;
  
  switch (system_read_counter % 4) {
    case 0:
      // Read main battery voltage
      {
        uint16_t main_voltage = roboclaw_.ReadMainBatteryVoltage(config_.address, &valid);
        if (valid) {
          system_status_.main_battery_voltage = main_voltage / 10.0f;
        }
      }
      break;
      
    case 1:
      // Read logic battery voltage
      {
        uint16_t logic_voltage = roboclaw_.ReadLogicBatteryVoltage(config_.address, &valid);
        if (valid) {
          system_status_.logic_battery_voltage = logic_voltage / 10.0f;
        }
      }
      break;
      
    case 2:
      // Read error status
      system_status_.error_status = roboclaw_.ReadError(config_.address, &valid);
      break;
      
    case 3:
      // Read temperature if available (this might be slower, so do it last)
      {
        uint16_t temp;
        if (roboclaw_.ReadTemp(config_.address, temp)) {
          system_status_.temperature_c = temp / 10.0f;
        }
      }
      break;
  }
  
  system_read_counter++;
  system_status_.last_status_update_ms = millis();
}

void RoboClawMonitor::checkSafetyConditions() {
  // Check for overcurrent conditions
  if (motor1_status_.current_valid && 
      abs(motor1_status_.current_amps) > config_.max_current_m1) {
    motor1_status_.overcurrent = true;
    total_safety_violations_++;
     SerialManager::getInstance().sendMessage("ESTOP",
      ("active:true,source:ROBOCLAW_CURRENT,reason:Motor 1 overcurrent," +
      String("value:") + String(motor1_status_.current_amps) +
      ",manual_reset:false,time:" + String(millis())).c_str());
  }
  
  if (motor2_status_.current_valid && 
      abs(motor2_status_.current_amps) > config_.max_current_m2) {
    motor2_status_.overcurrent = true;
    total_safety_violations_++;
     SerialManager::getInstance().sendMessage("ESTOP",
      ("active:true,source:ROBOCLAW_CURRENT,reason:Motor 2 overcurrent," +
      String("value:") + String(motor2_status_.current_amps) +
      ",manual_reset:false,time:" + String(millis())).c_str());
  }
  
  // Check for runaway detection
  detectMotorRunaway();
}

void RoboClawMonitor::detectMotorRunaway() {
  if (!runaway_detection_initialized_) {
    return;
  }
  
  uint32_t now = millis();
  if (now - last_runaway_check_time_ms_ < config_.runaway_check_interval_ms) {
    return;
  }
  
  // Check if motors are running away (moving when commanded to stop)
  if (last_commanded_m1_qpps_ == 0 && abs(motor1_status_.speed_qpps) > 100) {
    motor1_status_.runaway_detected = true;
    total_safety_violations_++;
     SerialManager::getInstance().sendMessage("ESTOP",
      ("active:true,source:MOTOR_RUNAWAY,reason:Motor 1 runaway detected," +
      String("value:") + String(motor1_status_.speed_qpps) +
      ",manual_reset:false,time:" + String(millis())).c_str());
  }
  
  if (last_commanded_m2_qpps_ == 0 && abs(motor2_status_.speed_qpps) > 100) {
    motor2_status_.runaway_detected = true;
    total_safety_violations_++;
     SerialManager::getInstance().sendMessage("ESTOP",
      ("active:true,source:MOTOR_RUNAWAY,reason:Motor 2 runaway detected," +
      String("value:") + String(motor2_status_.speed_qpps) +
      ",manual_reset:false,time:" + String(millis())).c_str());
  }
  
  last_runaway_check_time_ms_ = now;
}

void RoboClawMonitor::sendStatusReports() {
  // Send ROBOCLAW status message (JSON format)
  String status_msg = "{";
  status_msg += "\"LogicVoltage\":" + String(system_status_.logic_battery_voltage, 1);
  status_msg += ",\"MainVoltage\":" + String(system_status_.main_battery_voltage, 1);
  status_msg += ",\"Encoder_Left\":" + String(motor1_status_.encoder_count);
  status_msg += ",\"Encoder_Right\":" + String(motor2_status_.encoder_count);
  status_msg += ",\"LeftMotorCurrent\":" + String(motor1_status_.current_amps, 3);
  status_msg += ",\"RightMotorCurrent\":" + String(motor2_status_.current_amps, 3);
  status_msg += ",\"LeftMotorSpeed\":" + String(motor1_status_.speed_qpps);
  status_msg += ",\"RightMotorSpeed\":" + String(motor2_status_.speed_qpps);
  status_msg += ",\"Error\":" + String(system_status_.error_status, HEX);
  status_msg += ",\"ErrorDecoded\":\"" + decodeErrorStatus(system_status_.error_status) + "\"";
  status_msg += "}";
  
  SerialManager::getInstance().sendMessage("ROBOCLAW", status_msg.c_str());
}

void RoboClawMonitor::sendDiagnosticReports() {
  // Send diagnostic information
  String diag_msg = "commands:" + String(total_commands_sent_);
  diag_msg += ",errors:" + String(total_communication_errors_);
  diag_msg += ",safety_violations:" + String(total_safety_violations_);
  diag_msg += ",connection_state:" + String(static_cast<int>(connection_state_));
  diag_msg += ",emergency_stop:" + String(emergency_stop_active_ ? "true" : "false");
  
  SerialManager::getInstance().sendMessage("DIAG", 
    ("level:INFO,module:RoboClawMonitor,msg:Diagnostic report,details:" + diag_msg).c_str());
}

String RoboClawMonitor::decodeErrorStatus(uint32_t error_status) const {
  if (error_status == 0) {
    return "No errors";
  }
  
  String errors = "";
  bool first = true;
  
  // Check each error bit and add description
  if (error_status & static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT)) {
    if (!first) errors += ", ";
    errors += "M1_OVERCURRENT";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT)) {
    if (!first) errors += ", ";
    errors += "M2_OVERCURRENT";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::E_STOP)) {
    if (!first) errors += ", ";
    errors += "E_STOP_ACTIVE";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR)) {
    if (!first) errors += ", ";
    errors += "TEMPERATURE_ERROR";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT)) {
    if (!first) errors += ", ";
    errors += "M1_DRIVER_FAULT";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT)) {
    if (!first) errors += ", ";
    errors += "M2_DRIVER_FAULT";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH)) {
    if (!first) errors += ", ";
    errors += "MAIN_BATTERY_HIGH";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::LOGIC_BATTERY_HIGH)) {
    if (!first) errors += ", ";
    errors += "LOGIC_BATTERY_HIGH";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::LOGIC_BATTERY_LOW)) {
    if (!first) errors += ", ";
    errors += "LOGIC_BATTERY_LOW";
    first = false;
  }
  
  if (error_status & static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_LOW)) {
    if (!first) errors += ", ";
    errors += "MAIN_BATTERY_LOW";
    first = false;
  }
  
  // Check for unknown error bits
  uint32_t known_errors = static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT) |
                         static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT) |
                         static_cast<uint32_t>(RoboClawError::E_STOP) |
                         static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR) |
                         static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT) |
                         static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT) |
                         static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH) |
                         static_cast<uint32_t>(RoboClawError::LOGIC_BATTERY_HIGH) |
                         static_cast<uint32_t>(RoboClawError::LOGIC_BATTERY_LOW) |
                         static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_LOW);
  
  uint32_t unknown_errors = error_status & ~known_errors;
  if (unknown_errors != 0) {
    if (!first) errors += ", ";
    errors += "UNKNOWN_ERROR_BITS:0x" + String(unknown_errors, HEX);
  }
  
  return errors;
}

} // namespace sigyn_teensy
