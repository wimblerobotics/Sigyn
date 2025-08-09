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
      current_pose_(),
      current_velocity_(),
      prev_encoder_m1_(0),
      prev_encoder_m2_(0),
      last_odom_update_time_us_(0),
      odometry_initialized_(false),
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
      // HIGH FREQUENCY OPERATIONS (aim for ≥70Hz)
      // Odometry needs fresh encoder data, so read encoders first, then calculate odometry
      if (now - last_reading_time_ms_ >= 15) { // ~67Hz for encoder readings + odometry
        updateCriticalMotorStatus(); // Read fresh encoder values
        updateOdometry();           // Calculate odometry with fresh data
        last_reading_time_ms_ = now;
      }
      
      // HIGH FREQUENCY cmd_vel processing (can be independent)
      processVelocityCommands(); // Critical: cmd_vel processing at high frequency
      
      // MEDIUM FREQUENCY OPERATIONS (~10Hz) - safety checks
      if (now - last_safety_check_time_ms_ >= 100) { // 10Hz safety checks
        checkSafetyConditions();
        last_safety_check_time_ms_ = now;
      }
      
      // Handle command timeouts (check every loop but lightweight)
      if (now - last_velocity_command_.timestamp_ms > MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS) {
        // Timeout - stop motors
        setMotorSpeeds(0, 0);
      }
      
      break;
  }
  
  // LOW FREQUENCY OPERATIONS (~3Hz) - heavy system status
  if (now - last_status_report_time_ms_ >= 333) { // ~3Hz for heavy readings (voltage, current, etc.)
    updateSystemStatus(); // Voltage, current, error reads - slow operations
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }
  
  // DIAGNOSTIC REPORTING (~1Hz)
  if (now - last_diagnostic_report_time_ms_ >= 1000) { // 1Hz for diagnostics
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
    SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Attempting to clear RoboClaw errors");
    
    // Method 1: Stop all motor commands (sometimes clears latched errors)
    setMotorSpeeds(0, 0);
    
    // Method 2: Reset encoder counts (can clear some error states)
    bool reset_success = roboclaw_.ResetEncoders(config_.address);
    if (reset_success) {
      SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Encoder reset successful");
      // Reinitialize encoder tracking
      prev_encoder_m1_ = 0;
      prev_encoder_m2_ = 0;
      odometry_initialized_ = false; // Force re-initialization
    } else {
      SerialManager::getInstance().sendMessage("WARN", "RoboClawMonitor: Encoder reset failed");
    }
    
    // Check if error cleared
    bool valid;
    uint32_t error_status = roboclaw_.ReadError(config_.address, &valid);
    if (valid) {
      if (error_status == 0) {
        SerialManager::getInstance().sendMessage("INFO", "RoboClawMonitor: Error cleared successfully");
      } else {
        String msg = "RoboClawMonitor: Errors remain after reset: " + decodeErrorStatus(error_status);
        SerialManager::getInstance().sendMessage("WARN", msg.c_str());
      }
      system_status_.error_status = error_status;
    }
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

  // Set Max current limits (if applicable)
  if (!roboclaw_.SetM1MaxCurrent(config_.address, config_.max_current_m1 * 100) ||
      !roboclaw_.SetM2MaxCurrent(config_.address, config_.max_current_m2 * 100)) {
    SerialManager::getInstance().sendMessage("ERROR", "RoboClawMonitor: Failed to set max current limits");
    return false;
  } else {
    uint32_t m1_max_current;
    uint32_t m2_max_current;
    roboclaw_.ReadM1MaxCurrent(config_.address, m1_max_current);
    roboclaw_.ReadM2MaxCurrent(config_.address, m2_max_current);
    SerialManager::getInstance().sendMessage("INFO", 
      ("RoboClawMonitor: Max current limits set - M1: " + String(m1_max_current / 100.0f) + 
      "A, M2: " + String(m2_max_current / 100.0f) + "A").c_str());
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
  
  uint32_t m1_max_distance = abs(m1_qpps * MAX_SECONDS_COMMANDED_TRAVEL);
  uint32_t m2_max_distance = abs(m2_qpps * MAX_SECONDS_COMMANDED_TRAVEL);
  
  bool success = roboclaw_.SpeedAccelDistanceM1M2(
    config_.address,
    config_.default_acceleration,
    m1_qpps, m1_max_distance,
    m2_qpps, m2_max_distance,
    1  // Flag: 1 = buffered mode
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
  if (now - last_reading_time_ms_ < 25) { // Increased to 25ms for better performance (40Hz max)
    return;
  }
  
  bool valid;
  uint8_t status;
  
  // State machine to spread serial reads across multiple loop cycles
  switch (reading_state_) {
    case ReadingState::READ_ENCODER_M1:
      motor1_status_.encoder_count = roboclaw_.ReadEncM1(config_.address, &status, &valid);
      motor1_status_.encoder_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_SPEED_M1;
      break;
      
    case ReadingState::READ_SPEED_M1:
      motor1_status_.speed_qpps = roboclaw_.ReadSpeedM1(config_.address, &status, &valid);
      motor1_status_.speed_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_ENCODER_M2;
      break;
      
    case ReadingState::READ_ENCODER_M2:
      motor2_status_.encoder_count = roboclaw_.ReadEncM2(config_.address, &status, &valid);
      motor2_status_.encoder_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_SPEED_M2;
      break;
      
    case ReadingState::READ_SPEED_M2:
      motor2_status_.speed_qpps = roboclaw_.ReadSpeedM2(config_.address, &status, &valid);
      motor2_status_.speed_valid = valid;
      // Continue even if invalid to maintain performance
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
          // Continue even if invalid to maintain performance
        }
        reading_state_ = ReadingState::READ_VOLTAGES;
      }
      break;
      
    case ReadingState::READ_VOLTAGES:
      // This could be split further if needed, but voltages are typically fast
      updateSystemStatus();  // This contains voltage and error reads
      reading_state_ = ReadingState::READ_ERROR_STATUS;
      break;
      
    case ReadingState::READ_ERROR_STATUS:
      // Read error status directly here for better control
      system_status_.error_status = roboclaw_.ReadError(config_.address, &valid);
      reading_state_ = ReadingState::COMPLETE;
      break;
      
    case ReadingState::COMPLETE:
      // All readings complete for this cycle
      // Only set communication_ok if we had some valid readings
      bool any_valid = motor1_status_.encoder_valid || motor1_status_.speed_valid ||
                      motor2_status_.encoder_valid || motor2_status_.speed_valid ||
                      motor1_status_.current_valid;
      
      motor1_status_.communication_ok = any_valid;
      motor2_status_.communication_ok = any_valid;
      
      // Track consecutive communication failures for error handling
      static uint8_t consecutive_failures = 0;
      if (!any_valid) {
        consecutive_failures++;
        if (consecutive_failures >= 5) { // Only trigger error after 5 consecutive failures
          handleCommunicationError();
          consecutive_failures = 0;
        }
      } else {
        consecutive_failures = 0; // Reset on success
      }
      
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
  
  switch (system_read_counter % 8) { // Increased cycle count to reduce frequency further
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
      // Skip this cycle to reduce load
      break;
      
    case 2:
      // Read error status (critical - read more frequently)
      system_status_.error_status = roboclaw_.ReadError(config_.address, &valid);
      break;
      
    case 3:
      // Skip this cycle to reduce load
      break;
      
    case 4:
      // Read logic battery voltage (less frequent)
      {
        uint16_t logic_voltage = roboclaw_.ReadLogicBatteryVoltage(config_.address, &valid);
        if (valid) {
          system_status_.logic_battery_voltage = logic_voltage / 10.0f;
        }
      }
      break;
      
    case 5:
      // Skip this cycle to reduce load
      break;
      
    case 6:
      // Skip this cycle to reduce load  
      break;
      
    case 7:
      // Read temperature if available (slowest, do least frequently)
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

void RoboClawMonitor::updateCriticalMotorStatus() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }
  
  // High-frequency updates: ALWAYS read encoders (critical for odometry)
  bool valid;
  uint8_t status;
  
  // Read encoders first - these are critical for odometry
  // Add retry logic for encoder reads
  static uint32_t encoder_fail_count_m1 = 0;
  static uint32_t encoder_fail_count_m2 = 0;
  
  motor1_status_.encoder_count = roboclaw_.ReadEncM1(config_.address, &status, &valid);
  motor1_status_.encoder_valid = valid;
  if (!valid) {
    encoder_fail_count_m1++;
    // Try one immediate retry for encoder reads (no delay to maintain performance)
    motor1_status_.encoder_count = roboclaw_.ReadEncM1(config_.address, &status, &valid);
    motor1_status_.encoder_valid = valid;
  }

  motor2_status_.encoder_count = roboclaw_.ReadEncM2(config_.address, &status, &valid);
  motor2_status_.encoder_valid = valid;
  if (!valid) {
    encoder_fail_count_m2++;
    // Try one immediate retry for encoder reads (no delay to maintain performance)
    motor2_status_.encoder_count = roboclaw_.ReadEncM2(config_.address, &status, &valid);
    motor2_status_.encoder_valid = valid;
  }
  
  // Log persistent encoder failures
  static uint32_t last_fail_report = 0;
  uint32_t now = millis();
  if (now - last_fail_report > 10000) { // Every 10 seconds
    if (encoder_fail_count_m1 > 0 || encoder_fail_count_m2 > 0) {
      String msg = "Encoder fails - M1:" + String(encoder_fail_count_m1) + 
                   " M2:" + String(encoder_fail_count_m2);
      SerialManager::getInstance().sendMessage("WARN", msg.c_str());
      encoder_fail_count_m1 = 0;
      encoder_fail_count_m2 = 0;
    }
    last_fail_report = now;
  }
  
  // Read speeds less frequently to reduce serial load
  static uint8_t speed_read_counter = 0;
  if ((speed_read_counter % 3) == 0) { // Read speeds every 3rd call (~23Hz)
    motor1_status_.speed_qpps = roboclaw_.ReadSpeedM1(config_.address, &status, &valid);
    motor1_status_.speed_valid = valid;
    
    motor2_status_.speed_qpps = roboclaw_.ReadSpeedM2(config_.address, &status, &valid);
    motor2_status_.speed_valid = valid;
  }
  speed_read_counter++;
}

void RoboClawMonitor::updateOdometry() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    // Log why ODOM is not being sent
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time > 5000) { // Log every 5 seconds
      String msg = "ODOM: Skipping - not connected, state=" + String(static_cast<int>(connection_state_));
      SerialManager::getInstance().sendMessage("DEBUG", msg.c_str());
      last_log_time = now;
    }
    return;
  }
  
  uint32_t current_time_us = micros();
  
  // Initialize odometry on first run
  if (!odometry_initialized_) {
    prev_encoder_m1_ = motor1_status_.encoder_count;
    prev_encoder_m2_ = motor2_status_.encoder_count;
    last_odom_update_time_us_ = current_time_us;
    odometry_initialized_ = true;
    SerialManager::getInstance().sendMessage("DEBUG", "ODOM: Initialized");
    return;
  }
  
  // Calculate time delta
  float dt_s = (current_time_us - last_odom_update_time_us_) / 1000000.0f;
  
  // Skip if time delta is too small (avoid noise) or reset if too large (system overload)
  if (dt_s < 0.010f) {
    static uint32_t last_dt_log = 0;
    uint32_t now = millis();
    if (now - last_dt_log > 10000) { // Log every 10 seconds
      String msg = "ODOM: Skipping - dt too small=" + String(dt_s, 6) + "s";
      SerialManager::getInstance().sendMessage("DEBUG", msg.c_str());
      last_dt_log = now;
    }
    return;
  }
  
  if (dt_s > 0.1f) {
    // System overload - reset timing and continue (don't block ODOM indefinitely)
    static uint32_t last_reset_log = 0;
    uint32_t now = millis();
    if (now - last_reset_log > 5000) { // Log every 5 seconds
      String msg = "ODOM: Reset timing - dt was " + String(dt_s, 6) + "s (system overload)";
      SerialManager::getInstance().sendMessage("WARN", msg.c_str());
      last_reset_log = now;
    }
    
    // Reset timing but preserve encoder state for next update
    prev_encoder_m1_ = motor1_status_.encoder_count;
    prev_encoder_m2_ = motor2_status_.encoder_count;
    last_odom_update_time_us_ = current_time_us;
    
    // Send zero-velocity ODOM to maintain ROS connection
    char msg[256];
    float half_theta = current_pose_.theta / 2.0f;
    snprintf(msg, sizeof(msg),
             "px=%.3f,py=%.3f,ox=%.3f,oy=%.3f,oz=%.3f,ow=%.3f,vx=%.3f,vy=%.3f,wz=%.3f",
             current_pose_.x, current_pose_.y, 0.0f, 0.0f, sin(half_theta), cos(half_theta),
             0.0f, 0.0f, 0.0f);
    SerialManager::getInstance().sendMessage("ODOM", msg);
    return;
  }
  
  // Use the latest encoder readings from motor status
  if (!motor1_status_.encoder_valid || !motor2_status_.encoder_valid) {
    // Log encoder validity issues
    static uint32_t last_enc_log = 0;
    static uint32_t enc_fail_count = 0;
    uint32_t now = millis();
    enc_fail_count++;
    
    if (now - last_enc_log > 2000) { // Log every 2 seconds
      String msg = "ODOM: Encoder invalid - M1:" + String(motor1_status_.encoder_valid ? "OK" : "FAIL") + 
                   " M2:" + String(motor2_status_.encoder_valid ? "OK" : "FAIL") + 
                   " fails:" + String(enc_fail_count);
      SerialManager::getInstance().sendMessage("WARN", msg.c_str());
      last_enc_log = now;
      enc_fail_count = 0;
    }
    return; // Skip if encoder readings invalid
  }
  
  // Calculate encoder deltas
  int32_t delta_encoder_m1 = motor1_status_.encoder_count - prev_encoder_m1_;
  int32_t delta_encoder_m2 = motor2_status_.encoder_count - prev_encoder_m2_;
  
  // Send odometry periodically even when stationary (like legacy implementation)
  // This ensures ROS navigation stack receives regular odometry updates
  bool has_movement = (abs(delta_encoder_m1) >= 2 || abs(delta_encoder_m2) >= 2);
  bool should_send_periodic = (dt_s >= 0.033f); // Send at least every 33ms (~30Hz) like legacy
  
  if (!has_movement && !should_send_periodic) {
    return; // Skip if no movement and not time for periodic update
  }
  
  // Update previous values
  prev_encoder_m1_ = motor1_status_.encoder_count;
  prev_encoder_m2_ = motor2_status_.encoder_count;
  last_odom_update_time_us_ = current_time_us;
  
  // Convert encoder ticks to distances (meters)
  float wheel_circumference = M_PI * WHEEL_DIAMETER_M;
  float dist_m1 = (static_cast<float>(delta_encoder_m1) / QUADRATURE_PULSES_PER_REVOLUTION) * wheel_circumference;
  float dist_m2 = (static_cast<float>(delta_encoder_m2) / QUADRATURE_PULSES_PER_REVOLUTION) * wheel_circumference;
  
  // Calculate robot motion
  float delta_distance = (dist_m1 + dist_m2) / 2.0f;
  float delta_theta = (dist_m2 - dist_m1) / WHEEL_BASE_M;
  
  // Update pose (using midpoint integration) - only if there's actual movement
  if (has_movement) {
    current_pose_.x += delta_distance * cos(current_pose_.theta + delta_theta / 2.0f);
    current_pose_.y += delta_distance * sin(current_pose_.theta + delta_theta / 2.0f);
    current_pose_.theta += delta_theta;
    
    // Normalize angle to [-π, π]
    while (current_pose_.theta > M_PI) current_pose_.theta -= 2.0f * M_PI;
    while (current_pose_.theta < -M_PI) current_pose_.theta += 2.0f * M_PI;
    
    // Update velocity
    current_velocity_.linear_x = delta_distance / dt_s;
    current_velocity_.angular_z = delta_theta / dt_s;
  } else {
    // Stationary: set velocities to zero but keep pose unchanged
    current_velocity_.linear_x = 0.0f;
    current_velocity_.angular_z = 0.0f;
  }
  
  // Send odometry message (compact format for high frequency)
  float q[4]; // quaternion [w, x, y, z]
  float half_theta = current_pose_.theta / 2.0f;
  q[0] = cos(half_theta); // w
  q[1] = 0.0f;            // x
  q[2] = 0.0f;            // y  
  q[3] = sin(half_theta); // z
  
  char msg[256];
  snprintf(msg, sizeof(msg),
           "px=%.3f,py=%.3f,ox=%.3f,oy=%.3f,oz=%.3f,ow=%.3f,vx=%.3f,vy=%.3f,wz=%.3f",
           current_pose_.x, current_pose_.y, q[1], q[2], q[3], q[0],
           current_velocity_.linear_x, 0.0f, current_velocity_.angular_z);
  SerialManager::getInstance().sendMessage("ODOM", msg);
  
  // Track ODOM health statistics
  static uint32_t odom_sent_count = 0;
  static uint32_t last_health_report = 0;
  odom_sent_count++;
  
  uint32_t now = millis();
  if (now - last_health_report > 30000) { // Every 30 seconds
    String msg = "ODOM: Sent " + String(odom_sent_count) + " messages in 30s, freq=" + 
                 String(odom_sent_count / 30.0f, 1) + "Hz";
    SerialManager::getInstance().sendMessage("DEBUG", msg.c_str());
    last_health_report = now;
    odom_sent_count = 0;
  }
}

void RoboClawMonitor::processVelocityCommands() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }
  
  // Check for new TWIST commands from SerialManager
  if (SerialManager::getInstance().hasNewTwistCommand()) {
    String twist_data = SerialManager::getInstance().getLatestTwistCommand();
    handleTwistMessage(twist_data);
  }
  
  uint32_t now = millis();
  
  // Check for command timeout
  if (now - last_velocity_command_.timestamp_ms > MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS) {
    // Timeout - ensure motors are stopped
    if (last_commanded_m1_qpps_ != 0 || last_commanded_m2_qpps_ != 0) {
      setMotorSpeeds(0, 0);
    }
    return;
  }
  
  // Rate limit motor commands to prevent overwhelming the controller
  // But still allow high frequency for responsiveness  
  static uint32_t last_command_time = 0;
  if (now - last_command_time < 15) { // ~67Hz max command rate
    return;
  }
  last_command_time = now;
  
  // Convert twist to motor speeds
  int32_t m1_qpps, m2_qpps;
  velocityToMotorSpeeds(last_velocity_command_.linear_x, 
                       last_velocity_command_.angular_z, 
                       m1_qpps, m2_qpps);
  
  // Only send command if it changed significantly or periodically
  static uint32_t last_forced_update = 0;
  bool significant_change = (abs(m1_qpps - last_commanded_m1_qpps_) > 10) || 
                           (abs(m2_qpps - last_commanded_m2_qpps_) > 10);
  bool force_update = (now - last_forced_update > 100); // Force update every 100ms
  
  if (significant_change || force_update) {
    executeMotorCommand(m1_qpps, m2_qpps);
    last_commanded_m1_qpps_ = m1_qpps;
    last_commanded_m2_qpps_ = m2_qpps;
    last_command_time_ms_ = now;
    
    if (force_update) {
      last_forced_update = now;
    }
  }
}

void RoboClawMonitor::checkSafetyConditions() {
  // Rate limiting for ESTOP messages
  static uint32_t last_estop_msg_time = 0;
  const uint32_t ESTOP_MSG_INTERVAL = 1000; // 1 second between ESTOP messages
  uint32_t now = millis();
  
  // Check for overcurrent conditions
  if (motor1_status_.current_valid && 
      abs(motor1_status_.current_amps) > config_.max_current_m1) {
    motor1_status_.overcurrent = true;
    total_safety_violations_++;
    
    // Rate-limited ESTOP message
    if (now - last_estop_msg_time >= ESTOP_MSG_INTERVAL) {
      SerialManager::getInstance().sendMessage("ESTOP",
        ("active:true,source:ROBOCLAW_CURRENT,reason:Motor 1 overcurrent," +
        String("value:") + String(motor1_status_.current_amps) +
        ",manual_reset:false,time:" + String(millis())).c_str());
      last_estop_msg_time = now;
    }
  }
  
  if (motor2_status_.current_valid && 
      abs(motor2_status_.current_amps) > config_.max_current_m2) {
    motor2_status_.overcurrent = true;
    total_safety_violations_++;
    
    // Rate-limited ESTOP message
    if (now - last_estop_msg_time >= ESTOP_MSG_INTERVAL) {
      SerialManager::getInstance().sendMessage("ESTOP",
        ("active:true,source:ROBOCLAW_CURRENT,reason:Motor 2 overcurrent," +
        String("value:") + String(motor2_status_.current_amps) +
        ",manual_reset:false,time:" + String(millis())).c_str());
      last_estop_msg_time = now;
    }
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
  
  // Rate limiting for runaway ESTOP messages
  static uint32_t last_runaway_msg_time = 0;
  const uint32_t RUNAWAY_MSG_INTERVAL = 1000; // 1 second between runaway messages
  
  // Check if motors are running away (moving when commanded to stop)
  if (last_commanded_m1_qpps_ == 0 && abs(motor1_status_.speed_qpps) > 100) {
    motor1_status_.runaway_detected = true;
    total_safety_violations_++;
    
    // Rate-limited ESTOP message
    if (now - last_runaway_msg_time >= RUNAWAY_MSG_INTERVAL) {
      SerialManager::getInstance().sendMessage("ESTOP",
        ("active:true,source:MOTOR_RUNAWAY,reason:Motor 1 runaway detected," +
        String("value:") + String(motor1_status_.speed_qpps) +
        ",manual_reset:false,time:" + String(millis())).c_str());
      last_runaway_msg_time = now;
    }
  }
  
  if (last_commanded_m2_qpps_ == 0 && abs(motor2_status_.speed_qpps) > 100) {
    motor2_status_.runaway_detected = true;
    total_safety_violations_++;
    
    // Rate-limited ESTOP message
    if (now - last_runaway_msg_time >= RUNAWAY_MSG_INTERVAL) {
      SerialManager::getInstance().sendMessage("ESTOP",
        ("active:true,source:MOTOR_RUNAWAY,reason:Motor 2 runaway detected," +
        String("value:") + String(motor2_status_.speed_qpps) +
        ",manual_reset:false,time:" + String(millis())).c_str());
      last_runaway_msg_time = now;
    }
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
  
  // Auto-recovery for latched overcurrent errors when current is actually low
  if (system_status_.error_status != 0) {
    bool m1_overcurrent = (system_status_.error_status & static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT));
    bool m2_overcurrent = (system_status_.error_status & static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT));
    
    // If overcurrent error is flagged but actual current is low, attempt auto-recovery
    if ((m1_overcurrent && motor1_status_.current_valid && abs(motor1_status_.current_amps) < 0.5f) ||
        (m2_overcurrent && motor2_status_.current_valid && abs(motor2_status_.current_amps) < 0.5f)) {
      
      static uint32_t last_auto_recovery = 0;
      uint32_t now = millis();
      
      // Only attempt auto-recovery every 30 seconds to avoid spam
      if (now - last_auto_recovery > 30000) {
        SerialManager::getInstance().sendMessage("INFO", 
          "RoboClawMonitor: Attempting auto-recovery for latched overcurrent error");
        resetErrors();
        last_auto_recovery = now;
      }
    }
  }
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
