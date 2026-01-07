// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file roboclaw_monitor.cpp
 * @brief RoboClaw motor controller monitoring implementation for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "roboclaw_monitor.h"

#include <cstdlib>
#include <cstring>

#include "common/core/config.h"
#include "common/core/serial_manager.h"
#include "safety/safety_coordinator.h"

namespace sigyn_teensy {

// Legacy defaults (kept here to preserve behavior while allowing overrides via RoboClawConfig)
constexpr uint32_t DEFAULT_MAX_MOTOR_SPEED_QPPS = 1392;
constexpr uint32_t DEFAULT_MAX_ACCELERATION_QPPS2 = 3000;

namespace {
constexpr uint32_t kFatalRoboclawErrorMask =
    static_cast<uint32_t>(RoboClawError::E_STOP) |
    static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR) |
    static_cast<uint32_t>(RoboClawError::TEMPERATURE2_ERROR) |
    static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH_ERROR) |
    static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_HIGH_ERROR) |
    static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_LOW_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_SPEED_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_SPEED_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_POSITION_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_POSITION_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_CURRENT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_CURRENT_ERROR);

bool isPowerCycleLikelyRequired(uint32_t error_status) {
  // Based on RoboClaw manual behavior: E-stop may be configured as latching;
  // driver faults suggest damage/latched shutdown.
  // TODO(wimble): Confirm additional power-cycle-required error bits from the manual.
  return (error_status & static_cast<uint32_t>(RoboClawError::E_STOP)) ||
         (error_status & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR)) ||
         (error_status & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR));
}
}  // namespace

// Serial port (define this based on your hardware setup)
#ifndef UNIT_TEST
#define ROBOCLAW_SERIAL Serial7
#endif

RoboClawMonitor::RoboClawMonitor()
    : Module(),
      config_(),
#ifndef UNIT_TEST
  roboclaw_hw_(&ROBOCLAW_SERIAL, config_.timeout_us),
  roboclaw_adapter_(roboclaw_hw_),
  roboclaw_(&roboclaw_adapter_),
#else
  roboclaw_null_(),
  roboclaw_(&roboclaw_null_),
#endif
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
  // Set hardware-specific defaults
  config_.max_speed_qpps = DEFAULT_MAX_MOTOR_SPEED_QPPS;
  config_.default_acceleration = DEFAULT_MAX_ACCELERATION_QPPS2;
}

RoboClawMonitor& RoboClawMonitor::getInstance() {
  static RoboClawMonitor instance;
  return instance;
}

void RoboClawMonitor::setup() {
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Starting initialization");

  // Initialize serial communication
  // roboclaw_->begin(config_.baud_rate);

  connection_state_ = ConnectionState::CONNECTING;

  if (initializeRoboClaw()) {
    connection_state_ = ConnectionState::CONNECTED;
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Initialization successful");
    if (CONTROLS_ROBOCLAW_ESTOP_PIN) {
      pinMode(ESTOP_OUTPUT_PIN, OUTPUT);

      // Test E-stop control of the RoboClaw is functional.
      char msg[128];
      bool valid = false;
      digitalWrite(ESTOP_OUTPUT_PIN, HIGH);  // Deactivate E-stop
      delay(100);
      uint32_t error_status = roboclaw_->ReadError(config_.address, &valid);
      if (valid && !(error_status & static_cast<uint32_t>(RoboClawError::E_STOP))) {
        snprintf(msg, sizeof(msg), "RoboClaw E-stop deactivated successfully");
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
      } else {
        snprintf(msg, sizeof(msg), "RoboClaw E-stop deactivation failed, error_status=0x%08lX, ESTOP_OUTPUT_PIN=%d",
                 (unsigned long)error_status, ESTOP_OUTPUT_PIN);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        connection_state_ = ConnectionState::FAILED;
        return;
      }

      digitalWrite(ESTOP_OUTPUT_PIN, LOW);
      delay(100);
      error_status = roboclaw_->ReadError(config_.address, &valid);
      if (valid && (error_status & static_cast<uint32_t>(RoboClawError::E_STOP))) {
        snprintf(msg, sizeof(msg), "RoboClaw E-stop activated successfully");
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
      } else {
        snprintf(msg, sizeof(msg),
                 "RoboClaw E-stop activation failed. valid: %d, error_status=0x%08lX, ESTOP_OUTPUT_PIN=%d", valid,
                 (unsigned long)error_status, ESTOP_OUTPUT_PIN);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        connection_state_ = ConnectionState::FAILED;
        return;
      }

      digitalWrite(ESTOP_OUTPUT_PIN, HIGH);  // Leave E-stop deactivated
    }
  } else {
    connection_state_ = ConnectionState::ERROR_RECOVERY;
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Initialization failed");
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
    case ConnectionState::FAILED:
      // Do nothing, manual intervention required
      break;

    case ConnectionState::DISCONNECTED:
    case ConnectionState::ERROR_RECOVERY:
      // Attempt reconnection
      if (initializeRoboClaw()) {
        connection_state_ = ConnectionState::CONNECTED;
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Reconnected");
      }
      break;

    case ConnectionState::CONNECTING:
      // Wait for initialization to complete
      break;

    case ConnectionState::CONNECTED:
      // HIGH FREQUENCY OPERATIONS (aim for ≥70Hz)
      // Odometry needs fresh encoder data, so read encoders first, then
      // calculate odometry
      if (now - last_reading_time_ms_ >= 15) {  // ~67Hz for encoder readings + odometry
        updateCriticalMotorStatus();            // Read fresh encoder values
        updateOdometry();                       // Calculate odometry with fresh data
        last_reading_time_ms_ = now;
      }

      // HIGH FREQUENCY cmd_vel processing (can be independent)
      processVelocityCommands();  // Critical: cmd_vel processing at high
                                  // frequency

      // MEDIUM FREQUENCY OPERATIONS (~10Hz) - safety checks
      if (now - last_safety_check_time_ms_ >= 100) {  // 10Hz safety checks
        checkSafetyConditions();
        last_safety_check_time_ms_ = now;
      }

      // Handle command timeouts (check every loop but lightweight)
      if (now - last_velocity_command_.timestamp_ms > config_.cmd_vel_timeout_ms) {
        // Timeout - stop motors
        setMotorSpeeds(0, 0);
      }

      break;
  }

  // LOW FREQUENCY OPERATIONS (~3Hz) - heavy system status
  if (now - last_status_report_time_ms_ >= 333) {  // ~3Hz for heavy readings (voltage, current, etc.)
    updateSystemStatus();                          // Voltage, current, error reads - slow operations
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }

  // DIAGNOSTIC REPORTING (~1Hz)
  if (now - last_diagnostic_report_time_ms_ >= 1000) {  // 1Hz for diagnostics
    sendDiagnosticReports();
    last_diagnostic_report_time_ms_ = now;
  }
}

bool RoboClawMonitor::isUnsafe() {
  char safety_reasons[256] = {0};
  bool unsafe = false;

  auto append_reason = [&](const char* reason) {
    if (!reason) {
      return;
    }
    const size_t len = strlen(safety_reasons);
    if (len >= (sizeof(safety_reasons) - 1)) {
      return;
    }
    snprintf(safety_reasons + len, sizeof(safety_reasons) - len, "%s ", reason);
  };

  // Check local safety flags
  if (emergency_stop_active_) {
    append_reason("emergency_stop_active");
    unsafe = true;
  }

  if (motor1_status_.runaway_detected) {
    append_reason("motor1_runaway");
    unsafe = true;
  }

  if (motor2_status_.runaway_detected) {
    append_reason("motor2_runaway");
    unsafe = true;
  }

  if (motor1_status_.overcurrent) {
    append_reason("motor1_overcurrent");
    unsafe = true;
  }

  if (motor2_status_.overcurrent) {
    append_reason("motor2_overcurrent");
    unsafe = true;
  }

  if (!motor1_status_.communication_ok) {
    append_reason("motor1_comm_fail");
    unsafe = true;
  }

  if (!motor2_status_.communication_ok) {
    append_reason("motor2_comm_fail");
    unsafe = true;
  }

  // Check RoboClaw error status for safety-critical errors
  uint32_t error = system_status_.error_status;
  bool roboclaw_unsafe = (error & static_cast<uint32_t>(RoboClawError::E_STOP)) ||
                         (error & static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::TEMPERATURE2_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_HIGH_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_LOW_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M1_SPEED_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M2_SPEED_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M1_POSITION_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M2_POSITION_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M1_CURRENT_ERROR)) ||
                         (error & static_cast<uint32_t>(RoboClawError::M2_CURRENT_ERROR));

  if (roboclaw_unsafe) {
    char error_details[256] = {0};
    decodeErrorStatus(error, error_details, sizeof(error_details));
    const size_t len = strlen(safety_reasons);
    if (len < (sizeof(safety_reasons) - 1)) {
      const size_t remaining = sizeof(safety_reasons) - len - 1;
      const char* prefix = "roboclaw_hw_error[";
      const char* suffix = "] ";
      const size_t overhead = strlen(prefix) + strlen(suffix);

      const int max_detail = (remaining > overhead) ? static_cast<int>(remaining - overhead) : 0;
      if (max_detail > 0) {
        snprintf(safety_reasons + len, remaining + 1, "roboclaw_hw_error[%.*s] ", max_detail, error_details);
      }
    }
    unsafe = true;
  }

  // Log safety status if unsafe condition detected
  if (unsafe && (safety_reasons[0] != '\0')) {
    char msg[320] = {0};
    snprintf(msg, sizeof(msg), "Unsafe condition detected - reasons: %s", safety_reasons);
    SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
  }

  return unsafe;
}

void RoboClawMonitor::resetSafetyFlags() {
  if (CONTROLS_ROBOCLAW_ESTOP_PIN) {
    digitalWrite(ESTOP_OUTPUT_PIN, HIGH);  // Deactivate E-stop
  }
  emergency_stop_active_ = false;
  motor1_status_.runaway_detected = false;
  motor2_status_.runaway_detected = false;
  motor1_status_.overcurrent = false;
  motor2_status_.overcurrent = false;
  motor1_status_.timeout_error = false;
  motor2_status_.timeout_error = false;

  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Safety flags and E-stop pin reset");
}

void RoboClawMonitor::setVelocityCommand(float linear_x, float angular_z) {
  last_velocity_command_.linear_x = linear_x;
  last_velocity_command_.angular_z = angular_z;
  last_velocity_command_.timestamp_ms = millis();

  // If we receive a zero velocity command, check if we're in a runaway state and try to clear it
  // This helps recover if the runaway was false positive or transient
  if (linear_x == 0.0f && angular_z == 0.0f) {
      if (motor1_status_.runaway_detected || motor2_status_.runaway_detected) {
          motor1_status_.runaway_detected = false;
          motor2_status_.runaway_detected = false;
          // Note: We do NOT clear other faults (overcurrent/timeout) here.
          // The E-stop state will be cleared in checkSafetyConditions() only if all
          // safety conditions (including runaway, overcurrent, temp) are satisfied.
      }
  }

  // NOTE: We do NOT send the command to the motors here anymore.
  // To implement proper rate limiting (command_rate_limit_ms), actual transmission 
  // is handled in processVelocityCommands().
  // This prevents swamping the RoboClaw message buffer if high-frequency twist messages arrive.
}

void RoboClawMonitor::setMotorSpeeds(int32_t m1_qpps, int32_t m2_qpps) {
  if (emergency_stop_active_) {
    m1_qpps = 0;
    m2_qpps = 0;
  }

  // Constrain to maximum speeds
  m1_qpps =
      constrain(m1_qpps, -static_cast<int32_t>(config_.max_speed_qpps), static_cast<int32_t>(config_.max_speed_qpps));
  m2_qpps =
      constrain(m2_qpps, -static_cast<int32_t>(config_.max_speed_qpps), static_cast<int32_t>(config_.max_speed_qpps));

  executeMotorCommand(m1_qpps, m2_qpps);

  // Update command tracking
  last_commanded_m1_qpps_ = m1_qpps;
  last_commanded_m2_qpps_ = m2_qpps;
  last_command_time_ms_ = millis();
  total_commands_sent_++;
}

void RoboClawMonitor::clearEmergencyStop() {
  SerialManager::getInstance().sendDiagnosticMessage(
      "INFO", name(), "Clearing emergency stop command received, attempting to clear E-stop");
  digitalWrite(ESTOP_OUTPUT_PIN, HIGH);  // Deactivate E-stop.
  emergency_stop_active_ = false;
  char msg[128] = {0};
  snprintf(msg, sizeof(msg), "active:false,source:ROBOCLAW,reason:Emergency stop cleared,time:%lu",
           static_cast<unsigned long>(millis()));
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
}

void RoboClawMonitor::setEmergencyStop() {
  // Prefer the physical E-stop output pin when available, rather than trying
  // to send stop commands over a potentially-failing serial link.
  pinMode(ESTOP_OUTPUT_PIN, OUTPUT);
  digitalWrite(ESTOP_OUTPUT_PIN, LOW);
  emergency_stop_active_ = true;

#if !CONTROLS_ROBOCLAW_ESTOP_PIN
  // If we *can't* assert a real E-stop line to the RoboClaw, best-effort stop.
  setMotorSpeeds(0, 0);
#endif
  char msg[96] = {0};
  snprintf(msg, sizeof(msg), "Emergency stop activated,time:%lu", static_cast<unsigned long>(millis()));
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
}

void RoboClawMonitor::resetErrors() {
  resetSafetyFlags();
  total_communication_errors_ = 0;
  total_safety_violations_ = 0;

  // Attempt to clear RoboClaw errors if connected
  if (connection_state_ == ConnectionState::CONNECTED) {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Attempting to clear RoboClaw errors");

    // Method 1: Stop all motor commands (sometimes clears latched errors)
    setMotorSpeeds(0, 0);

    // Method 2: Reset encoder counts (can clear some error states)
    bool reset_success = roboclaw_->ResetEncoders(config_.address);
    if (reset_success) {
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Encoder reset successful");
      // Reinitialize encoder tracking
      prev_encoder_m1_ = 0;
      prev_encoder_m2_ = 0;
      odometry_initialized_ = false;  // Force re-initialization
    } else {
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), "Encoder reset failed");
    }

    // Check if error cleared
    bool valid;
    uint32_t error_status = roboclaw_->ReadError(config_.address, &valid);
    if (valid) {
      if (error_status == 0) {
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), "Error cleared successfully");
      } else {
        char decoded[256] = {0};
        decodeErrorStatus(error_status, decoded, sizeof(decoded));
        char msg[320] = {0};
        snprintf(msg, sizeof(msg), "Errors remain after reset: %s", decoded);
        SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      }
      system_status_.error_status = error_status;
    }
  }
}

void RoboClawMonitor::handleTwistMessage(const char* data) {
  // Parse twist message: "linear_x:<value>,angular_z:<value>"
  float linear_x = 0.0f;
  float angular_z = 0.0f;

  if (data) {
    const char* linear_start = strstr(data, "linear_x:");
    if (linear_start) {
      linear_start += strlen("linear_x:");
      linear_x = strtof(linear_start, nullptr);
    }

    const char* angular_start = strstr(data, "angular_z:");
    if (angular_start) {
      angular_start += strlen("angular_z:");
      angular_z = strtof(angular_start, nullptr);
    }
  }

  setVelocityCommand(linear_x, angular_z);
}

bool RoboClawMonitor::initializeRoboClaw() {
  // Test communication
  if (!testCommunication()) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Communication test failed");
    return false;
  }

  // Set Max current limits (if applicable)
    if (!roboclaw_->SetM1MaxCurrent(config_.address, config_.max_current_m1 * 100) ||
      !roboclaw_->SetM2MaxCurrent(config_.address, config_.max_current_m2 * 100)) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to set max current limits");
    return false;
  } else {
    uint32_t m1_max_current;
    uint32_t m2_max_current;
    roboclaw_->ReadM1MaxCurrent(config_.address, m1_max_current);
    roboclaw_->ReadM2MaxCurrent(config_.address, m2_max_current);
    char msg[128] = {0};
    snprintf(msg, sizeof(msg), "Max current limits set - M1: %.2fA, M2: %.2fA",
             static_cast<double>(m1_max_current / 100.0f), static_cast<double>(m2_max_current / 100.0f));
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
  }

  // Set PID values (ported from legacy code)
  if (!roboclaw_->SetM1VelocityPID(config_.address, 7.26239f, 2.43f, 0.0f, 2437)) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to set M1 PID");
    return false;
  }

  if (!roboclaw_->SetM2VelocityPID(config_.address, 7.26239f, 2.43f, 0.0f, 2437)) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to set M2 PID");
    return false;
  }

  // Initialize encoder readings for runaway detection
  bool valid1, valid2;
  last_runaway_encoder_m1_ = roboclaw_->ReadEncM1(config_.address, nullptr, &valid1);
  last_runaway_encoder_m2_ = roboclaw_->ReadEncM2(config_.address, nullptr, &valid2);

  if (valid1 && valid2) {
    runaway_detection_initialized_ = true;
  } else {
    SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), "Failed to read initial encoder values");
  }

  return true;
}

bool RoboClawMonitor::testCommunication() {
  char version[64];
  bool success = roboclaw_->ReadVersion(config_.address, version);

  if (success) {
    // Check for exact version match
    if (strcmp(version, "USB Roboclaw 2x15a v4.3.6\n") == 0) {
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "Version check passed: %s", version);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
      return true;
    } else {
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "Unexpected version: %s", version);
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);

      // Safety: if we can't positively identify the controller, stop motion.
      SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                     "RoboClaw communication failed (unexpected version)");
      setEmergencyStop();
      return false;
    }
  } else {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Failed to read version");

    // Safety: inability to communicate with the motor controller warrants E-stop.
    SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                   "RoboClaw communication failed (no version read)");
    setEmergencyStop();
    return false;
  }
}

void RoboClawMonitor::handleCommunicationError() {
  total_communication_errors_++;
  connection_state_ = ConnectionState::ERROR_RECOVERY;

  // Clear status flags
  motor1_status_.communication_ok = false;
  motor2_status_.communication_ok = false;

  // Safety: communication errors mean we cannot reliably control motion.
  SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                 "RoboClaw communication error");
  setEmergencyStop();

  SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), "Communication error detected");
}

void RoboClawMonitor::velocityToMotorSpeeds(float linear_x, float angular_z, int32_t& m1_qpps, int32_t& m2_qpps) {
  // Convert twist to differential drive wheel speeds
  float v_left_mps = linear_x - (angular_z * config_.wheel_base_m / 2.0f);
  float v_right_mps = linear_x + (angular_z * config_.wheel_base_m / 2.0f);

  // Convert m/s to RPM
  float rpm_left = (v_left_mps * 60.0f) / (M_PI * config_.wheel_diameter_m);
  float rpm_right = (v_right_mps * 60.0f) / (M_PI * config_.wheel_diameter_m);

  // Convert RPM to QPPS
  m1_qpps = static_cast<int32_t>((rpm_left / 60.0f) * config_.quadrature_pulses_per_revolution);
  m2_qpps = static_cast<int32_t>((rpm_right / 60.0f) * config_.quadrature_pulses_per_revolution);
}

void RoboClawMonitor::executeMotorCommand(int32_t m1_qpps, int32_t m2_qpps) {
  if (connection_state_ != ConnectionState::CONNECTED) {
    return;
  }

  // Use SpeedAccelM1M2 (Mix Mode) for continuous velocity control.
  // This avoids "stuttering" issues caused by Distance commands finishing before
  // the next cmd_vel arrives.
  // Safety is handled by the software watchdog in loop(), which stops motors if commands timeout.
  bool success = roboclaw_->SpeedAccelM1M2(config_.address, config_.default_acceleration, m1_qpps, m2_qpps);

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
  if (now - last_reading_time_ms_ < 25) {  // Increased to 25ms for better performance (40Hz max)
    return;
  }

  bool valid;
  uint8_t status;

  // State machine to spread serial reads across multiple loop cycles
  switch (reading_state_) {
    case ReadingState::READ_ENCODER_M1:
      motor1_status_.encoder_count = roboclaw_->ReadEncM1(config_.address, &status, &valid);
      motor1_status_.encoder_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_SPEED_M1;
      break;

    case ReadingState::READ_SPEED_M1:
      motor1_status_.speed_qpps = roboclaw_->ReadSpeedM1(config_.address, &status, &valid);
      motor1_status_.speed_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_ENCODER_M2;
      break;

    case ReadingState::READ_ENCODER_M2:
      motor2_status_.encoder_count = roboclaw_->ReadEncM2(config_.address, &status, &valid);
      motor2_status_.encoder_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_SPEED_M2;
      break;

    case ReadingState::READ_SPEED_M2:
      motor2_status_.speed_qpps = roboclaw_->ReadSpeedM2(config_.address, &status, &valid);
      motor2_status_.speed_valid = valid;
      // Continue even if invalid to maintain performance
      reading_state_ = ReadingState::READ_CURRENTS;
      break;

    case ReadingState::READ_CURRENTS: {
      int16_t current1, current2;
      bool current_valid = roboclaw_->ReadCurrents(config_.address, current1, current2);

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
    } break;

    case ReadingState::READ_VOLTAGES:
      // This could be split further if needed, but voltages are typically fast
      updateSystemStatus();  // This contains voltage and error reads
      reading_state_ = ReadingState::READ_ERROR_STATUS;
      break;

    case ReadingState::READ_ERROR_STATUS:
      // Read error status directly here for better control
      system_status_.error_status = roboclaw_->ReadError(config_.address, &valid);
      reading_state_ = ReadingState::COMPLETE;
      break;

    case ReadingState::COMPLETE:
      // All readings complete for this cycle
      // Only set communication_ok if we had some valid readings
      bool any_valid = motor1_status_.encoder_valid || motor1_status_.speed_valid || motor2_status_.encoder_valid ||
                       motor2_status_.speed_valid || motor1_status_.current_valid || motor2_status_.current_valid;

      motor1_status_.communication_ok =
          motor1_status_.encoder_valid || motor1_status_.speed_valid || motor1_status_.current_valid;
      motor2_status_.communication_ok =
          motor2_status_.encoder_valid || motor2_status_.speed_valid || motor2_status_.current_valid;

      if (!any_valid) {
        char msg[300];
        snprintf(msg, sizeof(msg),
                 "No valid readings from RoboClaw at address 0x%02X, consecutive "
                 "failures: %u, motor1_status.encoder_valid: %d, "
                 "motor1_status.speed_valid: %d, motor1_status.current_valid: %d, "
                 "motor2_status.encoder_valid: %d, motor2_status.speed_valid: %d, "
                 "motor2_status.current_valid: %d",
                 config_.address, consecutive_comm_failures_, motor1_status_.encoder_valid, motor1_status_.speed_valid,
                 motor1_status_.current_valid, motor2_status_.encoder_valid, motor2_status_.speed_valid,
                 motor2_status_.current_valid);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), msg);
        consecutive_comm_failures_++;
        if (consecutive_comm_failures_ >= config_.max_consecutive_comm_failures) {
          SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                         "RoboClaw read failures exceeded threshold");
          setEmergencyStop();
          handleCommunicationError();
          consecutive_comm_failures_ = 0;
        }
      } else {
        consecutive_comm_failures_ = 0;  // Reset on success
      }

      reading_state_ = ReadingState::READ_ENCODER_M1;  // Start next cycle
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

  switch (system_read_counter % 8) {  // Increased cycle count to reduce frequency further
    case 0:
      // Read main battery voltage
      {
        uint16_t main_voltage = roboclaw_->ReadMainBatteryVoltage(config_.address, &valid);
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
      system_status_.error_status = roboclaw_->ReadError(config_.address, &valid);
      break;

    case 3:
      // Skip this cycle to reduce load
      break;

    case 4:
      // Read logic battery voltage (less frequent)
      {
        uint16_t logic_voltage = roboclaw_->ReadLogicBatteryVoltage(config_.address, &valid);
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
        if (roboclaw_->ReadTemp(config_.address, temp)) {
          system_status_.temperature_c = temp / 10.0f;

          // TODO(wimble): Raise WARNING fault when temp exceeds roboclaw_temp_warning_c.
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

  motor1_status_.encoder_count = roboclaw_->ReadEncM1(config_.address, &status, &valid);
  motor1_status_.encoder_valid = valid;
  if (!valid) {
    encoder_fail_count_m1++;
    // Try one immediate retry for encoder reads (no delay to maintain
    // performance)
    motor1_status_.encoder_count = roboclaw_->ReadEncM1(config_.address, &status, &valid);
    motor1_status_.encoder_valid = valid;
  }

  motor2_status_.encoder_count = roboclaw_->ReadEncM2(config_.address, &status, &valid);
  motor2_status_.encoder_valid = valid;
  if (!valid) {
    encoder_fail_count_m2++;
    // Try one immediate retry for encoder reads (no delay to maintain
    // performance)
    motor2_status_.encoder_count = roboclaw_->ReadEncM2(config_.address, &status, &valid);
    motor2_status_.encoder_valid = valid;
  }

  // Log persistent encoder failures
  static uint32_t last_fail_report = 0;
  uint32_t now = millis();
  if (now - last_fail_report > 10000) {  // Every 10 seconds
    if (encoder_fail_count_m1 > 0 || encoder_fail_count_m2 > 0) {
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "Encoder fails - M1:%lu M2:%lu", static_cast<unsigned long>(encoder_fail_count_m1),
               static_cast<unsigned long>(encoder_fail_count_m2));
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);

      // TODO(wimble): Escalate encoder read failures via SafetyCoordinator.
      // Example: small consecutive count => WARNING; larger => EMERGENCY_STOP.
      encoder_fail_count_m1 = 0;
      encoder_fail_count_m2 = 0;
    }
    last_fail_report = now;
  }

  // Read speeds less frequently to reduce serial load
  static uint8_t speed_read_counter = 0;
  if ((speed_read_counter % 3) == 0) {  // Read speeds every 3rd call (~23Hz)
    motor1_status_.speed_qpps = roboclaw_->ReadSpeedM1(config_.address, &status, &valid);
    motor1_status_.speed_valid = valid;

    motor2_status_.speed_qpps = roboclaw_->ReadSpeedM2(config_.address, &status, &valid);
    motor2_status_.speed_valid = valid;
  }
  speed_read_counter++;
}

void RoboClawMonitor::updateOdometry() {
  if (connection_state_ != ConnectionState::CONNECTED) {
    // Log why ODOM is not being sent
    static uint32_t last_log_time = 0;
    uint32_t now = millis();
    if (now - last_log_time > 5000) {  // Log every 5 seconds
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "ODOM: Skipping - not connected, state=%d", static_cast<int>(connection_state_));
      SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
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
    SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), "ODOM: Initialized");
    return;
  }

  // Calculate time delta
  float dt_s = (current_time_us - last_odom_update_time_us_) / 1000000.0f;

  // Skip if time delta is too small (avoid noise) or reset if too large (system
  // overload)
  if (dt_s < config_.odom_min_dt_s) {
    static uint32_t last_dt_log = 0;
    uint32_t now = millis();
    if (now - last_dt_log > 10000) {  // Log every 10 seconds
      char msg[96] = {0};
      snprintf(msg, sizeof(msg), "ODOM: Skipping - dt too small=%.6fs", static_cast<double>(dt_s));
      SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
      last_dt_log = now;
    }
    return;
  }

  if (dt_s > config_.odom_max_dt_s) {
    // System overload - reset timing and continue (don't block ODOM
    // indefinitely)
    static uint32_t last_reset_log = 0;
    uint32_t now = millis();
    if (now - last_reset_log > 5000) {  // Log every 5 seconds
      char msg[128] = {0};
      snprintf(msg, sizeof(msg), "ODOM: Reset timing - dt was %.6fs (system overload)", static_cast<double>(dt_s));
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      last_reset_log = now;
    }

    // Reset timing but preserve encoder state for next update
    prev_encoder_m1_ = motor1_status_.encoder_count;
    prev_encoder_m2_ = motor2_status_.encoder_count;
    last_odom_update_time_us_ = current_time_us;

    // Send zero-velocity ODOM to maintain ROS connection (JSON format)
    char msg[256];
    float half_theta = current_pose_.theta / 2.0f;
    snprintf(msg, sizeof(msg),
             "{\"px\":%.3f,\"py\":%.3f,\"ox\":%.3f,\"oy\":%.3f,\"oz\":%.3f,"
             "\"ow\":%.3f,\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f}",
             current_pose_.x, current_pose_.y, 0.0f, 0.0f, sin(half_theta), cos(half_theta), 0.0f, 0.0f, 0.0f);
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

    if (now - last_enc_log > 2000) {  // Log every 2 seconds
      char msg[128] = {0};
      snprintf(msg, sizeof(msg), "ODOM: Encoder invalid - M1:%s M2:%s fails:%lu",
               motor1_status_.encoder_valid ? "OK" : "FAIL", motor2_status_.encoder_valid ? "OK" : "FAIL",
               static_cast<unsigned long>(enc_fail_count));
      SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), msg);
      last_enc_log = now;
      enc_fail_count = 0;
    }
    return;  // Skip if encoder readings invalid
  }

  // Calculate encoder deltas
  int32_t delta_encoder_m1 = motor1_status_.encoder_count - prev_encoder_m1_;
  int32_t delta_encoder_m2 = motor2_status_.encoder_count - prev_encoder_m2_;

  // Send odometry periodically even when stationary (like legacy
  // implementation) This ensures ROS navigation stack receives regular odometry
  // updates
  bool has_movement = (std::abs(delta_encoder_m1) >= config_.odom_movement_threshold_ticks ||
                       std::abs(delta_encoder_m2) >= config_.odom_movement_threshold_ticks);
  bool should_send_periodic = (dt_s >= config_.odom_periodic_send_dt_s);

  if (!has_movement && !should_send_periodic) {
    return;  // Skip if no movement and not time for periodic update
  }

  // Update previous values
  prev_encoder_m1_ = motor1_status_.encoder_count;
  prev_encoder_m2_ = motor2_status_.encoder_count;
  last_odom_update_time_us_ = current_time_us;

  // Convert encoder ticks to distances (meters)
  float wheel_circumference = M_PI * config_.wheel_diameter_m;
  float dist_m1 = (static_cast<float>(delta_encoder_m1) / config_.quadrature_pulses_per_revolution) * wheel_circumference;
  float dist_m2 = (static_cast<float>(delta_encoder_m2) / config_.quadrature_pulses_per_revolution) * wheel_circumference;

  // Calculate robot motion
  float delta_distance = (dist_m1 + dist_m2) / 2.0f;
  float delta_theta = (dist_m2 - dist_m1) / config_.wheel_base_m;

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
  float q[4];  // quaternion [w, x, y, z]
  float half_theta = current_pose_.theta / 2.0f;
  q[0] = cos(half_theta);  // w
  q[1] = 0.0f;             // x
  q[2] = 0.0f;             // y
  q[3] = sin(half_theta);  // z

  char msg[256];
  snprintf(msg, sizeof(msg),
           "{\"px\":%.3f,\"py\":%.3f,\"ox\":%.3f,\"oy\":%.3f,\"oz\":%.3f,"
           "\"ow\":%.3f,\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f}",
           current_pose_.x, current_pose_.y, q[1], q[2], q[3], q[0], current_velocity_.linear_x, 0.0f,
           current_velocity_.angular_z);
  SerialManager::getInstance().sendMessage("ODOM", msg);

  // Track ODOM health statistics
  static uint32_t odom_sent_count = 0;
  static uint32_t last_health_report = 0;
  odom_sent_count++;

  uint32_t now = millis();
  if (now - last_health_report > 30000) {  // Every 30 seconds
    char msg[128] = {0};
    snprintf(msg, sizeof(msg), "ODOM: Sent %lu messages in 30s, freq=%.1fHz", static_cast<unsigned long>(odom_sent_count),
             static_cast<double>(odom_sent_count / 30.0f));
    SerialManager::getInstance().sendDiagnosticMessage("DEBUG", name(), msg);
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
    const char* twist_data = SerialManager::getInstance().getLatestTwistCommand();
    handleTwistMessage(twist_data);
  }

  uint32_t now = millis();

  // Check for command timeout
  if (now - last_velocity_command_.timestamp_ms > config_.cmd_vel_timeout_ms) {
    // Timeout - ensure motors are stopped
    if (last_commanded_m1_qpps_ != 0 || last_commanded_m2_qpps_ != 0) {
      setMotorSpeeds(0, 0);
    }
    return;
  }

  // Rate limit motor commands to prevent overwhelming the controller
  // But still allow high frequency for responsiveness
  static uint32_t last_command_time = 0;
  if (now - last_command_time < config_.command_rate_limit_ms) {
    return;
  }
  last_command_time = now;

  // Convert twist to motor speeds
  int32_t m1_qpps, m2_qpps;
  velocityToMotorSpeeds(last_velocity_command_.linear_x, last_velocity_command_.angular_z, m1_qpps, m2_qpps);

  // Only send command if it changed significantly or periodically
  static uint32_t last_forced_update = 0;
  bool significant_change =
      (std::abs(static_cast<int64_t>(m1_qpps) - static_cast<int64_t>(last_commanded_m1_qpps_)) >
       config_.significant_change_qpps) ||
      (std::abs(static_cast<int64_t>(m2_qpps) - static_cast<int64_t>(last_commanded_m2_qpps_)) >
       config_.significant_change_qpps);
    bool force_update = (now - last_forced_update > config_.force_update_ms);

  if (significant_change || force_update) {
    // Route all motor commands through setMotorSpeeds() so clamping and
    // emergency-stop behavior is applied consistently.
    setMotorSpeeds(m1_qpps, m2_qpps);

    if (force_update) {
      last_forced_update = now;
    }
  }
}

void RoboClawMonitor::checkSafetyConditions() {
  // Rate limiting for ESTOP messages
  static uint32_t last_estop_msg_time = 0;
  uint32_t now = millis();

  // RoboClaw internal temperature safety.
  if (!std::isnan(system_status_.temperature_c) && system_status_.temperature_c >= config_.roboclaw_temp_fault_c) {
    char desc[128] = {0};
    snprintf(desc, sizeof(desc), "RoboClaw over-temperature fault: %.1fC", static_cast<double>(system_status_.temperature_c));
    SafetyCoordinator::getInstance().activateFault(
        FaultSeverity::EMERGENCY_STOP, name(), desc);
    setEmergencyStop();
  }

  // RoboClaw fatal error bits.
  if ((system_status_.error_status & kFatalRoboclawErrorMask) != 0) {
    char decoded[256] = {0};
    decodeErrorStatus(system_status_.error_status, decoded, sizeof(decoded));
    char desc[320] = {0};
    snprintf(desc, sizeof(desc), "RoboClaw reported fatal error bits: %s", decoded);
    SafetyCoordinator::getInstance().activateFault(
      FaultSeverity::EMERGENCY_STOP, name(), desc);
    setEmergencyStop();

    if (isPowerCycleLikelyRequired(system_status_.error_status)) {
      // TODO: Power-cycle RoboClaw via PIN_RELAY_ROBOCLAW_POWER to clear latching faults.
      // This is required for certain latching errors (like Logic Battery High) that
      // cannot be cleared by a soft reset or serial command.
    }
  }

  // Check for overcurrent conditions
  // NOTE: Overcurrent is a LATCING fault. It stops the robot to prevent oscillation ("hammering").
  // It must be cleared by an explicit Reset command (Service/Topic) from the host.
  if (motor1_status_.current_valid) {
    if (std::abs(motor1_status_.current_amps) > config_.max_current_m1) {
      if (!motor1_status_.overcurrent) {
        // First trip
        SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(), "Motor 1 overcurrent");
        setEmergencyStop();
        motor1_status_.overcurrent = true;
        total_safety_violations_++;

        // TODO: If this error persists or is massive (short circuit), trigger Power Cycle.
      }
      
      // Rate-limited diagnostics
      if (now - last_estop_msg_time >= config_.estop_msg_interval_ms) {
          char msg[192] = {0};
          snprintf(msg, sizeof(msg),
                  "active:true,source:ROBOCLAW_CURRENT,reason:Motor 1 overcurrent,value:%.2f,manual_reset:true,time:%lu",
                  static_cast<double>(motor1_status_.current_amps), static_cast<unsigned long>(millis()));
          SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", "RoboClaw", msg);
          last_estop_msg_time = now;
      }
    }
  }

  if (motor2_status_.current_valid) {
    if (std::abs(motor2_status_.current_amps) > config_.max_current_m2) {
      if (!motor2_status_.overcurrent) {
        SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(), "Motor 2 overcurrent");
        setEmergencyStop();
        motor2_status_.overcurrent = true;
        total_safety_violations_++;

        // TODO: If this error persists or is massive (short circuit), trigger Power Cycle.
      }
      
      if (now - last_estop_msg_time >= config_.estop_msg_interval_ms) {
          char msg[192] = {0};
          snprintf(msg, sizeof(msg),
                  "active:true,source:ROBOCLAW_CURRENT,reason:Motor 2 overcurrent,value:%.2f,manual_reset:true,time:%lu",
                  static_cast<double>(motor2_status_.current_amps), static_cast<unsigned long>(millis()));
          SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", "RoboClaw", msg);
          last_estop_msg_time = now;
      }
    }
  }

  // Check for runaway detection
  detectMotorRunaway();

  // Attempt Auto-Reset of E-Stop if all conditions are now safe
  if (emergency_stop_active_) {
    // Check all latching safety flags
    // Overcurrent ismanual reset only (require power cycling the RoboClaw).
    // Runaway can be auto-cleared if velocity is zeroed (handled in setVelocityCommand)
    
    // We do NOT auto-reset here anymore to strictly enforce "Manual Reset" policy for safety.
    // The previous auto-reset logic was risky for overcurrent.
    // The host must send 'reset' command which calls resetSafetyFlags().
  }
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

  // Check if motors are running away (moving when commanded to stop)
  if (std::abs(motor1_status_.speed_qpps) > config_.runaway_speed_threshold_qpps) {
    const bool first_trip = !motor1_status_.runaway_detected;
    motor1_status_.runaway_detected = true;
    total_safety_violations_++;

    if (first_trip) {
      SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                     "Motor 1 runaway detected");
      setEmergencyStop();
    }

    // Rate-limited ESTOP message
    if (now - last_runaway_msg_time >= config_.runaway_msg_interval_ms) {
      char msg[160] = {0};
      snprintf(msg, sizeof(msg),
               "active:true,source:MOTOR_RUNAWAY,reason:Motor 1 runaway detected,value:%ld,manual_reset:false,time:%lu",
               static_cast<long>(motor1_status_.speed_qpps), static_cast<unsigned long>(millis()));
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
      last_runaway_msg_time = now;
    }
  }

  if (last_commanded_m2_qpps_ == 0 && std::abs(motor2_status_.speed_qpps) > config_.runaway_speed_threshold_qpps) {
    const bool first_trip = !motor2_status_.runaway_detected;
    motor2_status_.runaway_detected = true;
    total_safety_violations_++;

    if (first_trip) {
      SafetyCoordinator::getInstance().activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                                                     "Motor 2 runaway detected");
      setEmergencyStop();
    }

    // Rate-limited ESTOP message
    if (now - last_runaway_msg_time >= config_.runaway_msg_interval_ms) {
      char msg[160] = {0};
      snprintf(msg, sizeof(msg),
               "active:true,source:MOTOR_RUNAWAY,reason:Motor 2 runaway detected,value:%ld,manual_reset:false,time:%lu",
               static_cast<long>(motor2_status_.speed_qpps), static_cast<unsigned long>(millis()));
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
      last_runaway_msg_time = now;
    }
  }

  last_runaway_check_time_ms_ = now;
}

void RoboClawMonitor::sendStatusReports() {
  // Send ROBOCLAW status message (JSON format)
  char decoded[256] = {0};
  decodeErrorStatus(system_status_.error_status, decoded, sizeof(decoded));

  char status_msg[512] = {0};
  snprintf(status_msg, sizeof(status_msg),
           "{\"LogicVoltage\":%.1f,\"MainVoltage\":%.1f,\"Encoder_Left\":%ld,\"Encoder_Right\":%ld,"
           "\"LeftMotorCurrent\":%.3f,\"RightMotorCurrent\":%.3f,\"LeftMotorSpeed\":%ld,\"RightMotorSpeed\":%ld,"
           "\"Error\":\"%lX\",\"ErrorDecoded\":\"%s\"}",
           static_cast<double>(system_status_.logic_battery_voltage), static_cast<double>(system_status_.main_battery_voltage),
           static_cast<long>(motor1_status_.encoder_count), static_cast<long>(motor2_status_.encoder_count),
           static_cast<double>(motor1_status_.current_amps), static_cast<double>(motor2_status_.current_amps),
           static_cast<long>(motor1_status_.speed_qpps), static_cast<long>(motor2_status_.speed_qpps),
           static_cast<unsigned long>(system_status_.error_status), decoded);

  SerialManager::getInstance().sendMessage("ROBOCLAW", status_msg);

  // Auto-recovery for latched overcurrent errors when current is actually low
  if (system_status_.error_status != 0) {
    bool m1_overcurrent = (system_status_.error_status & static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT_WARNING));
    bool m2_overcurrent = (system_status_.error_status & static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT_WARNING));

    // If overcurrent error is flagged but actual current is low, attempt
    // auto-recovery
    if ((m1_overcurrent && motor1_status_.current_valid &&
       std::abs(motor1_status_.current_amps) < config_.overcurrent_recovery_current_amps) ||
      (m2_overcurrent && motor2_status_.current_valid &&
       std::abs(motor2_status_.current_amps) < config_.overcurrent_recovery_current_amps)) {
      static uint32_t last_auto_recovery = 0;
      uint32_t now = millis();

      // Only attempt auto-recovery every 30 seconds to avoid spam
      if (now - last_auto_recovery > 30000) {
        SerialManager::getInstance().sendDiagnosticMessage("INFO", name(),
                                                           "Attempting auto-recovery for latched overcurrent error");
        resetErrors();
        last_auto_recovery = now;
      }
    }
  }
}

void RoboClawMonitor::sendDiagnosticReports() {
  // Send diagnostic information
  char diag_msg[256] = {0};
  snprintf(diag_msg, sizeof(diag_msg),
           "Diagnostic report,details:commands:%lu,errors:%lu,safety_violations:%lu,connection_state:%d,emergency_stop:%s",
           static_cast<unsigned long>(total_commands_sent_), static_cast<unsigned long>(total_communication_errors_),
           static_cast<unsigned long>(total_safety_violations_), static_cast<int>(connection_state_),
           emergency_stop_active_ ? "true" : "false");
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), diag_msg);
}

void RoboClawMonitor::decodeErrorStatus(uint32_t error_status, char* out, size_t out_len) const {
  if (!out || out_len == 0) {
    return;
  }
  out[0] = '\0';

  if (error_status == 0) {
    snprintf(out, out_len, "%s", "No errors");
    return;
  }

  bool first = true;
  auto append = [&](const char* token) {
    if (!token) {
      return;
    }
    const size_t len = strlen(out);
    if (len >= (out_len - 1)) {
      return;
    }
    if (!first) {
      snprintf(out + len, out_len - len, ", %s", token);
    } else {
      snprintf(out + len, out_len - len, "%s", token);
      first = false;
    }
  };

  // Check each error bit and add description
  if (error_status & static_cast<uint32_t>(RoboClawError::E_STOP)) append("E_STOP");

  if (error_status & static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR)) append("TEMPERATURE_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::TEMPERATURE2_ERROR)) append("TEMPERATURE2_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH_ERROR)) append("MAIN_BATTERY_HIGH_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_HIGH_ERROR)) append("LOGIC_VOLTAGE_HIGH_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_LOW_ERROR)) append("LOGIC_VOLTAGE_LOW_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR)) append("M1_DRIVER_FAULT_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR)) append("M2_DRIVER_FAULT_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M1_SPEED_ERROR)) append("M1_SPEED_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M2_SPEED_ERROR)) append("M2_SPEED_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M1_POSITION_ERROR)) append("M1_POSITION_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M2_POSITION_ERROR)) append("M2_POSITION_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M1_CURRENT_ERROR)) append("M1_CURRENT_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M2_CURRENT_ERROR)) append("M2_CURRENT_ERROR");

  if (error_status & static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT_WARNING)) append("M1_OVERCURRENT_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT_WARNING)) append("M2_OVERCURRENT_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::MAIN_VOLTAGE_HIGH_WARNING)) append("MAIN_VOLTAGE_HIGH_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::MAIN_VOLTAGE_LOW_WARNING)) append("MAIN_VOLTAGE_LOW_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::TEMPERATURE_WARNING)) append("TEMPERATURE_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::TEMPERATURE_2_WARNING)) append("TEMPERATURE_2_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::S4_SIGNAL_TRIGGERED)) append("S4_SIGNAL_TRIGGERED");

  if (error_status & static_cast<uint32_t>(RoboClawError::S5_SIGNAL_TRIGGERED)) append("S5_SIGNAL_TRIGGERED");

  if (error_status & static_cast<uint32_t>(RoboClawError::SPEED_ERROR_LIMIT_WARNING)) append("SPEED_ERROR_LIMIT_WARNING");

  if (error_status & static_cast<uint32_t>(RoboClawError::POSITION_ERROR_LIMIT_WARNING)) append("POSITION_ERROR_LIMIT_WARNING");

  // Check for unknown error bits
  uint32_t known_errors =
      static_cast<uint32_t>(RoboClawError::E_STOP) | static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR) |
      static_cast<uint32_t>(RoboClawError::TEMPERATURE2_ERROR) |
      static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH_ERROR) |
      static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_HIGH_ERROR) |
      static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_LOW_ERROR) |
      static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR) |
      static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR) |
      static_cast<uint32_t>(RoboClawError::M1_SPEED_ERROR) | static_cast<uint32_t>(RoboClawError::M2_SPEED_ERROR) |
      static_cast<uint32_t>(RoboClawError::M1_POSITION_ERROR) |
      static_cast<uint32_t>(RoboClawError::M2_POSITION_ERROR) | static_cast<uint32_t>(RoboClawError::M1_CURRENT_ERROR) |
      static_cast<uint32_t>(RoboClawError::M2_CURRENT_ERROR) |
      static_cast<uint32_t>(RoboClawError::M1_OVERCURRENT_WARNING) |
      static_cast<uint32_t>(RoboClawError::M2_OVERCURRENT_WARNING) |
      static_cast<uint32_t>(RoboClawError::MAIN_VOLTAGE_HIGH_WARNING) |
      static_cast<uint32_t>(RoboClawError::MAIN_VOLTAGE_LOW_WARNING) |
      static_cast<uint32_t>(RoboClawError::TEMPERATURE_WARNING) |
      static_cast<uint32_t>(RoboClawError::TEMPERATURE_2_WARNING) |
      static_cast<uint32_t>(RoboClawError::S4_SIGNAL_TRIGGERED) |
      static_cast<uint32_t>(RoboClawError::S5_SIGNAL_TRIGGERED) |
      static_cast<uint32_t>(RoboClawError::SPEED_ERROR_LIMIT_WARNING) |
      static_cast<uint32_t>(RoboClawError::POSITION_ERROR_LIMIT_WARNING);

  uint32_t unknown_errors = error_status & ~known_errors;
  if (unknown_errors != 0) {
    char token[32] = {0};
    snprintf(token, sizeof(token), "UNKNOWN_ERROR_BITS:0x%lX", static_cast<unsigned long>(unknown_errors));
    append(token);
  }
}

}  // namespace sigyn_teensy
