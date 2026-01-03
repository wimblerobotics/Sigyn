// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file roboclaw_monitor.h
 * @brief RoboClaw motor controller monitoring for TeensyV2 system
 *
 * Provides real-time monitoring and control of RoboClaw motor controller with
 * integrated safety features, performance monitoring, and diagnostic reporting.
 * This module interfaces with the RoboClaw hardware driver to provide high-level
 * motor control capabilities and safety monitoring.
 *
 * Features:
 * - Motor velocity and position control
 * - Real-time current and voltage monitoring
 * - Motor runaway detection and prevention
 * - Temperature monitoring (if supported by RoboClaw model)
 * - Safety integration with automatic E-stop capabilities
 * - Comprehensive error reporting and recovery
 *
 * Safety Features:
 * - Motor runaway detection based on encoder feedback
 * - Current overconsumption protection
 * - Communication timeout protection
 * - Automatic emergency stop coordination
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#ifdef UNIT_TEST
#include "Arduino.h"  // Mock Arduino for testing
#else
#include <Arduino.h>  // Real Arduino SDK
#endif

#include <cmath>
#include <cstdint>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"
#include "interfaces/i_roboclaw.h"

#ifndef UNIT_TEST
#include "RoboClaw.h"
#include "roboclaw_adapter.h"
#endif

namespace sigyn_teensy {

/**
 * @brief Configuration parameters for RoboClaw monitoring and control.
 */
struct RoboClawConfig {
  // Communication settings
  uint8_t address = 0x80;        ///< RoboClaw device address
  uint32_t timeout_us = 100000;  ///< Communication timeout (microseconds)
  uint32_t baud_rate = 230400;   ///< Serial communication baud rate
  uint8_t max_consecutive_comm_failures = 3;  ///< Escalate to E-stop after this many failed read cycles

  // Robot kinematics / encoder model
  float wheel_diameter_m = 0.102224144529039f;
  float wheel_base_m = 0.3906f;
  uint32_t quadrature_pulses_per_revolution = 1000;

  // Safety thresholds
  float max_current_m1 = 100.0f;    ///< Maximum current for motor 1 (Amps)
  float max_current_m2 = 100.0f;    ///< Maximum current for motor 2 (Amps)
  float warning_current = 10.0f;    ///< Warning current threshold (Amps)
  uint32_t max_speed_qpps = 10000;  ///< Maximum speed (quad pulses per second)

  // RoboClaw internal temperature thresholds (based on manual: warning around 85C, fault around 100C)
  float roboclaw_temp_warning_c = 85.0f;
  float roboclaw_temp_fault_c = 100.0f;

  // Command + control tuning
  uint32_t cmd_vel_timeout_ms = 200;              ///< Stop motors if cmd_vel is stale
  uint32_t command_rate_limit_ms = 15;            ///< Max send rate to controller
  int32_t significant_change_qpps = 10;           ///< Change threshold to resend
  uint32_t force_update_ms = 100;                 ///< Periodic resend even without change
  float max_seconds_commanded_travel_s = 0.05f;   ///< Distance = |qpps| * this

  // Odometry tuning
  float odom_min_dt_s = 0.010f;                   ///< Ignore tiny dt to reduce noise
  float odom_max_dt_s = 0.100f;                   ///< Reset timing if dt too large
  float odom_periodic_send_dt_s = 0.033f;         ///< Send at least this often when stationary
  int32_t odom_movement_threshold_ticks = 2;       ///< Consider moving if delta ticks exceeds

  // Safety tuning
  int32_t runaway_speed_threshold_qpps = 100;      ///< Runaway if commanded 0 but speed exceeds
  float overcurrent_recovery_current_amps = 0.5f;  ///< Auto-recover if latched but actual low
  uint32_t estop_msg_interval_ms = 1000;           ///< Rate-limit estop messages
  uint32_t runaway_msg_interval_ms = 1000;         ///< Rate-limit runaway messages

  // Runaway detection
  uint32_t runaway_check_interval_ms = 200;   ///< Runaway detection check interval (reduced for performance)
  uint32_t runaway_encoder_threshold = 1000;  ///< Encoder change threshold for runaway
  uint32_t command_timeout_ms = 1000;         ///< Command timeout for runaway detection

  // Motor control parameters
  uint32_t default_acceleration = 2000;  ///< Default acceleration (QPPS²)
  uint32_t max_distance = 1000000;       ///< Maximum distance for move commands

  // Reporting intervals
  uint32_t status_report_interval_ms = 200;       ///< Status reporting interval
  uint32_t diagnostic_report_interval_ms = 1000;  ///< Diagnostic reporting interval
};

/**
 * @brief Motor status and diagnostic information.
 */
struct MotorStatus {
  // Current readings
  int32_t encoder_count = 0;  ///< Current encoder count
  int32_t speed_qpps = 0;     ///< Current speed (quad pulses per second)
  float current_amps = 0.0f;  ///< Current draw (Amps)

  // Status flags
  bool communication_ok = true;  ///< Communication with RoboClaw OK
  bool encoder_valid = true;     ///< Encoder reading valid
  bool speed_valid = true;       ///< Speed reading valid
  bool current_valid = true;     ///< Current reading valid

  // Safety flags
  bool runaway_detected = false;  ///< Motor runaway detected
  bool overcurrent = false;       ///< Overcurrent condition
  bool timeout_error = false;     ///< Communication timeout
};

/**
 * @brief RoboClaw system status information.
 */
typedef struct RoboClawStatus {
  // System voltages
  float main_battery_voltage = 0.0f;   ///< Main battery voltage (V)
  float logic_battery_voltage = 0.0f;  ///< Logic battery voltage (V)

  // Temperature (if available)
  float temperature_c = NAN;  ///< System temperature (°C)

  // Error status
  uint32_t error_status = 0;  ///< RoboClaw error status register

  // Timing information
  uint32_t last_command_time_ms = 0;   ///< Time of last command
  uint32_t last_status_update_ms = 0;  ///< Time of last status update

  // Communication statistics
  uint32_t command_count = 0;  ///< Total commands sent
  uint32_t error_count = 0;    ///< Total communication errors
} RoboClawStatus;

/**
 * @brief Velocity command structure for motor control.
 */
struct VelocityCommand {
  float linear_x = 0.0f;      ///< Linear velocity (m/s)
  float angular_z = 0.0f;     ///< Angular velocity (rad/s)
  uint32_t timestamp_ms = 0;  ///< Command timestamp
};

/**
 * @brief RoboClaw motor controller monitor and control module.
 *
 * This module provides comprehensive monitoring and control of the RoboClaw
 * motor controller, including safety features, performance monitoring, and
 * diagnostic reporting. It follows the TeensyV2 module architecture with
 * real-time performance requirements and safety integration.
 */
class RoboClawMonitor : public Module {
 public:
  static RoboClawMonitor& getInstance();

#ifdef UNIT_TEST
  // Allow unit tests to inject a controllable RoboClaw implementation.
  void setRoboClawForTesting(IRoboClaw* roboclaw) { roboclaw_ = roboclaw; }

  // Allow unit tests to bypass hardware init and enable command sending.
  void setConnectedForTesting(bool connected) {
    connection_state_ = connected ? ConnectionState::CONNECTED : ConnectionState::DISCONNECTED;
  }

  // Test helpers to exercise safety logic without hardware.
  void setMotorCurrentsForTesting(float m1_amps, float m2_amps, bool valid = true) {
    motor1_status_.current_amps = m1_amps;
    motor2_status_.current_amps = m2_amps;
    motor1_status_.current_valid = valid;
    motor2_status_.current_valid = valid;
  }

  void setMotorSpeedFeedbackForTesting(int32_t m1_qpps, int32_t m2_qpps, bool valid = true) {
    motor1_status_.speed_qpps = m1_qpps;
    motor2_status_.speed_qpps = m2_qpps;
    motor1_status_.speed_valid = valid;
    motor2_status_.speed_valid = valid;
  }

  void setRunawayDetectionInitializedForTesting(bool initialized) { runaway_detection_initialized_ = initialized; }
  void setLastCommandedQppsForTesting(int32_t m1_qpps, int32_t m2_qpps) {
    last_commanded_m1_qpps_ = m1_qpps;
    last_commanded_m2_qpps_ = m2_qpps;
  }
  void setSystemErrorStatusForTesting(uint32_t error_status) { system_status_.error_status = error_status; }
  void setRoboClawTemperatureForTesting(float temp_c) { system_status_.temperature_c = temp_c; }
  bool isEmergencyStopActiveForTesting() const { return emergency_stop_active_; }

  void runSafetyChecksForTesting() { checkSafetyConditions(); }
  bool testCommunicationForTesting() { return testCommunication(); }

  // Allows tests to drive the read-failure retry logic.
  void updateMotorStatusForTesting() { updateMotorStatus(); }
#endif

  // E-STOP interface.
  void setEmergencyStop();
  void clearEmergencyStop();

  // Motor control interface
  void setVelocityCommand(float linear_x, float angular_z);
  void setMotorSpeeds(int32_t m1_qpps, int32_t m2_qpps);
  void resetErrors();

  // Status access
  const MotorStatus& getMotor1Status() const { return motor1_status_; }
  const MotorStatus& getMotor2Status() const { return motor2_status_; }
  const RoboClawStatus& getSystemStatus() const { return system_status_; }

  // Configuration
  void updateConfig(const RoboClawConfig& config) { config_ = config; }
  const RoboClawConfig& getConfig() const { return config_; }

  // Message handling
  void handleTwistMessage(const String& data);

  // Error decoding
  String decodeErrorStatus(uint32_t error_status) const;

 protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() const override { return "RoboClawMonitor"; }

  // Safety interface
  bool isUnsafe() override;
  void resetSafetyFlags() override;

 private:
  // Singleton constructor
  RoboClawMonitor();

#ifdef UNIT_TEST
  class NullRoboClaw final : public IRoboClaw {
   public:
    void begin(long) override {}
    bool ResetEncoders(uint8_t) override { return false; }
    uint32_t ReadError(uint8_t, bool* valid = nullptr) override {
      if (valid) *valid = false;
      return 0;
    }
    bool SetM1MaxCurrent(uint8_t, uint32_t) override { return false; }
    bool SetM2MaxCurrent(uint8_t, uint32_t) override { return false; }
    bool ReadM1MaxCurrent(uint8_t, uint32_t& max) override {
      max = 0;
      return false;
    }
    bool ReadM2MaxCurrent(uint8_t, uint32_t& max) override {
      max = 0;
      return false;
    }
    bool SetM1VelocityPID(uint8_t, float, float, float, uint32_t) override { return false; }
    bool SetM2VelocityPID(uint8_t, float, float, float, uint32_t) override { return false; }
    uint32_t ReadEncM1(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
      if (status) *status = 0;
      if (valid) *valid = false;
      return 0;
    }
    uint32_t ReadEncM2(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
      if (status) *status = 0;
      if (valid) *valid = false;
      return 0;
    }
    uint32_t ReadSpeedM1(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
      if (status) *status = 0;
      if (valid) *valid = false;
      return 0;
    }
    uint32_t ReadSpeedM2(uint8_t, uint8_t* status = nullptr, bool* valid = nullptr) override {
      if (status) *status = 0;
      if (valid) *valid = false;
      return 0;
    }
    bool ReadCurrents(uint8_t, int16_t& current1, int16_t& current2) override {
      current1 = 0;
      current2 = 0;
      return false;
    }
    uint16_t ReadMainBatteryVoltage(uint8_t, bool* valid = nullptr) override {
      if (valid) *valid = false;
      return 0;
    }
    uint16_t ReadLogicBatteryVoltage(uint8_t, bool* valid = nullptr) override {
      if (valid) *valid = false;
      return 0;
    }
    bool ReadTemp(uint8_t, uint16_t& temp) override {
      temp = 0;
      return false;
    }
    bool ReadVersion(uint8_t, char*) override { return false; }
    bool SpeedAccelDistanceM1M2(uint8_t, uint32_t, int32_t, uint32_t, int32_t, uint32_t, uint8_t) override {
      return false;
    }
  };
#endif

  // State machine for connection management
  enum class ConnectionState { DISCONNECTED, CONNECTING, CONNECTED, ERROR_RECOVERY, FAILED };

  enum class ReadingState {
    READ_ENCODER_M1,
    READ_SPEED_M1,
    READ_ENCODER_M2,
    READ_SPEED_M2,
    READ_CURRENTS,
    READ_VOLTAGES,
    READ_ERROR_STATUS,
    COMPLETE
  };

  // Core functionality
  void updateMotorStatus();
  void updateCriticalMotorStatus();  // High-frequency: encoder/speed only
  void updateOdometry();             // High-frequency: odometry calculation
  void processVelocityCommands();    // High-frequency: cmd_vel processing
  void updateSystemStatus();
  void checkSafetyConditions();
  void handleRunawayDetection();
  void sendStatusReports();
  void sendDiagnosticReports();

  // RoboClaw communication
  bool initializeRoboClaw();
  bool testCommunication();
  void handleCommunicationError();

  // Motor control helpers
  void velocityToMotorSpeeds(float linear_x, float angular_z, int32_t& m1_qpps, int32_t& m2_qpps);
  void executeMotorCommand(int32_t m1_qpps, int32_t m2_qpps);

  // Safety and error handling
  void detectMotorRunaway();
  void handleOvercurrent();
  void handleCommunicationTimeout();

  // Configuration and constants
  RoboClawConfig config_;

  // Hardware interface
#ifndef UNIT_TEST
  RoboClaw roboclaw_hw_;
  RoboClawAdapter roboclaw_adapter_;
#else
  NullRoboClaw roboclaw_null_;
#endif
  IRoboClaw* roboclaw_;
  ConnectionState connection_state_;
  ReadingState reading_state_;
  uint32_t last_reading_time_ms_;

  // Status tracking
  MotorStatus motor1_status_;
  MotorStatus motor2_status_;
  RoboClawStatus system_status_;

  // Command tracking
  VelocityCommand last_velocity_command_;
  int32_t last_commanded_m1_qpps_;
  int32_t last_commanded_m2_qpps_;
  uint32_t last_command_time_ms_;

  // Safety state
  bool emergency_stop_active_;
  bool runaway_detection_initialized_;
  uint32_t last_runaway_check_time_ms_;
  int32_t last_runaway_encoder_m1_;
  int32_t last_runaway_encoder_m2_;

  // Odometry state for high-frequency updates
  struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
  };

  struct Velocity2D {
    float linear_x = 0.0f;
    float angular_z = 0.0f;
  };

  Pose2D current_pose_;
  Velocity2D current_velocity_;
  int32_t prev_encoder_m1_;
  int32_t prev_encoder_m2_;
  uint32_t last_odom_update_time_us_;
  bool odometry_initialized_;

  // Timing for periodic operations
  uint32_t last_status_report_time_ms_;
  uint32_t last_diagnostic_report_time_ms_;
  uint32_t last_safety_check_time_ms_;

  // Performance statistics
  uint32_t total_commands_sent_;
  uint32_t total_communication_errors_;
  uint32_t total_safety_violations_;

  // Tracks consecutive read cycles where *no* readings were valid.
  uint8_t consecutive_comm_failures_ = 0;
};

// RoboClaw error status bit definitions
enum class RoboClawError : uint32_t {
  E_STOP = 0x0000'0001,
  TEMPERATURE_ERROR = 0x0000'0002,
  TEMPERATURE2_ERROR = 0x0000'0004,
  MAIN_BATTERY_HIGH_ERROR = 0x0000'0008,
  LOGIC_VOLTAGE_HIGH_ERROR = 0x0000'0010,
  LOGIC_VOLTAGE_LOW_ERROR = 0x0000'0020,
  M1_DRIVER_FAULT_ERROR = 0x0000'0040,
  M2_DRIVER_FAULT_ERROR = 0x0000'0080,
  M1_SPEED_ERROR = 0x0000'0100,
  M2_SPEED_ERROR = 0x0000'0200,
  M1_POSITION_ERROR = 0x0000'0400,
  M2_POSITION_ERROR = 0x0000'0800,
  M1_CURRENT_ERROR = 0x0000'1000,
  M2_CURRENT_ERROR = 0x0000'2000,
  M1_OVERCURRENT_WARNING = 0x0001'0000,
  M2_OVERCURRENT_WARNING = 0x0002'0000,
  MAIN_VOLTAGE_HIGH_WARNING = 0x0004'0000,
  MAIN_VOLTAGE_LOW_WARNING = 0x0008'0000,
  TEMPERATURE_WARNING = 0x0010'0000,
  TEMPERATURE_2_WARNING = 0x0020'0000,
  S4_SIGNAL_TRIGGERED = 0x0040'0000,
  S5_SIGNAL_TRIGGERED = 0x0080'0000,
  SPEED_ERROR_LIMIT_WARNING = 0x0100'0000,
  POSITION_ERROR_LIMIT_WARNING = 0x0200'0000,
};

}  // namespace sigyn_teensy
