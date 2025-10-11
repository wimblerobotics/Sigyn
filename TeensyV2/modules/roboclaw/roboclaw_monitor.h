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

#include <Arduino.h>
#include <cstdint>
#include <cmath>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"
#include "RoboClaw.h"

namespace sigyn_teensy {

/**
 * @brief Configuration parameters for RoboClaw monitoring and control.
 */
struct RoboClawConfig {
  // Communication settings
  uint8_t address = 0x80;              ///< RoboClaw device address
  uint32_t timeout_us = 100000;        ///< Communication timeout (microseconds)
  uint32_t baud_rate = 230400;         ///< Serial communication baud rate
  
  // Safety thresholds
  float max_current_m1 = 100.0f;        ///< Maximum current for motor 1 (Amps)
  float max_current_m2 = 100.0f;        ///< Maximum current for motor 2 (Amps)
  float warning_current = 10.0f;       ///< Warning current threshold (Amps)
  uint32_t max_speed_qpps = 10000;     ///< Maximum speed (quad pulses per second)
  
  // Runaway detection
  uint32_t runaway_check_interval_ms = 200;    ///< Runaway detection check interval (reduced for performance)
  uint32_t runaway_encoder_threshold = 1000;   ///< Encoder change threshold for runaway
  uint32_t command_timeout_ms = 1000;          ///< Command timeout for runaway detection
  
  // Motor control parameters
  uint32_t default_acceleration = 2000;        ///< Default acceleration (QPPS²)
  uint32_t max_distance = 1000000;             ///< Maximum distance for move commands
  
  // Reporting intervals
  uint32_t status_report_interval_ms = 200;    ///< Status reporting interval
  uint32_t diagnostic_report_interval_ms = 1000; ///< Diagnostic reporting interval
};

/**
 * @brief Motor status and diagnostic information.
 */
struct MotorStatus {
  // Current readings
  int32_t encoder_count = 0;           ///< Current encoder count
  int32_t speed_qpps = 0;              ///< Current speed (quad pulses per second)
  float current_amps = 0.0f;           ///< Current draw (Amps)
  
  // Status flags
  bool communication_ok = false;       ///< Communication with RoboClaw OK
  bool encoder_valid = false;          ///< Encoder reading valid
  bool speed_valid = false;            ///< Speed reading valid
  bool current_valid = false;          ///< Current reading valid
  
  // Safety flags
  bool runaway_detected = false;       ///< Motor runaway detected
  bool overcurrent = false;            ///< Overcurrent condition
  bool timeout_error = false;          ///< Communication timeout
};

/**
 * @brief RoboClaw system status information.
 */
struct RoboClawStatus {
  // System voltages
  float main_battery_voltage = 0.0f;   ///< Main battery voltage (V)
  float logic_battery_voltage = 0.0f; ///< Logic battery voltage (V)
  
  // Temperature (if available)
  float temperature_c = NAN;           ///< System temperature (°C)
  
  // Error status
  uint32_t error_status = 0;           ///< RoboClaw error status register
  
  // Timing information
  uint32_t last_command_time_ms = 0;   ///< Time of last command
  uint32_t last_status_update_ms = 0;  ///< Time of last status update
  
  // Communication statistics
  uint32_t command_count = 0;          ///< Total commands sent
  uint32_t error_count = 0;            ///< Total communication errors
};

/**
 * @brief Velocity command structure for motor control.
 */
struct VelocityCommand {
  float linear_x = 0.0f;               ///< Linear velocity (m/s)
  float angular_z = 0.0f;              ///< Angular velocity (rad/s)
  uint32_t timestamp_ms = 0;           ///< Command timestamp
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
  
  // Motor control interface
  void setVelocityCommand(float linear_x, float angular_z);
  void setMotorSpeeds(int32_t m1_qpps, int32_t m2_qpps);
  void emergencyStop();
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
  
  // State machine for connection management
  enum class ConnectionState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR_RECOVERY
  };
  
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
  void updateCriticalMotorStatus();    // High-frequency: encoder/speed only
  void updateOdometry();               // High-frequency: odometry calculation
  void processVelocityCommands();      // High-frequency: cmd_vel processing
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
  void velocityToMotorSpeeds(float linear_x, float angular_z, 
                            int32_t& m1_qpps, int32_t& m2_qpps);
  void executeMotorCommand(int32_t m1_qpps, int32_t m2_qpps);
  
  // Safety and error handling
  void detectMotorRunaway();
  void handleOvercurrent();
  void handleCommunicationTimeout();
  
  // Configuration and constants
  RoboClawConfig config_;
  
  // Hardware interface
  RoboClaw roboclaw_;
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
};

// RoboClaw error status bit definitions
enum class RoboClawError : uint32_t {
  M1_OVERCURRENT = 0x000001,
  M2_OVERCURRENT = 0x000002,
  E_STOP = 0x000004,
  TEMPERATURE_ERROR = 0x000008,
  TEMPERATURE2_ERROR = 0x000010,
  MAIN_BATTERY_HIGH = 0x000020,
  LOGIC_BATTERY_HIGH = 0x000040,
  LOGIC_BATTERY_LOW = 0x000080,
  M1_DRIVER_FAULT = 0x000100,
  M2_DRIVER_FAULT = 0x000200,
  MAIN_BATTERY_LOW = 0x000400,
  M1_SPEED_ERROR = 0x000800,
  M2_SPEED_ERROR = 0x001000,
  M1_POSITION_ERROR = 0x002000,
  M2_POSITION_ERROR = 0x004000,
  M1_CURRENT_ERROR = 0x008000,
  M2_CURRENT_ERROR = 0x010000,
  LOGIC_VOLTAGE_FAULT = 0x100000,
  COMM_ERROR = 0x01000000,
  M1_HOME = 0x10000000,
  M2_HOME = 0x20000000,
  S3_MODE = 0x40000000,
  S4_MODE = 0x80000000
};

} // namespace sigyn_teensy
