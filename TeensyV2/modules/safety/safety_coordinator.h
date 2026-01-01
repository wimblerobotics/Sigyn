// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file safety_coordinator.h
 * @brief Central safety coordination and E-stop management for TeensyV2
 *
 * Coordinates safety across all system modules and manages E-stop conditions
 * from multiple sources. Provides centralized safety state management with
 * inter-board coordination and graceful recovery mechanisms.
 *
 * E-stop Sources:
 * - Hardware E-stop button (immediate hardware cutoff)
 * - Software E-stop from ROS2 system
 * - Performance violations (timing, frequency)
 * - Battery critical conditions (low voltage, high current)
 * - Motor fault conditions (overcurrent, runaway)
 * - Sensor failures (critical sensors offline)
 * - Inter-board safety signals
 *
 * Recovery Logic:
 * - Automatic recovery when transient conditions clear
 * - Manual recovery required for persistent conditions
 * - Graceful degradation for non-critical issues
 * - Full system shutdown for emergency conditions
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <Arduino.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

/**
 * @brief E-stop sources enumeration for tracking and reporting.
 */
typedef enum class FaultSource {
  HARDWARE_BUTTON,       ///< Physical E-stop button pressed
  SOFTWARE_COMMAND,      ///< Software E-stop from ROS2
  PERFORMANCE,           ///< Performance violation (timing/frequency)
  BATTERY_LOW_VOLTAGE,   ///< Battery voltage critically low
  BATTERY_HIGH_CURRENT,  ///< Battery current critically high
  MOTOR_OVERCURRENT,     ///< Motor drawing excessive current
  MOTOR_RUNAWAY,         ///< Motor speed control failure
  SENSOR_FAILURE,        ///< Critical sensor offline
  INTER_BOARD_BOARD2,    ///< Safety signal from Board 2
  INTER_BOARD_BOARD3,    ///< Safety signal from Board 3
  UNKNOWN,               ///< Undefined or multiple sources
  NUMBER_FAULT_SOURCES
} FaultSource;

/**
 * @brief Safety state enumeration for system status.
 */
typedef enum class FaultSeverity {
  NORMAL,          ///< All systems operational
  WARNING,         ///< Minor issues detected, monitoring
  DEGRADED,        ///< Operating with reduced functionality
  EMERGENCY_STOP,  ///< Emergency stop active
  SYSTEM_SHUTDOWN  ///< Complete system shutdown required
} FaultSeverity;

struct Fault {
  bool active;             ///< Is the fault currently active
  FaultSource source;      ///< Source of the fault
  FaultSeverity severity;  ///< Severity level of the fault
  String description;      ///< Human-readable description
  uint32_t timestamp;      ///< Time when fault was detected
};

/**
 * @brief Central safety coordinator for TeensyV2 system.
 *
 * Manages E-stop conditions from multiple sources, coordinates safety
 * state across all modules, and provides graceful recovery mechanisms.
 *
 * Key responsibilities:
 * - Monitor hardware and software E-stop signals
 * - Aggregate safety status from all registered modules
 * - Trigger system-wide E-stop when safety is compromised
 * - Coordinate safety state with other Teensy boards
 * - Manage graceful recovery from transient faults
 * - Provide detailed diagnostic information for safety events
 *
 * Example usage:
 * @code
 *   SafetyCoordinator& safety = SafetyCoordinator::GetInstance();
 *   if (safety.IsEstopActive()) {
 *     // Halt motors and critical systems
 *   }
 * @endcode
 */
class SafetyCoordinator : public Module {
 public:
  // --- Singleton Access ---
  /**
   * @brief Get singleton instance of SafetyCoordinator.
   * @return Reference to singleton SafetyCoordinator
   */
  static SafetyCoordinator& getInstance();

  // // --- Public API ---
  /**
   * @brief Activate the E-stop state.
   * @param severity The severity level of the fault
   * @param source The source of the E-stop
   * @param description A description of the reason
   */
  void activateFault(FaultSeverity severity, FaultSource source, const String& description);

  /**
   * @brief Deactivate the E-stop state.
   */
  void deactivateFault(FaultSource source);

  void setEstopCommand(String command);

  // /**
  //  * @brief Check all safety conditions and update system state.
  //  *
  //  * This method is called automatically by the main loop and should not be
  //  * called directly by other modules.
  //  */
  // void checkSafetyStatus();

  // /**
  //  * @brief Attempt to recover from an E-stop condition.
  //  *
  //  * This method will only succeed if all underlying safety conditions have
  //  * been cleared.
  //  */
  // void attemptRecovery();

  /**
   * @brief Get the current safety state of the system.
   * @return The current safety state
   */
  // FaultSeverity getSafetyState() const;

  // /**
  //  * @brief Get the current E-stop condition.
  //  * @return Const reference to the current E-stop condition
  //  */
  // const EstopCondition& getEstopCondition() const;

  // --- Module Overrides ---
  /**
   * @brief Get the name of this module.
   * @return "SafetyCoordinator"
   */
  const char* name() const override;

  /**
   * @brief Check if the system is in an unsafe state.
   * @return True if an E-stop condition is active, false otherwise
   */
  bool isUnsafe() override;

  /**
   * @brief Reset all safety flags and attempt recovery.
   */
  void resetSafetyFlags() override;

 protected:
  // --- Module Overrides ---
  /**
   * @brief Initialize safety coordinator module.
   *
   * This method is called once at startup to initialize the module.
   */
  void setup() override;

  /**
   * @brief Main loop for safety coordination.
   *
   * This method is called automatically by the main loop. It checks all
   * safety conditions and manages the system's safety state.
   */
  void loop() override;

 private:
  // --- Singleton Implementation ---
  /**
   * @brief Private constructor for singleton pattern.
   */
  SafetyCoordinator();
  SafetyCoordinator(const SafetyCoordinator&) = delete;
  SafetyCoordinator& operator=(const SafetyCoordinator&) = delete;

  // Relays
  void setRoboClawPower(bool on);
  void setMainBatteryPower(bool on);

  // --- Private Methods ---
  // /**
  //  * @brief Check for hardware E-stop button press.
  //  */
  // void checkHardwareEstop();

  /**
   * @brief Check for safety signals from other boards.
   */
  // void checkInterBoardSafety();

  // /**
  //  * @brief Check for safety violations from other modules.
  //  */
  // void checkModuleSafety();

  /**
   * @brief Send a status update via SerialManager.
   */
  void sendStatusUpdate();

  // --- Member Variables ---
  uint8_t active_estop_count_ = 0;
  Fault faults_[static_cast<size_t>(FaultSource::NUMBER_FAULT_SOURCES)];  ///< Active faults
};

static const char* faultSourceToString(FaultSource source) {
  switch (source) {
    case FaultSource::HARDWARE_BUTTON:
      return "HARDWARE_BUTTON";
    case FaultSource::SOFTWARE_COMMAND:
      return "SOFTWARE_COMMAND";
    case FaultSource::PERFORMANCE:
      return "PERFORMANCE";
    case FaultSource::BATTERY_LOW_VOLTAGE:
      return "BATTERY_LOW_VOLTAGE";
    case FaultSource::BATTERY_HIGH_CURRENT:
      return "BATTERY_HIGH_CURRENT";
    case FaultSource::MOTOR_OVERCURRENT:
      return "MOTOR_OVERCURRENT";
    case FaultSource::MOTOR_RUNAWAY:
      return "MOTOR_RUNAWAY";
    case FaultSource::SENSOR_FAILURE:
      return "SENSOR_FAILURE";
    case FaultSource::INTER_BOARD_BOARD2:
      return "INTER_BOARD_BOARD2";
    case FaultSource::INTER_BOARD_BOARD3:
      return "INTER_BOARD_BOARD3";
    case FaultSource::UNKNOWN:
    default:
      return "UNKNOWN";
  }
}

static const char* faultSeverityToString(FaultSeverity severity) {
  switch (severity) {
    case FaultSeverity::NORMAL:
      return "NORMAL";
    case FaultSeverity::WARNING:
      return "WARNING";
    case FaultSeverity::DEGRADED:
      return "DEGRADED";
    case FaultSeverity::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case FaultSeverity::SYSTEM_SHUTDOWN:
      return "SYSTEM_SHUTDOWN";
    default:
      return "UNKNOWN";
  }
}

}  // namespace sigyn_teensy
