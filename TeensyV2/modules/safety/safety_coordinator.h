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
#include <cstdint>
#include <cstddef>
#include <cmath>
#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

/**
 * @brief E-stop sources enumeration for tracking and reporting.
 */
enum class EstopSource {
  HARDWARE_BUTTON,     ///< Physical E-stop button pressed
  SOFTWARE_COMMAND,    ///< Software E-stop from ROS2
  PERFORMANCE,         ///< Performance violation (timing/frequency)
  BATTERY_LOW_VOLTAGE, ///< Battery voltage critically low
  BATTERY_HIGH_CURRENT,///< Battery current critically high
  MOTOR_OVERCURRENT,   ///< Motor drawing excessive current
  MOTOR_RUNAWAY,       ///< Motor speed control failure
  SENSOR_FAILURE,      ///< Critical sensor offline
  INTER_BOARD,         ///< Safety signal from other board
  UNKNOWN              ///< Undefined or multiple sources
};

/**
 * @brief Safety state enumeration for system status.
 */
enum class SafetyState {
  NORMAL,              ///< All systems operational
  WARNING,             ///< Minor issues detected, monitoring
  DEGRADED,            ///< Operating with reduced functionality
  EMERGENCY_STOP,      ///< Emergency stop active
  SYSTEM_SHUTDOWN      ///< Complete system shutdown required
};

/**
 * @brief E-stop condition tracking structure.
 */
struct EstopCondition {
  EstopSource source;           ///< Source of the E-stop condition
  bool active;                  ///< Current active status
  bool requires_manual_reset;   ///< Requires manual reset to clear
  uint32_t activation_time;     ///< Timestamp when condition became active
  uint32_t last_check_time;     ///< Last time condition was evaluated
  String description;           ///< Human-readable description
  float trigger_value;          ///< Value that triggered condition (if applicable)
};

/**
 * @brief Safety configuration parameters.
 */
struct SafetyConfig {
  // Hardware pins
  uint8_t hardware_estop_pin = 2;      ///< Hardware E-stop input pin
  uint8_t estop_output_pin = 3;        ///< E-stop signal output pin
  uint8_t inter_board_input_pin = 4;   ///< Inter-board safety input
  uint8_t inter_board_output_pin = 5;  ///< Inter-board safety output
  
  // Timing parameters
  uint32_t estop_check_interval_ms = 10;  ///< How often to check E-stop conditions
  uint32_t recovery_delay_ms = 1000;      ///< Delay before allowing recovery
  uint32_t heartbeat_timeout_ms = 5000;   ///< Inter-board heartbeat timeout
  
  // Safety thresholds
  uint8_t max_consecutive_violations = 3; ///< Max violations before E-stop
  bool enable_auto_recovery = true;       ///< Enable automatic recovery
  bool enable_inter_board_safety = true;  ///< Enable inter-board coordination
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

  // --- Public API ---
  /**
   * @brief Check all safety conditions and update system state.
   * 
   * This method is called automatically by the main loop and should not be
   * called directly by other modules.
   */
  void checkSafetyStatus();

  /**
   * @brief Trigger an emergency stop from a software source.
   * @param source The source of the E-stop request
   * @param description A human-readable description of the reason
   */
  void triggerEstop(EstopSource source, const String& description);

  /**
   * @brief Attempt to recover from an E-stop condition.
   * 
   * This method will only succeed if all underlying safety conditions have
   * been cleared.
   */
  void attemptRecovery();

  /**
   * @brief Get the current safety state of the system.
   * @return The current safety state
   */
  SafetyState getSafetyState() const;

  /**
   * @brief Get the current E-stop condition.
   * @return Const reference to the current E-stop condition
   */
  const EstopCondition& getEstopCondition() const;

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

  // --- Private Methods ---
  /**
   * @brief Check for hardware E-stop button press.
   */
  void checkHardwareEstop();

  /**
   * @brief Check for safety signals from other boards.
   */
  void checkInterBoardSafety();

  /**
   * @brief Check for safety violations from other modules.
   */
  void checkModuleSafety();

  /**
   * @brief Activate the E-stop state.
   * @param source The source of the E-stop
   * @param description A description of the reason
   */
  void activateEstop(EstopSource source, const String& description);

  /**
   * @brief Deactivate the E-stop state.
   */
  void deactivateEstop();

  /**
   * @brief Send a status update via SerialManager.
   */
  void sendStatusUpdate();

  // --- Member Variables ---
  SafetyConfig config_;                   ///< Safety configuration
  SafetyState current_state_;             ///< Current safety state
  EstopCondition estop_condition_;        ///< Current E-stop condition
  uint32_t last_check_time_ms_ = 0;       ///< Timestamp of last safety check
  uint32_t last_status_update_ms_ = 0;    ///< Timestamp of last status update
};

}  // namespace sigyn_teensy
