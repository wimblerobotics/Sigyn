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
#include "module.h"
#include "serial_manager.h"

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
   * @brief Check if E-stop is currently active.
   * @return True if E-stop is active
   */
  bool isEstopActive() const;

  /**
   * @brief Trigger a software E-stop.
   * @param source The source of the E-stop request
   * @param description A description of the reason for the E-stop
   */
  void triggerEstop(EstopSource source, const String& description);

  /**
   * @brief Attempt to clear an E-stop condition.
   * @return True if E-stop was successfully cleared
   */
  bool clearEstop();

  /**
   * @brief Return the name of this module.
   */
  const char* name() const override;

 protected:
  // --- Module Overrides ---
  void setup() override;
  void loop() override;

 private:
  // --- Private Constructor ---
  SafetyCoordinator();
  ~SafetyCoordinator() = default;
  SafetyCoordinator(const SafetyCoordinator&) = delete;
  SafetyCoordinator& operator=(const SafetyCoordinator&) = delete;

  // --- Member Functions ---
  void checkEstopConditions();
  void updateSafetyState();
  void sendHeartbeat();
  void checkHeartbeat();

  // --- Data Members ---
  SafetyConfig config_;
  SafetyState current_state_;
  EstopCondition estop_conditions_[10];  // Array to hold multiple E-stop sources
  uint8_t estop_condition_count_ = 0;
  uint32_t last_estop_check_time_ms_ = 0;
  uint32_t last_heartbeat_time_ms_ = 0;
};

}  // namespace sigyn_teensy
