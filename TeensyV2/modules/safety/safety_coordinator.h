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
 * @brief Central safety coordination system for TeensyV2.
 * 
 * Manages all E-stop conditions and safety states across the system.
 * Coordinates between multiple boards and provides unified safety reporting
 * to the ROS2 system. Implements both automatic and manual recovery mechanisms.
 * 
 * Key features:
 * - Centralized E-stop condition tracking from all sources
 * - Hardware and software E-stop input handling
 * - Inter-board safety coordination via GPIO signals
 * - Automatic recovery for transient conditions
 * - Manual recovery for persistent safety issues
 * - Real-time safety status reporting
 * 
 * Safety Philosophy:
 * - Fail-safe: Default to stopped state on any safety condition
 * - Transparent: All safety conditions visible and trackable
 * - Recoverable: Automatic recovery when conditions clear
 * - Coordinated: Multi-board systems work together
 * 
 * Usage:
 * @code
 * // Get singleton instance (automatically registers with module system)
 * SafetyCoordinator& safety = SafetyCoordinator::GetInstance();
 * 
 * // Trigger E-stop from module
 * safety.TriggerEstop(EstopSource::BATTERY_LOW_VOLTAGE, "Voltage: 31.2V", 31.2f);
 * 
 * // Check if recovery is needed
 * if (safety.GetSafetyState() == SafetyState::EMERGENCY_STOP) {
 *   safety.AttemptRecovery(EstopSource::BATTERY_LOW_VOLTAGE);
 * }
 * @endcode
 */
class SafetyCoordinator : public Module {
 public:
  // --- Constants ---
  /**
   * @brief Maximum number of E-stop conditions to track simultaneously.
   */
  static constexpr size_t kMaxEstopConditions = 16;

  // --- Singleton Access ---
  /**
   * @brief Get the singleton instance of SafetyCoordinator.
   * @return Reference to the singleton SafetyCoordinator instance
   */
  static SafetyCoordinator& GetInstance();

  // --- Module Interface ---
  bool IsUnsafe() override;
  void loop() override;
  const char* name() const override { return "SafetyCoordinator"; }
  void ProcessMessage(const String& message);
  void setup() override;

  // --- Public Methods ---
  uint8_t AttemptAutoRecovery();
  bool ClearEstop(EstopSource source);
  void Configure(const SafetyConfig& config);
  void ForceShutdown(const String& reason);
  uint8_t GetActiveConditions(EstopCondition* active_conditions, uint8_t max_conditions) const;
  SafetyState GetSafetyState() const { return current_safety_state_; }
  String GetSafetyStatusDescription() const;
  bool IsEstopActive() const;
  bool IsHardwareEstopPressed() const;
  bool ManualReset(EstopSource source, bool force_reset = false);
  void SetEstopSourceEnabled(EstopSource source, bool enabled);
  void TriggerEstop(EstopSource source, 
                    const String& description, 
                    float trigger_value = NAN,
                    bool requires_manual_reset = false);

 private:
  // --- Private constructor for Singleton ---
  SafetyCoordinator();
  SafetyCoordinator(const SafetyCoordinator&) = delete;
  SafetyCoordinator& operator=(const SafetyCoordinator&) = delete;

  // --- Private Methods ---
  void CheckHardwareEstop();
  void CheckInterBoardSafety();
  EstopCondition* FindCondition(EstopSource source);
  void InitializeHardware();
  void SendEstopMessage(const EstopCondition& condition, bool activated);
  void SendSafetyStatusMessage();
  void UpdateSafetyState();

  // --- Static Members ---
  static SafetyCoordinator* instance_;

  // --- Member Variables ---
  uint8_t condition_count_;
  EstopCondition conditions_[kMaxEstopConditions];
  SafetyConfig config_;
  SafetyState current_safety_state_;
  bool inter_board_estop_active_;
  uint32_t last_heartbeat_received_ms_;
  uint32_t last_heartbeat_sent_ms_;
  uint32_t last_status_report_ms_;
};

}  // namespace sigyn_teensy
