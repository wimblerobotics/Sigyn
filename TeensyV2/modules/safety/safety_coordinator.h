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
 * @author Sigyn Robotics
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
  /**
   * @brief Maximum number of E-stop conditions to track simultaneously.
   */
  static constexpr size_t kMaxEstopConditions = 16;

  /**
   * @brief Get the singleton instance of SafetyCoordinator.
   * 
   * @return Reference to the singleton SafetyCoordinator instance
   */
  static SafetyCoordinator& GetInstance();

  /**
   * @brief Configure safety system parameters.
   * 
   * @param[in] config Safety configuration structure
   */
  void Configure(const SafetyConfig& config);

  /**
   * @brief Trigger an E-stop condition.
   * 
   * @param[in] source Source of the E-stop condition
   * @param[in] description Human-readable description of the condition
   * @param[in] trigger_value Optional value that triggered the condition
   * @param[in] requires_manual_reset true if manual reset required
   */
  void TriggerEstop(EstopSource source, 
                    const String& description, 
                    float trigger_value = NAN,
                    bool requires_manual_reset = false);

  /**
   * @brief Clear an E-stop condition (automatic recovery).
   * 
   * @param[in] source Source of the E-stop condition to clear
   * @return true if condition was cleared, false if manual reset required
   */
  bool ClearEstop(EstopSource source);

  /**
   * @brief Manually reset an E-stop condition.
   * 
   * @param[in] source Source of the E-stop condition to reset
   * @param[in] force_reset true to force reset even if condition still active
   * @return true if reset successful, false otherwise
   */
  bool ManualReset(EstopSource source, bool force_reset = false);

  /**
   * @brief Attempt automatic recovery for all clearable conditions.
   * 
   * @return Number of conditions that were successfully cleared
   */
  uint8_t AttemptAutoRecovery();

  /**
   * @brief Get current overall safety state.
   * 
   * @return Current safety state enumeration
   */
  SafetyState GetSafetyState() const { return current_safety_state_; }

  /**
   * @brief Check if any E-stop conditions are currently active.
   * 
   * @return true if any E-stop condition is active
   */
  bool IsEstopActive() const;

  /**
   * @brief Get list of currently active E-stop conditions.
   * 
   * @param[out] active_conditions Array to fill with active conditions
   * @param[in] max_conditions Maximum number of conditions to return
   * @return Number of active conditions returned
   */
  uint8_t GetActiveConditions(EstopCondition* active_conditions, uint8_t max_conditions) const;

  /**
   * @brief Get description of current safety status.
   * 
   * @return Human-readable safety status description
   */
  String GetSafetyStatusDescription() const;

  /**
   * @brief Force immediate system shutdown.
   * 
   * Used for critical safety conditions that require immediate action.
   * 
   * @param[in] reason Description of why shutdown was triggered
   */
  void ForceShutdown(const String& reason);

  /**
   * @brief Enable or disable a specific E-stop source.
   * 
   * @param[in] source E-stop source to enable/disable
   * @param[in] enabled true to enable, false to disable
   */
  void SetEstopSourceEnabled(EstopSource source, bool enabled);

  /**
   * @brief Check if hardware E-stop button is currently pressed.
   * 
   * @return true if hardware E-stop is active
   */
  bool IsHardwareEstopPressed() const;

 protected:
  // Module interface implementation
  void setup() override;
  void loop() override;
  const char* name() const override { return "SafetyCoordinator"; }
  bool IsUnsafe() override;
  void ProcessMessage(const String& message);

 private:
  /**
   * @brief Private constructor for singleton pattern.
   */
  SafetyCoordinator();

  /**
   * @brief Initialize hardware pins for safety monitoring.
   */
  void InitializeHardware();

  /**
   * @brief Check hardware E-stop button state.
   */
  void CheckHardwareEstop();

  /**
   * @brief Check inter-board safety signals.
   */
  void CheckInterBoardSafety();

  /**
   * @brief Update overall safety state based on active conditions.
   */
  void UpdateSafetyState();

  /**
   * @brief Send safety status message via serial.
   */
  void SendSafetyStatusMessage();

  /**
   * @brief Send E-stop notification message.
   * 
   * @param[in] condition E-stop condition to report
   * @param[in] activated true if condition activated, false if cleared
   */
  void SendEstopMessage(const EstopCondition& condition, bool activated);

  /**
   * @brief Find E-stop condition by source.
   * 
   * @param[in] source E-stop source to find
   * @return Pointer to condition if found, nullptr otherwise
   */
  EstopCondition* FindCondition(EstopSource source);

  /**
   * @brief Create new E-stop condition.
   * 
   * @param[in] source E-stop source
   * @return Pointer to new condition, nullptr if no space available
   */
  EstopCondition* CreateCondition(EstopSource source);

  /**
   * @brief Convert E-stop source to string.
   * 
   * @param[in] source E-stop source enumeration
   * @return String representation of source
   */
  String EstopSourceToString(EstopSource source) const;

  /**
   * @brief Convert safety state to string.
   * 
   * @param[in] state Safety state enumeration
   * @return String representation of state
   */
  String SafetyStateToString(SafetyState state) const;

  // Configuration
  SafetyConfig config_;

  // E-stop condition tracking
  EstopCondition estop_conditions_[kMaxEstopConditions];
  uint8_t condition_count_;

  // Safety state
  SafetyState current_safety_state_;
  SafetyState previous_safety_state_;

  // Hardware state tracking
  bool hardware_estop_state_;
  bool inter_board_safety_state_;
  uint32_t last_heartbeat_time_;

  // Timing control
  uint32_t last_safety_check_;
  uint32_t last_status_report_;
  uint32_t shutdown_time_;

  // Serial communication
  SerialManager* serial_manager_;

  // Source enable/disable flags
  bool source_enabled_[static_cast<uint8_t>(EstopSource::UNKNOWN) + 1];
};

}  // namespace sigyn_teensy
