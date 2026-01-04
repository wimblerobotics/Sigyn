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

#include <cstdio>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

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
  static constexpr size_t kMaxSourceLen = 32;
  static constexpr size_t kMaxDescriptionLen = 128;

  bool active = false;             ///< Is the fault currently active
  FaultSeverity severity = FaultSeverity::NORMAL;  ///< Severity level of the fault
  char source[kMaxSourceLen] = {0};                ///< Source module name (null-terminated)
  char description[kMaxDescriptionLen] = {0};      ///< Human-readable description (null-terminated)
  uint32_t timestamp = 0;                          ///< Time when fault was detected

  Fault() = default;

  Fault(bool is_active, FaultSeverity sev, const char* src, const char* desc)
      : active(is_active), severity(sev), timestamp(millis()) {
    setSource(src);
    setDescription(desc);
  }

  void clear() {
    active = false;
    severity = FaultSeverity::NORMAL;
    timestamp = 0;
    source[0] = '\0';
    description[0] = '\0';
  }

  void setSource(const char* src) {
    snprintf(source, sizeof(source), "%s", (src != nullptr) ? src : "");
  }

  void setDescription(const char* desc) {
    snprintf(description, sizeof(description), "%s", (desc != nullptr) ? desc : "");
  }
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
  // --- Test Access ---
  friend class SafetyCoordinatorTest;
  
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
    * @param source Module name that raised the fault
   * @param description A description of the reason
   */
    void activateFault(FaultSeverity severity, const char* source, const char* description);

  /**
   * @brief Deactivate the E-stop state.
   */
  void deactivateFault(const char* source);

  void setEstopCommand(const char* command);

  /**
   * @brief Get a specific fault by source.
    * @param source The module name to query
   * @return Const reference to the fault
   */
    const Fault& getFault(const char* source) const;

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

#ifdef UNIT_TEST
  // Public wrappers for testing
  void testSetup() { setup(); }
  void testLoop() { loop(); }
#endif

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
  /**
   * @brief Send a status update via SerialManager.
   */
  void sendStatusUpdate();

  // --- Member Variables ---
  uint8_t active_estop_count_ = 0;

  static constexpr size_t kMaxFaults = 16;
  Fault faults_[kMaxFaults];  ///< Active faults (fixed-size, heap-free)
  Fault empty_fault_;

  static bool sourcesEqual(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
      return false;
    }
    return strncmp(a, b, Fault::kMaxSourceLen) == 0;
  }

  int findFaultIndex(const char* source) const;
  int getOrAllocateFaultIndex(const char* source);
};

[[maybe_unused]] static const char* faultSeverityToString(FaultSeverity severity) {
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
