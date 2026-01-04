// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file safety_coordinator.cpp
 * @brief Implementation of central safety coordination system
 *
 * This file implements the SafetyCoordinator module that provides centralized
 * safety management, E-stop coordination, and emergency response for the
 * TeensyV2 system. The implementation handles multiple E-stop sources with
 * appropriate recovery logic and inter-board safety communication.
 *
 * Key Implementation Features:
 *
 * **E-stop Management:**
 * - Multiple trigger sources (hardware, software, performance, battery)
 * - Automatic hardware signal assertion for immediate motor cutoff
 * - Inter-board safety communication for system-wide coordination
 * - Configurable manual vs. automatic recovery modes
 *
 * **Safety State Machine:**
 * - NORMAL: All systems operational, continuous monitoring active
 * - WARNING: Non-critical issues detected, degraded operation possible
 * - EMERGENCY_STOP: Critical safety violation, all motion disabled
 * - RECOVERY: Attempting to clear E-stop conditions and return to normal
 *
 * **Recovery Logic:**
 * - Automatic recovery when transient conditions clear (e.g., performance)
 * - Manual recovery required for persistent conditions (e.g., hardware button)
 * - Comprehensive condition verification before recovery completion
 * - Configurable recovery delays to prevent oscillation
 *
 * **Communication Integration:**
 * - Real-time status updates via SerialManager
 * - Structured message format for ROS2 integration
 * - Emergency priority messaging during safety events
 * - Diagnostic logging for post-incident analysis
 *
 * **Hardware Integration:**
 * - Direct GPIO control for hardware E-stop output signals
 * - Interrupt-driven monitoring of hardware E-stop inputs
 * - Inter-board safety signals for multi-controller coordination
 * - Fail-safe design with active-high safety signals
 *
 * The implementation prioritizes safety above all other considerations,
 * ensuring that any safety violation results in immediate protective
 * action while maintaining system visibility and recovery capabilities.
 *
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 */

#include "safety_coordinator.h"

#include <Arduino.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "common/core/config.h"
#include "common/core/serial_manager.h"
#if CONTROLS_ROBOCLAW_ESTOP_PIN
#include "modules/roboclaw/roboclaw_monitor.h"
#endif

namespace sigyn_teensy {

int SafetyCoordinator::findFaultIndex(const char* source) const {
  if (source == nullptr || source[0] == '\0') {
    return -1;
  }

  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].source[0] != '\0' && sourcesEqual(faults_[i].source, source)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

int SafetyCoordinator::getOrAllocateFaultIndex(const char* source) {
  if (source == nullptr || source[0] == '\0') {
    return -1;
  }

  const int existing = findFaultIndex(source);
  if (existing >= 0) {
    return existing;
  }

  // Prefer an unused slot.
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].source[0] == '\0') {
      return static_cast<int>(i);
    }
  }

  // Next best: reuse an inactive slot.
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (!faults_[i].active) {
      faults_[i].clear();
      return static_cast<int>(i);
    }
  }

  // Worst case: overwrite the oldest fault.
  size_t oldest_idx = 0;
  uint32_t oldest_ts = faults_[0].timestamp;
  for (size_t i = 1; i < kMaxFaults; i++) {
    if (faults_[i].timestamp < oldest_ts) {
      oldest_ts = faults_[i].timestamp;
      oldest_idx = i;
    }
  }
  faults_[oldest_idx].clear();
  return static_cast<int>(oldest_idx);
}

SafetyCoordinator::SafetyCoordinator() {
  // Initialize faults array
  for (size_t i = 0; i < kMaxFaults; i++) {
    faults_[i].clear();
  }
  empty_fault_.clear();
}

void SafetyCoordinator::activateFault(FaultSeverity severity, const char* source, const char* description) {
  char msg[256];
  const int idx = getOrAllocateFaultIndex(source);
  if (idx < 0) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(),
                                                       "activateFault called with empty/null source");
    return;
  }
  const bool new_is_estop = (severity == FaultSeverity::EMERGENCY_STOP || severity == FaultSeverity::SYSTEM_SHUTDOWN);
  const bool was_active = faults_[idx].active;
  const bool was_estop = (faults_[idx].severity == FaultSeverity::EMERGENCY_STOP ||
                          faults_[idx].severity == FaultSeverity::SYSTEM_SHUTDOWN);

  // Always update the fault record (so WARNING can upgrade to EMERGENCY_STOP, etc.)
  faults_[idx] = Fault(/*is_active=*/true, severity, source, description);

  // Update E-stop count based on severity transitions
  if (!was_active && new_is_estop) {
    active_estop_count_++;
  } else if (was_active && !was_estop && new_is_estop) {
    // Upgrade WARNING/DEGRADED -> EMERGENCY_STOP
    active_estop_count_++;
  } else if (was_active && was_estop && !new_is_estop) {
    // Downgrade EMERGENCY_STOP -> WARNING/DEGRADED
    if (active_estop_count_ > 0) {
      active_estop_count_--;
    }
  }

  // Trigger physical safeties only when entering E-stop state
  if (new_is_estop && active_estop_count_ == 1) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
    RoboClawMonitor::getInstance().setEmergencyStop();
#endif
  }

  snprintf(msg, sizeof(msg), "%s: source=%s, severity=%s description=%s, active_estop_count_=%d",
           was_active ? "Fault updated" : "Fault activated", faults_[idx].source, faultSeverityToString(severity),
           faults_[idx].description, active_estop_count_);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
}

void SafetyCoordinator::deactivateFault(const char* source) {
  const int idx = findFaultIndex(source);
  if (idx < 0 || !faults_[idx].active) {
    SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(),
                                                       "Attempted to deactivate a fault that is not active");
    return;
  } else {
    const bool was_estop = (faults_[idx].severity == FaultSeverity::EMERGENCY_STOP ||
                            faults_[idx].severity == FaultSeverity::SYSTEM_SHUTDOWN);
    faults_[idx].active = false;
    if (was_estop && active_estop_count_ > 0) {
      active_estop_count_--;
    }

    // If this was the LAST active fault, clear the physical safeties
    if (active_estop_count_ == 0) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
      RoboClawMonitor::getInstance().clearEmergencyStop();
#endif
      char msg[256];
      snprintf(msg, sizeof(msg), "Fault deactivated: source=%s, severity=%s, description=%s, active_faults=%d",
               faults_[idx].source, faultSeverityToString(faults_[idx].severity), faults_[idx].description,
               active_estop_count_);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
    }

    // Release the slot.
    faults_[idx].clear();
  }
}

SafetyCoordinator& SafetyCoordinator::getInstance() {
  static SafetyCoordinator instance;
  return instance;
}

// FaultSeverity SafetyCoordinator::getSafetyState() const { return current_state_; }

bool SafetyCoordinator::isUnsafe() {
  // Unsafe == E-stop level fault active
  return active_estop_count_ > 0;
}

void SafetyCoordinator::loop() {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  static uint32_t last_check_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms - last_check_ms >= 1000) {
    last_check_ms = now_ms;
    sendStatusUpdate();
  }
#endif
}

const char* SafetyCoordinator::name() const { return "SafetyCoordinator"; }

void SafetyCoordinator::resetSafetyFlags() {
  // Clear all active faults
  for (size_t i = 0; i < kMaxFaults; i++) {
    faults_[i].clear();
  }
  active_estop_count_ = 0;
}

void SafetyCoordinator::setup() {
}

void SafetyCoordinator::setEstopCommand(const char* command) {
  if (command != nullptr && strstr(command, "trigger=true") != nullptr) {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(),
                                                       "Software E-stop command received, activating E-stop");
    activateFault(FaultSeverity::EMERGENCY_STOP, name(), "Software E-stop command received");
  } else if (command != nullptr && strstr(command, "reset=true") != nullptr) {
    SerialManager::getInstance().sendDiagnosticMessage(
        "INFO", name(), "Software E-stop reset command received, attempting to clear E-stop");
    deactivateFault(name());
  }
}

void SafetyCoordinator::setRoboClawPower(bool on) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  digitalWrite(PIN_RELAY_ROBOCLAW_POWER, on ? HIGH : LOW);
#else
  (void)on;  // Suppress unused parameter warning
#endif
}

void SafetyCoordinator::setMainBatteryPower(bool on) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  digitalWrite(PIN_RELAY_MAIN_BATTERY, on ? HIGH : LOW);
#else
  (void)on;  // Suppress unused parameter warning
#endif
}

const Fault& SafetyCoordinator::getFault(const char* source) const {
  const int idx = findFaultIndex(source);
  if (idx < 0) {
    return empty_fault_;
  }
  return faults_[static_cast<size_t>(idx)];
}

void SafetyCoordinator::sendStatusUpdate() {
  char status_msg[512];
  bool any_active = false;
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].active) {
      any_active = true;
      snprintf(status_msg, sizeof(status_msg),
               "{\"active_fault\":\"true\",\"source\":\"%s\",\"severity\":\"%s\",\"description\":\"%s\",\"timestamp\":%lu}",
               faults_[i].source, faultSeverityToString(faults_[i].severity), faults_[i].description,
               static_cast<unsigned long>(faults_[i].timestamp));
      SerialManager::getInstance().sendMessage("FAULT", status_msg);
    }
  }

  if (!any_active) {
    snprintf(status_msg, sizeof(status_msg), "{\"active_fault\":\"false\"}");
    SerialManager::getInstance().sendMessage("FAULT", status_msg);
    return;
  }
}

}  // namespace sigyn_teensy
