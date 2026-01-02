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
// #include <cstdio>
#include <cstring>

#include "common/core/config.h"
#include "common/core/serial_manager.h"
#if CONTROLS_ROBOCLAW_ESTOP_PIN
#include "modules/roboclaw/roboclaw_monitor.h"
#endif

namespace sigyn_teensy {

SafetyCoordinator::SafetyCoordinator() {
  // Initialize faults array
  for (int i = 0; i < static_cast<int>(FaultSource::NUMBER_FAULT_SOURCES); i++) {
    faults_[i].active = false;
    faults_[i].source = static_cast<FaultSource>(i);
    faults_[i].severity = FaultSeverity::NORMAL;
    faults_[i].description = "";
    faults_[i].timestamp = 0;
  }
}

void SafetyCoordinator::activateFault(FaultSeverity severity, FaultSource source, const String& description) {
  char msg[256];
  int idx = static_cast<int>(source);
  const bool new_is_estop = (severity == FaultSeverity::EMERGENCY_STOP || severity == FaultSeverity::SYSTEM_SHUTDOWN);
  const bool was_active = faults_[idx].active;
  const bool was_estop = (faults_[idx].severity == FaultSeverity::EMERGENCY_STOP ||
                          faults_[idx].severity == FaultSeverity::SYSTEM_SHUTDOWN);

  // Always update the fault record (so WARNING can upgrade to EMERGENCY_STOP, etc.)
  faults_[idx].active = true;
  faults_[idx].source = source;
  faults_[idx].severity = severity;
  faults_[idx].description = description;
  faults_[idx].timestamp = millis();

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
           was_active ? "Fault updated" : "Fault activated", faultSourceToString(source),
           faultSeverityToString(severity), description.c_str(), active_estop_count_);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
}

void SafetyCoordinator::deactivateFault(FaultSource source) {
  int idx = static_cast<int>(source);
  if (!faults_[idx].active) {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(),
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
               faultSourceToString(source), faultSeverityToString(faults_[idx].severity),
               faults_[idx].description.c_str(), active_estop_count_);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
    }
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
  for (int i = 0; i < static_cast<int>(FaultSource::NUMBER_FAULT_SOURCES); i++) {
    if (faults_[i].active) {
      deactivateFault(static_cast<FaultSource>(i));
    }
  }
  active_estop_count_ = 0;
}

void SafetyCoordinator::setup() {
}

void SafetyCoordinator::setEstopCommand(String command) {
  if (command.indexOf("trigger=true") >= 0) {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(),
                                                       "Software E-stop command received, activating E-stop");
    activateFault(FaultSeverity::EMERGENCY_STOP, FaultSource::SOFTWARE_COMMAND, "Software E-stop command received");
  } else if (command.indexOf("reset=true") >= 0) {
    SerialManager::getInstance().sendDiagnosticMessage(
        "INFO", name(), "Software E-stop reset command received, attempting to clear E-stop");
    deactivateFault(FaultSource::SOFTWARE_COMMAND);
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

const Fault& SafetyCoordinator::getFault(FaultSource source) const {
  return faults_[static_cast<size_t>(source)];
}

void SafetyCoordinator::sendStatusUpdate() {
  char status_msg[512];
  bool any_active = false;
  for (size_t i = 0; i < static_cast<size_t>(FaultSource::NUMBER_FAULT_SOURCES); i++) {
    if (faults_[i].active) {
      any_active = true;
      snprintf(status_msg, sizeof(status_msg),
               "{\"active_fault\":\"true\",\"source\":\"%s\",\"severity\":\"%s\",\"description\":\"%s\",\"timestamp\":%lu}",
               faultSourceToString(faults_[i].source), faultSeverityToString(faults_[i].severity),
               faults_[i].description.c_str(), static_cast<unsigned long>(faults_[i].timestamp));
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
