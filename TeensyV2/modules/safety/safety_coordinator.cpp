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
  if (!faults_[idx].active) {
    faults_[idx].active = true;
    faults_[idx].source = source;
    faults_[idx].severity = severity;
    faults_[idx].description = description;
    faults_[idx].timestamp = millis();
    active_estop_count_++;

    // If this is the FIRST fault, trigger the physical safeties
    if (active_estop_count_ == 1) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
      RoboClawMonitor::getInstance().setEmergencyStop();
#else
      digitalWrite(PIN_SAFETY_OUT_TO_MASTER, HIGH);
#endif
      snprintf(msg, sizeof(msg), "Fault activated: source=%s, severity=%s description=%s, active_estop_count_=%d",
               faultSourceToString(source), faultSeverityToString(severity), description.c_str(), active_estop_count_);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
    }
  } else {
    snprintf(msg, sizeof(msg), "Fault already active: source=%s, severity=%s description=%s, active_estop_count_=%d",
             faultSourceToString(source), faultSeverityToString(severity), description.c_str(), active_estop_count_);
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
  }
}

void SafetyCoordinator::deactivateFault(FaultSource source) {
  int idx = static_cast<int>(source);
  if (!faults_[idx].active) {
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(),
                                                       "Attempted to deactivate a fault that is not active");
    return;
  } else {
    faults_[idx].active = false;
    active_estop_count_--;

    // If this was the LAST active fault, clear the physical safeties
    if (active_estop_count_ == 0) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
      RoboClawMonitor::getInstance().clearEmergencyStop();
#else
      digitalWrite(PIN_SAFETY_OUT_TO_MASTER, LOW);
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
  // return current_state_ == FaultSeverity::EMERGENCY_STOP;
  return false;
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
  // if (current_state_ == FaultSeverity::EMERGENCY_STOP) {
  //   attemptRecovery();
  // }
}

void SafetyCoordinator::setup() {
  // #if CONTROLS_ROBOCLAW_ESTOP_PIN
  //   pinMode(PIN_SAFETY_IN_BOARD2, INPUT_PULLUP);
  //   pinMode(PIN_SAFETY_IN_BOARD3, INPUT_PULLUP);
  //   pinMode(PIN_RELAY_ROBOCLAW_POWER, OUTPUT);
  //   pinMode(PIN_RELAY_MAIN_BATTERY, OUTPUT);
  //   digitalWrite(PIN_RELAY_ROBOCLAW_POWER, LOW);
  //   digitalWrite(PIN_RELAY_MAIN_BATTERY, LOW);
  // #else
  //   pinMode(PIN_SAFETY_OUT_TO_MASTER, OUTPUT);
  //   digitalWrite(PIN_SAFETY_OUT_TO_MASTER, LOW);
  // #endif
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
#endif
}

void SafetyCoordinator::setMainBatteryPower(bool on) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  digitalWrite(PIN_RELAY_MAIN_BATTERY, on ? HIGH : LOW);
#endif
}

// void SafetyCoordinator::attemptRecovery() {
//   // Verify that the original trigger condition has been resolved
//   // Different E-stop sources require different verification methods
//   bool condition_cleared = true;

//   switch (estop_condition_.source) {
//     case FaultSource::HARDWARE_BUTTON:
//       // Hardware button must be released (LOW = not pressed)
//       // if (digitalRead(config_.hardware_estop_pin) == LOW) condition_cleared = false;
//       break;

//     // case EstopSource::INTER_BOARD:
//     //   // Other board must clear its safety signal
//     //   if (digitalRead(config_.inter_board_input_pin) == LOW) condition_cleared = false;
//     //   break;

//     // Software-triggered E-stops: check module safety states
//     default:
//       // For module-based safety violations, verify all modules are now safe
//       if (Module::isAnyModuleUnsafe()) {
//         condition_cleared = false;
//       }
//       break;
//   }

//   if (condition_cleared) {
//     deactivateEstop();
//   } else {
//     // Optional: log that recovery failed
//     SerialManager::getInstance().sendDiagnosticMessage("SAFETY", name(), "recovery_failed");
//   }
// }

// void SafetyCoordinator::checkHardwareEstop() {
//   // if (digitalRead(config_.hardware_estop_pin) == LOW) {
//   //   activateEstop(EstopSource::HARDWARE_BUTTON, "Hardware E-stop pressed");
//   // }
// }

// void SafetyCoordinator::checkInterBoardSafety() {
//   if (config_.enable_inter_board_safety && digitalRead(config_.inter_board_input_pin) == LOW) {
//     activateEstop(EstopSource::INTER_BOARD, "Inter-board safety signal active");
//   }
// }

// void SafetyCoordinator::checkModuleSafety() {
//   if (Module::isAnyModuleUnsafe()) {
//     // Find which module is unsafe for a better description
//     for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
//       Module* mod = Module::getModule(i);
//       if (mod && mod->isUnsafe()) {
//         String desc = "Module unsafe: ";
//         desc += mod->name();
//         // This logic might need refinement if multiple modules are unsafe
//         activateEstop(FaultSource::UNKNOWN,
//                       desc);  // TODO: Better source mapping
//         break;
//       }
//     }
//   }
// }

// void SafetyCoordinator::checkSafetyStatus() {
//   if (current_state_ == FaultSeverity::EMERGENCY_STOP) {
//     // If in E-stop, don't check for new triggers, just wait for recovery
//     // attempt
//     return;
//   }

//   // checkHardwareEstop();
//   // if (isUnsafe()) return;

//   // checkInterBoardSafety();
//   // if (isUnsafe()) return;

//   checkModuleSafety();
// }

// const EstopCondition& SafetyCoordinator::getEstopCondition() const { return estop_condition_; }

void SafetyCoordinator::sendStatusUpdate() {
  char status_msg[512];
  bool any_active = false;
  for (size_t i = 0; i < static_cast<size_t>(FaultSource::NUMBER_FAULT_SOURCES); i++) {
    if (faults_[i].active) {
      any_active = true;
      snprintf(status_msg, sizeof(status_msg),
               "{\"active_fault\":\"true\",\"source\":\"%s\",\"severity\":\"%s\",\"description\":\"%s\",\"timestamp\":%lu}",
               faultSourceToString(faults_[i].source), faultSeverityToString(faults_[i].severity),
               faults_[i].description.c_str(), faults_[i].timestamp);
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
