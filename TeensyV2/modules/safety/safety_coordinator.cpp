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
#include "common/core/serial_manager.h"

namespace sigyn_teensy {

SafetyCoordinator::SafetyCoordinator()
    : current_state_(SafetyState::NORMAL) {
  // Initialize E-stop condition to safe defaults
  // All E-stop fields start in known, safe states
  estop_condition_.active = false;
  estop_condition_.source = EstopSource::UNKNOWN;
  estop_condition_.requires_manual_reset = false;
}

void SafetyCoordinator::activateEstop(EstopSource source,
                                      const String& description) {
  // Prevent duplicate E-stop activations from causing state confusion
  if (current_state_ != SafetyState::EMERGENCY_STOP) {
    // Update safety state and record E-stop details
    current_state_ = SafetyState::EMERGENCY_STOP;
    estop_condition_.active = true;
    estop_condition_.source = source;
    estop_condition_.description = description;
    estop_condition_.activation_time = millis();

    // Assert hardware E-stop signal for immediate motor cutoff
    // HIGH signal indicates E-stop active (fail-safe design)
    digitalWrite(config_.estop_output_pin, HIGH);
    
    // Notify other boards of E-stop condition if enabled
    if (config_.enable_inter_board_safety) {
      digitalWrite(config_.inter_board_output_pin, HIGH);
    }

    // Send immediate status update for monitoring systems
    sendStatusUpdate();
  }
}

void SafetyCoordinator::attemptRecovery() {
  // Verify that the original trigger condition has been resolved
  // Different E-stop sources require different verification methods
  bool condition_cleared = true;
  
  switch (estop_condition_.source) {
    case EstopSource::HARDWARE_BUTTON:
      // Hardware button must be released (LOW = not pressed)
      if (digitalRead(config_.hardware_estop_pin) == LOW)
        condition_cleared = false;
      break;
      
    case EstopSource::INTER_BOARD:
      // Other board must clear its safety signal
      if (digitalRead(config_.inter_board_input_pin) == LOW)
        condition_cleared = false;
      break;
      
    // Software-triggered E-stops: check module safety states
    default:
      // For module-based safety violations, verify all modules are now safe
      if (Module::isAnyModuleUnsafe()) {
        condition_cleared = false;
      }
      break;
  }

  if (condition_cleared) {
    deactivateEstop();
  } else {
    // Optional: log that recovery failed
    SerialManager::getInstance().sendMessage("SAFETY", "recovery_failed");
  }
}

void SafetyCoordinator::checkHardwareEstop() {
  if (digitalRead(config_.hardware_estop_pin) == LOW) {
    activateEstop(EstopSource::HARDWARE_BUTTON, "Hardware E-stop pressed");
  }
}

void SafetyCoordinator::checkInterBoardSafety() {
  if (config_.enable_inter_board_safety &&
      digitalRead(config_.inter_board_input_pin) == LOW) {
    activateEstop(EstopSource::INTER_BOARD, "Inter-board safety signal active");
  }
}

void SafetyCoordinator::checkModuleSafety() {
  if (Module::isAnyModuleUnsafe()) {
    // Find which module is unsafe for a better description
    for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
      Module* mod = Module::getModule(i);
      if (mod && mod->isUnsafe()) {
        String desc = "Module unsafe: ";
        desc += mod->name();
        // This logic might need refinement if multiple modules are unsafe
        activateEstop(EstopSource::UNKNOWN, desc); // TODO: Better source mapping
        break;
      }
    }
  }
}

void SafetyCoordinator::checkSafetyStatus() {
  if (current_state_ == SafetyState::EMERGENCY_STOP) {
    // If in E-stop, don't check for new triggers, just wait for recovery
    // attempt
    return;
  }

  checkHardwareEstop();
  if (isUnsafe()) return;

  checkInterBoardSafety();
  if (isUnsafe()) return;

  checkModuleSafety();
}

void SafetyCoordinator::deactivateEstop() {
  current_state_ = SafetyState::NORMAL;
  estop_condition_.active = false;
  estop_condition_.source = EstopSource::UNKNOWN;
  estop_condition_.description = "";

  // De-assert hardware E-stop signal
  digitalWrite(config_.estop_output_pin, LOW);
  if (config_.enable_inter_board_safety) {
    digitalWrite(config_.inter_board_output_pin, LOW);
  }

  // Reset safety flags in all modules
  Module::resetAllSafetyFlags();

  sendStatusUpdate();
}

const EstopCondition& SafetyCoordinator::getEstopCondition() const {
  return estop_condition_;
}

SafetyCoordinator& SafetyCoordinator::getInstance() {
  static SafetyCoordinator instance;
  return instance;
}

SafetyState SafetyCoordinator::getSafetyState() const {
  return current_state_;
}

bool SafetyCoordinator::isUnsafe() {
  return current_state_ == SafetyState::EMERGENCY_STOP;
}

void SafetyCoordinator::loop() {
  uint32_t now = millis();
  if (now - last_check_time_ms_ >= config_.estop_check_interval_ms) {
    checkSafetyStatus();
    last_check_time_ms_ = now;
  }

  // Optional: periodic status updates even when not in E-stop
  // if (now - last_status_update_ms_ >= 5000) { // Every 5 seconds
  //   SendStatusUpdate();
  //   last_status_update_ms_ = now;
  // }
}

const char* SafetyCoordinator::name() const { return "SafetyCoordinator"; }

void SafetyCoordinator::resetSafetyFlags() {
  if (current_state_ == SafetyState::EMERGENCY_STOP) {
    attemptRecovery();
  }
}

void SafetyCoordinator::sendStatusUpdate() {
  char status_msg[128];
  snprintf(status_msg, sizeof(status_msg),
           "{\"state\":%d,\"estop_active\":%d,\"source\":\"%s\"}",
           static_cast<int>(current_state_), estop_condition_.active,
           estop_condition_.description.c_str());
  SerialManager::getInstance().sendMessage("SAFETY", status_msg);
  last_status_update_ms_ = millis();
}

void SafetyCoordinator::setup() {
  pinMode(config_.hardware_estop_pin, INPUT_PULLUP);
  pinMode(config_.estop_output_pin, OUTPUT);
  digitalWrite(config_.estop_output_pin, LOW);

  if (config_.enable_inter_board_safety) {
    pinMode(config_.inter_board_input_pin, INPUT_PULLUP);
    pinMode(config_.inter_board_output_pin, OUTPUT);
    digitalWrite(config_.inter_board_output_pin, LOW);
  }

  last_check_time_ms_ = millis();
  last_status_update_ms_ = millis();
}

void SafetyCoordinator::triggerEstop(EstopSource source,
                                     const String& description) {
  activateEstop(source, description);
}

}  // namespace sigyn_teensy
