/**
 * @file safety_coordinator.cpp
 * @brief Implementation of central safety coordination system
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#include "safety_coordinator.h"

namespace sigyn_teensy {

SafetyCoordinator::SafetyCoordinator()
    : current_state_(SafetyState::NORMAL),
      estop_condition_count_(0),
      last_estop_check_time_ms_(0),
      last_heartbeat_time_ms_(0) {}

bool SafetyCoordinator::clearEstop() {
  bool all_cleared = true;
  for (uint8_t i = 0; i < estop_condition_count_; ++i) {
    if (estop_conditions_[i].active &&
        !estop_conditions_[i].requires_manual_reset) {
      estop_conditions_[i].active = false;
    } else if (estop_conditions_[i].active) {
      all_cleared = false;
    }
  }

  if (all_cleared) {
    updateSafetyState();
  }
  return all_cleared;
}

void SafetyCoordinator::checkEstopConditions() {
  // Check hardware E-stop
  if (digitalRead(config_.hardware_estop_pin) == LOW) {
    triggerEstop(EstopSource::HARDWARE_BUTTON, "Hardware E-stop pressed");
  }

  // Check inter-board safety signal
  if (digitalRead(config_.inter_board_input_pin) == LOW) {
    triggerEstop(EstopSource::INTER_BOARD, "Inter-board safety signal");
  }

  checkHeartbeat();
}

void SafetyCoordinator::checkHeartbeat() {
  if (config_.enable_inter_board_safety &&
      millis() - last_heartbeat_time_ms_ > config_.heartbeat_timeout_ms) {
    triggerEstop(EstopSource::INTER_BOARD, "Inter-board heartbeat timeout");
  }
}

SafetyCoordinator& SafetyCoordinator::getInstance() {
  static SafetyCoordinator instance;
  return instance;
}

bool SafetyCoordinator::isEstopActive() const {
  return current_state_ == SafetyState::EMERGENCY_STOP;
}

void SafetyCoordinator::loop() {
  if (millis() - last_estop_check_time_ms_ > config_.estop_check_interval_ms) {
    checkEstopConditions();
    last_estop_check_time_ms_ = millis();
  }

  if (config_.enable_inter_board_safety) {
    sendHeartbeat();
  }

  updateSafetyState();
}

const char* SafetyCoordinator::name() const { return "SafetyCoordinator"; }

void SafetyCoordinator::sendHeartbeat() {
  digitalWrite(config_.inter_board_output_pin, HIGH);
  delayMicroseconds(100);
  digitalWrite(config_.inter_board_output_pin, LOW);
  last_heartbeat_time_ms_ = millis();
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
}

void SafetyCoordinator::triggerEstop(EstopSource source,
                                     const String& description) {
  for (uint8_t i = 0; i < estop_condition_count_; ++i) {
    if (estop_conditions_[i].source == source) {
      estop_conditions_[i].active = true;
      estop_conditions_[i].description = description;
      updateSafetyState();
      return;
    }
  }

  if (estop_condition_count_ < 10) {
    estop_conditions_[estop_condition_count_].source = source;
    estop_conditions_[estop_condition_count_].active = true;
    estop_conditions_[estop_condition_count_].description = description;
    estop_condition_count_++;
    updateSafetyState();
  }
}

void SafetyCoordinator::updateSafetyState() {
  bool estop_active = false;
  for (uint8_t i = 0; i < estop_condition_count_; ++i) {
    if (estop_conditions_[i].active) {
      estop_active = true;
      break;
    }
  }

  if (estop_active) {
    current_state_ = SafetyState::EMERGENCY_STOP;
    digitalWrite(config_.estop_output_pin, HIGH);
  } else {
    current_state_ = SafetyState::NORMAL;
    digitalWrite(config_.estop_output_pin, LOW);
  }
}

}  // namespace sigyn_teensy
