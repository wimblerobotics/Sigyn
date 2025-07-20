/**
 * @file safety_coordinator.cpp
 * @brief Implementation of central safety coordination system
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#include "safety_coordinator.h"
#include <cmath>

namespace sigyn_teensy {

// Static member definition
SafetyCoordinator* SafetyCoordinator::instance_ = nullptr;

uint8_t SafetyCoordinator::AttemptAutoRecovery() {
  uint8_t cleared_count = 0;
  
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (conditions_[i].active && !conditions_[i].requires_manual_reset) {
      if (ClearEstop(conditions_[i].source)) {
        cleared_count++;
      }
    }
  }
  
  return cleared_count;
}

void SafetyCoordinator::CheckHardwareEstop() {
  bool current_estop_state = !digitalRead(config_.hardware_estop_pin);  // Active low
  
  if (current_estop_state != IsHardwareEstopPressed()) {
    if (current_estop_state) {
      TriggerEstop(EstopSource::HARDWARE_BUTTON, "Hardware E-stop button pressed", NAN, true);
    } else {
      // Hardware E-stop released - allow manual reset
      EstopCondition* condition = FindCondition(EstopSource::HARDWARE_BUTTON);
      if (condition && condition->active) {
        condition->requires_manual_reset = false;  // Allow automatic recovery
      }
    }
  }
}

void SafetyCoordinator::CheckInterBoardSafety() {
  bool current_inter_board_state = !digitalRead(config_.inter_board_input_pin);  // Active low
  
  if (current_inter_board_state != inter_board_estop_active_) {
    inter_board_estop_active_ = current_inter_board_state;
    
    if (inter_board_estop_active_) {
      TriggerEstop(EstopSource::INTER_BOARD, "Safety signal from other board", NAN, false);
    } else {
      ClearEstop(EstopSource::INTER_BOARD);
    }
  }
  
  // Update heartbeat
  last_heartbeat_received_ms_ = millis();
}

bool SafetyCoordinator::ClearEstop(EstopSource source) {
  EstopCondition* condition = FindCondition(source);
  if (!condition || !condition->active) {
    return false;  // Condition not found or not active
  }
  
  if (condition->requires_manual_reset) {
    return false;  // Manual reset required
  }
  
  uint32_t current_time = millis();
  if (current_time - condition->activation_time < config_.recovery_delay_ms) {
    return false;  // Not enough time has passed for recovery
  }
  
  condition->active = false;
  condition->last_check_time = current_time;
  
  SendEstopMessage(*condition, false);
  UpdateSafetyState();
  
  return true;
}

void SafetyCoordinator::Configure(const SafetyConfig& config) {
  config_ = config;
}

EstopCondition* SafetyCoordinator::FindCondition(EstopSource source) {
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (conditions_[i].source == source) {
      return &conditions_[i];
    }
  }
  return nullptr;
}

void SafetyCoordinator::ForceShutdown(const String& reason) {
  current_safety_state_ = SafetyState::SYSTEM_SHUTDOWN;
  
  // Send immediate shutdown notification
  String shutdown_msg = "reason=" + reason;
  SerialManager::GetInstance().SendMessage("SHUTDOWN", shutdown_msg.c_str());
  
  // Set hardware outputs to safe state
  digitalWrite(config_.estop_output_pin, HIGH);  // Assert E-stop output
  if (config_.enable_inter_board_safety) {
    digitalWrite(config_.inter_board_output_pin, HIGH);  // Signal other boards
  }
}

uint8_t SafetyCoordinator::GetActiveConditions(EstopCondition* active_conditions, uint8_t max_conditions) const {
  uint8_t count = 0;
  
  for (uint8_t i = 0; i < condition_count_ && count < max_conditions; ++i) {
    if (conditions_[i].active) {
      active_conditions[count] = conditions_[i];
      count++;
    }
  }
  
  return count;
}

SafetyCoordinator& SafetyCoordinator::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new SafetyCoordinator();
  }
  return *instance_;
}

String SafetyCoordinator::GetSafetyStatusDescription() const {
  String status; // Simplified from original
  
  if (IsEstopActive()) {
    status += " (";
    bool first = true;
    for (uint8_t i = 0; i < condition_count_; ++i) {
      if (conditions_[i].active) {
        if (!first) status += ", ";
        // status += EstopSourceToString(conditions_[i].source); // EstopSourceToString is private
        first = false;
      }
    }
    status += ")";
  }
  
  return status;
}

void SafetyCoordinator::InitializeHardware() {
  // Configure E-stop input pin (active low with pullup)
  pinMode(config_.hardware_estop_pin, INPUT_PULLUP);
  
  // Configure E-stop output pin (active high)
  pinMode(config_.estop_output_pin, OUTPUT);
  digitalWrite(config_.estop_output_pin, LOW);  // Start in safe state
  
  if (config_.enable_inter_board_safety) {
    // Configure inter-board safety pins
    pinMode(config_.inter_board_input_pin, INPUT_PULLUP);
    pinMode(config_.inter_board_output_pin, OUTPUT);
    digitalWrite(config_.inter_board_output_pin, LOW);  // Start in safe state
  }
}

bool SafetyCoordinator::IsEstopActive() const {
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (conditions_[i].active) {
      return true;
    }
  }
  return false;
}

bool SafetyCoordinator::IsHardwareEstopPressed() const {
  return !digitalRead(config_.hardware_estop_pin);
}

bool SafetyCoordinator::IsUnsafe() {
  return (current_safety_state_ == SafetyState::EMERGENCY_STOP ||
          current_safety_state_ == SafetyState::SYSTEM_SHUTDOWN);
}

void SafetyCoordinator::loop() {
  uint32_t current_time_ms = millis();
  
  // Check safety conditions at configured interval
  if (current_time_ms - last_status_report_ms_ >= config_.estop_check_interval_ms) {
    CheckHardwareEstop();
    
    if (config_.enable_inter_board_safety) {
      CheckInterBoardSafety();
    }
    
    UpdateSafetyState();
  }
  
  // Send heartbeat to other board
  if (config_.enable_inter_board_safety &&
      (current_time_ms - last_heartbeat_sent_ms_ > 100)) { // 10Hz heartbeat
    digitalWrite(config_.inter_board_output_pin, !digitalRead(config_.inter_board_output_pin));
    last_heartbeat_sent_ms_ = current_time_ms;
  }
  
  // Send periodic status report
  if (current_time_ms - last_status_report_ms_ > 1000) { // 1Hz status report
    SendSafetyStatusMessage();
    last_status_report_ms_ = current_time_ms;
  }
}

bool SafetyCoordinator::ManualReset(EstopSource source, bool force_reset) {
  EstopCondition* condition = FindCondition(source);
  if (!condition || !condition->active) {
    return true; // Condition not active, so reset is "successful"
  }
  
  if (condition->requires_manual_reset && !force_reset) {
    return false; // Requires manual reset, and not forced
  }
  
  condition->active = false;
  condition->last_check_time = millis();
  
  SendEstopMessage(*condition, false);
  UpdateSafetyState();
  
  return true;
}

void SafetyCoordinator::ProcessMessage(const String& message) {
  // Example: "ESTOP:source=SOFTWARE,reason=user_request"
  if (message.startsWith("ESTOP:")) {
    String payload = message.substring(6);
    // Basic parsing - a real implementation would be more robust
    if (payload.indexOf("source=SOFTWARE") != -1) {
      TriggerEstop(EstopSource::SOFTWARE_COMMAND, "Software E-stop from ROS2", NAN, true);
    }
  }
}

SafetyCoordinator::SafetyCoordinator()
    : Module(),
      condition_count_(0),
      current_safety_state_(SafetyState::NORMAL),
      last_status_report_ms_(0),
      last_heartbeat_sent_ms_(0),
      last_heartbeat_received_ms_(0),
      inter_board_estop_active_(false) {
  // Initialize with default configuration
  config_ = SafetyConfig{};
  
  // Clear conditions
  for (uint8_t i = 0; i < kMaxEstopConditions; ++i) {
    conditions_[i] = EstopCondition{};
  }
}

void SafetyCoordinator::SendEstopMessage(const EstopCondition& condition, bool activated) {
  char estop_msg[128];
  snprintf(estop_msg, sizeof(estop_msg),
          "source=%d,activated=%s,desc=%s,val=%.2f,manual_reset=%s",
          static_cast<int>(condition.source),
          activated ? "true" : "false",
          condition.description.c_str(),
          condition.trigger_value,
          condition.requires_manual_reset ? "true" : "false");
  
  SerialManager::GetInstance().SendMessage("ESTOP", estop_msg);
}

void SafetyCoordinator::SendSafetyStatusMessage() {
  char status_msg[64];
  snprintf(status_msg, sizeof(status_msg),
          "state=%d,estop_active=%s,hw_estop=%s",
          static_cast<int>(current_safety_state_),
          IsEstopActive() ? "true" : "false",
          IsHardwareEstopPressed() ? "true" : "false");
  
  SerialManager::GetInstance().SendMessage("SAFETY_STATUS", status_msg);
}

void SafetyCoordinator::SetEstopSourceEnabled(EstopSource source, bool enabled) {
  // This is a placeholder. A full implementation would allow disabling
  // certain E-stop sources for testing or specific operational modes.
  // Care must be taken to ensure safety is not compromised.
}

void SafetyCoordinator::setup() {
  InitializeHardware();
  
  // Register with the module system
  
  SerialManager::GetInstance().SendMessage("INFO", "SafetyCoordinator initialized");
}

void SafetyCoordinator::TriggerEstop(EstopSource source, 
                                     const String& description, 
                                     float trigger_value,
                                     bool requires_manual_reset) {
  EstopCondition* condition = FindCondition(source);
  
  if (condition == nullptr) {
    if (condition_count_ < kMaxEstopConditions) {
      condition = &conditions_[condition_count_++];
      condition->source = source;
    } else {
      // Cannot track new condition, this is a system limitation
      return;
    }
  }

  if (!condition->active) {
    condition->active = true;
    condition->description = description;
    condition->trigger_value = trigger_value;
    condition->requires_manual_reset = requires_manual_reset;
    condition->activation_time = millis();
    
    SendEstopMessage(*condition, true);
    UpdateSafetyState();
  }
}

void SafetyCoordinator::UpdateSafetyState() {
  if (IsEstopActive()) {
    current_safety_state_ = SafetyState::EMERGENCY_STOP;
    digitalWrite(config_.estop_output_pin, HIGH); // Assert E-stop
    if (config_.enable_inter_board_safety) {
      digitalWrite(config_.inter_board_output_pin, HIGH);
    }
  } else {
    current_safety_state_ = SafetyState::NORMAL;
    digitalWrite(config_.estop_output_pin, LOW); // De-assert E-stop
    if (config_.enable_inter_board_safety) {
      digitalWrite(config_.inter_board_output_pin, LOW);
    }
  }
}

}  // namespace sigyn_teensy
