/**
 * @file safety_coordinator.cpp
 * @brief Implementation of central safety coordination system
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "safety_coordinator.h"
#include <cmath>

namespace sigyn_teensy {

SafetyCoordinator& SafetyCoordinator::GetInstance() {
  static SafetyCoordinator instance;
  return instance;
}

SafetyCoordinator::SafetyCoordinator()
    : Module(),
      condition_count_(0),
      current_safety_state_(SafetyState::NORMAL),
      previous_safety_state_(SafetyState::NORMAL),
      hardware_estop_state_(false),
      inter_board_safety_state_(false),
      last_heartbeat_time_(0),
      last_safety_check_(0),
      last_status_report_(0),
      shutdown_time_(0),
      serial_manager_(&SerialManager::GetInstance()) {
  
  // Initialize all E-stop sources as enabled by default
  for (uint8_t i = 0; i <= static_cast<uint8_t>(EstopSource::UNKNOWN); ++i) {
    source_enabled_[i] = true;
  }
}

void SafetyCoordinator::Configure(const SafetyConfig& config) {
  config_ = config;
}

void SafetyCoordinator::setup() {
  Serial.println("SafetyCoordinator: Initializing safety system");
  
  InitializeHardware();
  
  // Initialize safety state
  current_safety_state_ = SafetyState::NORMAL;
  previous_safety_state_ = SafetyState::NORMAL;
  
  // Initialize timing
  uint32_t current_time = millis();
  last_safety_check_ = current_time;
  last_status_report_ = current_time;
  last_heartbeat_time_ = current_time;
  
  Serial.println("SafetyCoordinator: Initialization complete");
}

void SafetyCoordinator::loop() {
  uint32_t current_time = millis();
  
  // Check safety conditions at configured interval
  if (current_time - last_safety_check_ >= config_.estop_check_interval_ms) {
    last_safety_check_ = current_time;
    
    CheckHardwareEstop();
    
    if (config_.enable_inter_board_safety) {
      CheckInterBoardSafety();
    }
    
    UpdateSafetyState();
  }
  
  // Send status updates
  if (current_time - last_status_report_ >= 1000) {  // Every 1 second
    last_status_report_ = current_time;
    SendSafetyStatusMessage();
  }
  
  // Check for automatic recovery opportunities
  if (config_.enable_auto_recovery && IsEstopActive()) {
    AttemptAutoRecovery();
  }
}

bool SafetyCoordinator::IsUnsafe() {
  return (current_safety_state_ == SafetyState::EMERGENCY_STOP ||
          current_safety_state_ == SafetyState::SYSTEM_SHUTDOWN);
}

void SafetyCoordinator::ProcessMessage(const String& message) {
  if (message.startsWith("ESTOP:")) {
    String estop_data = message.substring(6);
    
    // Parse E-stop command
    if (estop_data.indexOf("trigger=") != -1) {
      // Software E-stop trigger
      int reason_start = estop_data.indexOf("reason=");
      String reason = "Software E-stop";
      if (reason_start != -1) {
        int reason_end = estop_data.indexOf(',', reason_start);
        if (reason_end == -1) reason_end = estop_data.length();
        reason = estop_data.substring(reason_start + 7, reason_end);
      }
      
      TriggerEstop(EstopSource::SOFTWARE_COMMAND, reason, NAN, true);
    } else if (estop_data.indexOf("reset=") != -1) {
      // Manual reset command
      int source_start = estop_data.indexOf("source=");
      if (source_start != -1) {
        int source_end = estop_data.indexOf(',', source_start);
        if (source_end == -1) source_end = estop_data.length();
        String source_str = estop_data.substring(source_start + 7, source_end);
        
        // Convert string to EstopSource enum
        EstopSource source = EstopSource::UNKNOWN;
        if (source_str == "software") source = EstopSource::SOFTWARE_COMMAND;
        else if (source_str == "battery") source = EstopSource::BATTERY_LOW_VOLTAGE;
        else if (source_str == "motor") source = EstopSource::MOTOR_OVERCURRENT;
        // Add more source mappings as needed
        
        ManualReset(source, true);
      }
    }
  } else if (message.startsWith("SAFETY_CONFIG:")) {
    String config_data = message.substring(14);
    
    // Parse configuration updates
    int start = 0;
    while (start < config_data.length()) {
      int equals_pos = config_data.indexOf('=', start);
      int comma_pos = config_data.indexOf(',', start);
      
      if (equals_pos == -1) break;
      if (comma_pos == -1) comma_pos = config_data.length();
      
      String key = config_data.substring(start, equals_pos);
      String value = config_data.substring(equals_pos + 1, comma_pos);
      
      // Update configuration based on key
      if (key == "auto_recovery") {
        config_.enable_auto_recovery = (value == "true");
      } else if (key == "recovery_delay") {
        config_.recovery_delay_ms = value.toInt();
      } else if (key == "max_violations") {
        config_.max_consecutive_violations = value.toInt();
      }
      
      start = comma_pos + 1;
    }
    
    Serial.println("SafetyCoordinator: Configuration updated");
  }
}

void SafetyCoordinator::TriggerEstop(EstopSource source, 
                                     const String& description, 
                                     float trigger_value,
                                     bool requires_manual_reset) {
  if (!source_enabled_[static_cast<uint8_t>(source)]) {
    return;  // Source is disabled
  }
  
  EstopCondition* condition = FindCondition(source);
  if (!condition) {
    condition = CreateCondition(source);
    if (!condition) {
      Serial.println("SafetyCoordinator: ERROR - No space for new E-stop condition");
      return;
    }
  }
  
  // Update condition if not already active
  if (!condition->active) {
    condition->source = source;
    condition->active = true;
    condition->requires_manual_reset = requires_manual_reset;
    condition->activation_time = millis();
    condition->description = description;
    condition->trigger_value = trigger_value;
    
    Serial.println("SafetyCoordinator: E-stop triggered - " + EstopSourceToString(source) + ": " + description);
    
    SendEstopMessage(*condition, true);
    UpdateSafetyState();
  }
  
  condition->last_check_time = millis();
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
  
  Serial.println("SafetyCoordinator: E-stop cleared - " + EstopSourceToString(source));
  
  SendEstopMessage(*condition, false);
  UpdateSafetyState();
  
  return true;
}

bool SafetyCoordinator::ManualReset(EstopSource source, bool force_reset) {
  EstopCondition* condition = FindCondition(source);
  if (!condition || !condition->active) {
    return false;  // Condition not found or not active
  }
  
  if (!force_reset && !condition->requires_manual_reset) {
    return false;  // Manual reset not required, use automatic recovery
  }
  
  condition->active = false;
  condition->requires_manual_reset = false;
  condition->last_check_time = millis();
  
  Serial.println("SafetyCoordinator: Manual reset - " + EstopSourceToString(source));
  
  SendEstopMessage(*condition, false);
  UpdateSafetyState();
  
  return true;
}

uint8_t SafetyCoordinator::AttemptAutoRecovery() {
  uint8_t cleared_count = 0;
  
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (estop_conditions_[i].active && !estop_conditions_[i].requires_manual_reset) {
      if (ClearEstop(estop_conditions_[i].source)) {
        cleared_count++;
      }
    }
  }
  
  return cleared_count;
}

bool SafetyCoordinator::IsEstopActive() const {
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (estop_conditions_[i].active) {
      return true;
    }
  }
  return false;
}

uint8_t SafetyCoordinator::GetActiveConditions(EstopCondition* active_conditions, uint8_t max_conditions) const {
  uint8_t count = 0;
  
  for (uint8_t i = 0; i < condition_count_ && count < max_conditions; ++i) {
    if (estop_conditions_[i].active) {
      active_conditions[count] = estop_conditions_[i];
      count++;
    }
  }
  
  return count;
}

String SafetyCoordinator::GetSafetyStatusDescription() const {
  String status = SafetyStateToString(current_safety_state_);
  
  if (IsEstopActive()) {
    status += " (";
    bool first = true;
    for (uint8_t i = 0; i < condition_count_; ++i) {
      if (estop_conditions_[i].active) {
        if (!first) status += ", ";
        status += EstopSourceToString(estop_conditions_[i].source);
        first = false;
      }
    }
    status += ")";
  }
  
  return status;
}

void SafetyCoordinator::ForceShutdown(const String& reason) {
  current_safety_state_ = SafetyState::SYSTEM_SHUTDOWN;
  shutdown_time_ = millis();
  
  Serial.println("SafetyCoordinator: SYSTEM SHUTDOWN - " + reason);
  
  // Send immediate shutdown notification
  if (serial_manager_) {
    String shutdown_msg = "reason=" + reason + ",time=" + String(shutdown_time_);
    serial_manager_->SendMessage("SHUTDOWN", shutdown_msg.c_str());
  }
  
  // Set hardware outputs to safe state
  digitalWrite(config_.estop_output_pin, HIGH);  // Assert E-stop output
  if (config_.enable_inter_board_safety) {
    digitalWrite(config_.inter_board_output_pin, HIGH);  // Signal other boards
  }
}

void SafetyCoordinator::SetEstopSourceEnabled(EstopSource source, bool enabled) {
  uint8_t index = static_cast<uint8_t>(source);
  if (index <= static_cast<uint8_t>(EstopSource::UNKNOWN)) {
    source_enabled_[index] = enabled;
  }
}

bool SafetyCoordinator::IsHardwareEstopPressed() const {
  return hardware_estop_state_;
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
  
  Serial.println("SafetyCoordinator: Hardware pins initialized");
}

void SafetyCoordinator::CheckHardwareEstop() {
  bool current_estop_state = !digitalRead(config_.hardware_estop_pin);  // Active low
  
  if (current_estop_state != hardware_estop_state_) {
    hardware_estop_state_ = current_estop_state;
    
    if (hardware_estop_state_) {
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
  
  if (current_inter_board_state != inter_board_safety_state_) {
    inter_board_safety_state_ = current_inter_board_state;
    
    if (inter_board_safety_state_) {
      TriggerEstop(EstopSource::INTER_BOARD, "Safety signal from other board", NAN, false);
    } else {
      ClearEstop(EstopSource::INTER_BOARD);
    }
  }
  
  // Update heartbeat
  last_heartbeat_time_ = millis();
}

void SafetyCoordinator::UpdateSafetyState() {
  previous_safety_state_ = current_safety_state_;
  
  if (current_safety_state_ == SafetyState::SYSTEM_SHUTDOWN) {
    return;  // Once in shutdown, stay in shutdown
  }
  
  if (IsEstopActive()) {
    current_safety_state_ = SafetyState::EMERGENCY_STOP;
  } else {
    // Determine safety level based on system health
    // This could be expanded to check performance violations, sensor health, etc.
    current_safety_state_ = SafetyState::NORMAL;
  }
  
  // Update hardware outputs
  if (current_safety_state_ == SafetyState::EMERGENCY_STOP || 
      current_safety_state_ == SafetyState::SYSTEM_SHUTDOWN) {
    digitalWrite(config_.estop_output_pin, HIGH);  // Assert E-stop
    if (config_.enable_inter_board_safety) {
      digitalWrite(config_.inter_board_output_pin, HIGH);  // Signal other boards
    }
  } else {
    digitalWrite(config_.estop_output_pin, LOW);   // Release E-stop
    if (config_.enable_inter_board_safety) {
      digitalWrite(config_.inter_board_output_pin, LOW);   // Signal normal operation
    }
  }
  
  // Send notification if state changed
  if (current_safety_state_ != previous_safety_state_) {
    Serial.println("SafetyCoordinator: Safety state changed from " + 
                   SafetyStateToString(previous_safety_state_) + " to " + 
                   SafetyStateToString(current_safety_state_));
  }
}

void SafetyCoordinator::SendSafetyStatusMessage() {
  if (!serial_manager_) return;
  
  String message = "state=" + SafetyStateToString(current_safety_state_);
  message += ",hw_estop=" + String(hardware_estop_state_ ? "true" : "false");
  
  if (config_.enable_inter_board_safety) {
    message += ",inter_board=" + String(inter_board_safety_state_ ? "true" : "false");
  }
  
  message += ",active_conditions=" + String(IsEstopActive() ? "true" : "false");
  
  // Add list of active E-stop sources
  if (IsEstopActive()) {
    message += ",sources=";
    bool first = true;
    for (uint8_t i = 0; i < condition_count_; ++i) {
      if (estop_conditions_[i].active) {
        if (!first) message += "+";
        message += EstopSourceToString(estop_conditions_[i].source);
        first = false;
      }
    }
  }
  
  serial_manager_->SendMessage("SAFETY", message.c_str());
}

void SafetyCoordinator::SendEstopMessage(const EstopCondition& condition, bool activated) {
  if (!serial_manager_) return;
  
  String message = "active=" + String(activated ? "true" : "false");
  message += ",source=" + EstopSourceToString(condition.source);
  message += ",reason=" + condition.description;
  
  if (!isnan(condition.trigger_value)) {
    message += ",value=" + String(condition.trigger_value, 2);
  }
  
  message += ",manual_reset=" + String(condition.requires_manual_reset ? "true" : "false");
  message += ",time=" + String(condition.activation_time);
  
  serial_manager_->SendMessage("ESTOP", message.c_str());
}

EstopCondition* SafetyCoordinator::FindCondition(EstopSource source) {
  for (uint8_t i = 0; i < condition_count_; ++i) {
    if (estop_conditions_[i].source == source) {
      return &estop_conditions_[i];
    }
  }
  return nullptr;
}

EstopCondition* SafetyCoordinator::CreateCondition(EstopSource source) {
  if (condition_count_ >= kMaxEstopConditions) {
    return nullptr;  // No space available
  }
  
  EstopCondition* condition = &estop_conditions_[condition_count_];
  condition->source = source;
  condition->active = false;
  condition->requires_manual_reset = false;
  condition->activation_time = 0;
  condition->last_check_time = 0;
  condition->description = "";
  condition->trigger_value = NAN;
  
  condition_count_++;
  return condition;
}

String SafetyCoordinator::EstopSourceToString(EstopSource source) const {
  switch (source) {
    case EstopSource::HARDWARE_BUTTON: return "HARDWARE";
    case EstopSource::SOFTWARE_COMMAND: return "SOFTWARE";
    case EstopSource::PERFORMANCE: return "PERFORMANCE";
    case EstopSource::BATTERY_LOW_VOLTAGE: return "BATTERY_VOLTAGE";
    case EstopSource::BATTERY_HIGH_CURRENT: return "BATTERY_CURRENT";
    case EstopSource::MOTOR_OVERCURRENT: return "MOTOR_CURRENT";
    case EstopSource::MOTOR_RUNAWAY: return "MOTOR_RUNAWAY";
    case EstopSource::SENSOR_FAILURE: return "SENSOR";
    case EstopSource::INTER_BOARD: return "INTER_BOARD";
    default: return "UNKNOWN";
  }
}

String SafetyCoordinator::SafetyStateToString(SafetyState state) const {
  switch (state) {
    case SafetyState::NORMAL: return "NORMAL";
    case SafetyState::WARNING: return "WARNING";
    case SafetyState::DEGRADED: return "DEGRADED";
    case SafetyState::EMERGENCY_STOP: return "ESTOP";
    case SafetyState::SYSTEM_SHUTDOWN: return "SHUTDOWN";
    default: return "UNKNOWN";
  }
}

}  // namespace sigyn_teensy
