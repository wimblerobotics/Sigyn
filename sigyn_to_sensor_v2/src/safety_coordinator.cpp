/**
 * @file safety_coordinator.cpp
 * @brief Implementation of safety coordination ROS2 node for TeensyV2 system
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/safety_coordinator.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace sigyn_to_sensor_v2 {

SafetyCoordinator::SafetyCoordinator() : Node("safety_coordinator"), global_estop_active_(false) {
  // Initialize parameters
  InitializeParameters();

  // Create publishers
  global_estop_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "teensy_v2/global_estop", 10);
  
  safety_diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "teensy_v2/safety_diagnostics", 10);
  
  safety_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "teensy_v2/safety_status", 10);

  // Create services
  emergency_stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "teensy_v2/emergency_stop",
    std::bind(&SafetyCoordinator::EmergencyStopCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  reset_estop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "teensy_v2/reset_estop",
    std::bind(&SafetyCoordinator::ResetEstopCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Create safety check timer
  safety_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2Hz safety checks
    std::bind(&SafetyCoordinator::SafetyCheckTimer, this));

  // Parameter change callback
  auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    this->ParameterCallback(event);
  };
  auto param_sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10, param_callback);

  last_global_estop_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "SafetyCoordinator node initialized");
}

void SafetyCoordinator::InitializeParameters() {
  // Declare parameters with default values
  this->declare_parameter("auto_reset_enabled", false);
  this->declare_parameter("safety_timeout_seconds", 2.0);
  this->declare_parameter("global_safety_level", 2);  // ERROR level triggers global action

  // Get parameter values
  auto_reset_enabled_ = this->get_parameter("auto_reset_enabled").as_bool();
  safety_timeout_seconds_ = this->get_parameter("safety_timeout_seconds").as_double();
  global_safety_level_ = this->get_parameter("global_safety_level").as_int();

  RCLCPP_INFO(this->get_logger(), "Safety coordination parameters initialized");
}

void SafetyCoordinator::ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  // Handle parameter updates
  for (const auto& changed_param : event->changed_parameters) {
    if (changed_param.name == "auto_reset_enabled") {
      auto_reset_enabled_ = changed_param.value.bool_value;
    } else if (changed_param.name == "safety_timeout_seconds") {
      safety_timeout_seconds_ = changed_param.value.double_value;
    } else if (changed_param.name == "global_safety_level") {
      global_safety_level_ = changed_param.value.integer_value;
    }
  }
}

void SafetyCoordinator::ProcessSafetyStatus(uint8_t board_id, bool estop_active, 
                                           uint8_t safety_level, const std::string& safety_message) {
  // Update safety state for this board
  auto& state = board_safety_states_[board_id];
  bool estop_changed = (state.estop_active != estop_active);
  
  state.estop_active = estop_active;
  state.safety_level = safety_level;
  state.safety_message = safety_message;
  state.last_update = this->now();
  state.data_valid = true;
  state.communication_timeout = false;

  // Log significant safety events
  if (estop_changed) {
    RCLCPP_WARN(this->get_logger(), 
      "Board %d E-stop status changed: %s", 
      board_id, estop_active ? "ACTIVE" : "INACTIVE");
  }

  if (safety_level >= 2) {  // ERROR or FATAL
    RCLCPP_ERROR(this->get_logger(), 
      "Board %d safety issue [level %d]: %s", 
      board_id, safety_level, safety_message.c_str());
  }

  // Update global E-stop state
  UpdateGlobalEstopState();

  RCLCPP_DEBUG(this->get_logger(), 
    "Safety status from board %d: estop=%s, level=%d, msg=%s",
    board_id, estop_active ? "true" : "false", safety_level, safety_message.c_str());
}

void SafetyCoordinator::ProcessSafetyCoordination(uint8_t source_board, uint8_t target_board, 
                                                 const std::string& command) {
  RCLCPP_INFO(this->get_logger(), 
    "Safety coordination: board %d -> %s%d: %s", 
    source_board, 
    target_board == 0 ? "ALL" : "board ", 
    target_board == 0 ? 0 : target_board,
    command.c_str());

  // Handle specific coordination commands
  if (command == "EMERGENCY_STOP") {
    RCLCPP_ERROR(this->get_logger(), 
      "Emergency stop commanded by board %d", source_board);
    // Force global E-stop activation
    global_estop_active_ = true;
    last_global_estop_time_ = this->now();
    
    // Publish immediately
    auto estop_msg = std_msgs::msg::Bool();
    estop_msg.data = true;
    global_estop_pub_->publish(estop_msg);
  }

  // Publish coordination message for other nodes
  auto coord_msg = std_msgs::msg::String();
  coord_msg.data = "SRC:" + std::to_string(source_board) + 
                   ",TGT:" + std::to_string(target_board) + 
                   ",CMD:" + command;
  safety_status_pub_->publish(coord_msg);
}

void SafetyCoordinator::SafetyCheckTimer() {
  // Check for communication timeouts
  auto current_time = this->now();
  bool timeout_detected = false;

  for (auto& [board_id, state] : board_safety_states_) {
    if (state.data_valid && IsCommTimeoutExpired(state)) {
      if (!state.communication_timeout) {
        RCLCPP_ERROR(this->get_logger(), 
          "Communication timeout with board %d", board_id);
        state.communication_timeout = true;
        timeout_detected = true;
      }
    }
  }

  // If any board has timed out, trigger E-stop
  if (timeout_detected) {
    global_estop_active_ = true;
    last_global_estop_time_ = current_time;
  }

  // Update global E-stop state
  UpdateGlobalEstopState();

  // Publish diagnostics
  PublishSafetyDiagnostics();

  // Publish safety status
  auto status_msg = std_msgs::msg::String();
  status_msg.data = GenerateSafetyStatusMessage();
  safety_status_pub_->publish(status_msg);
}

void SafetyCoordinator::UpdateGlobalEstopState() {
  bool any_board_estop = false;
  bool any_critical_safety = false;

  // Check all boards for E-stop or critical safety issues
  for (const auto& [board_id, state] : board_safety_states_) {
    if (state.data_valid) {
      if (state.estop_active || state.communication_timeout) {
        any_board_estop = true;
      }
      if (state.safety_level >= global_safety_level_) {
        any_critical_safety = true;
      }
    }
  }

  bool previous_global_estop = global_estop_active_;
  global_estop_active_ = any_board_estop || any_critical_safety;

  // Publish global E-stop status if changed
  if (global_estop_active_ != previous_global_estop) {
    auto estop_msg = std_msgs::msg::Bool();
    estop_msg.data = global_estop_active_;
    global_estop_pub_->publish(estop_msg);

    if (global_estop_active_) {
      last_global_estop_time_ = this->now();
      RCLCPP_ERROR(this->get_logger(), "GLOBAL E-STOP ACTIVATED");
    } else {
      RCLCPP_INFO(this->get_logger(), "Global E-stop deactivated");
    }
  }
}

void SafetyCoordinator::PublishSafetyDiagnostics() {
  auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
  diag_array.header.stamp = this->now();

  // Global safety status
  auto global_status = diagnostic_msgs::msg::DiagnosticStatus();
  global_status.name = "teensy_v2_global_safety";
  global_status.level = global_estop_active_ ? 
    diagnostic_msgs::msg::DiagnosticStatus::ERROR : 
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  global_status.message = global_estop_active_ ? 
    "Global E-stop active" : "System operational";

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "global_estop";
  kv.value = global_estop_active_ ? "true" : "false";
  global_status.values.push_back(kv);

  kv.key = "active_boards";
  kv.value = std::to_string(board_safety_states_.size());
  global_status.values.push_back(kv);

  kv.key = "system_safety_level";
  kv.value = std::to_string(GetSystemSafetyLevel());
  global_status.values.push_back(kv);

  diag_array.status.push_back(global_status);

  // Per-board safety status
  for (const auto& [board_id, state] : board_safety_states_) {
    if (!state.data_valid) continue;

    auto board_status = diagnostic_msgs::msg::DiagnosticStatus();
    board_status.name = "teensy_v2_safety_board_" + std::to_string(board_id);
    
    if (state.communication_timeout) {
      board_status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      board_status.message = "Communication timeout";
    } else if (state.estop_active) {
      board_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      board_status.message = "E-stop active: " + state.safety_message;
    } else {
      switch (state.safety_level) {
        case 0: 
          board_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
          board_status.message = "OK";
          break;
        case 1: 
          board_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          board_status.message = "Warning: " + state.safety_message;
          break;
        case 2: 
          board_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          board_status.message = "Error: " + state.safety_message;
          break;
        default: 
          board_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
          board_status.message = "Fatal: " + state.safety_message;
          break;
      }
    }

    kv.key = "estop_active";
    kv.value = state.estop_active ? "true" : "false";
    board_status.values.push_back(kv);

    kv.key = "safety_level";
    kv.value = std::to_string(state.safety_level);
    board_status.values.push_back(kv);

    diag_array.status.push_back(board_status);
  }

  safety_diagnostics_pub_->publish(diag_array);
}

void SafetyCoordinator::EmergencyStopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;  // Unused parameter
  
  global_estop_active_ = true;
  last_global_estop_time_ = this->now();
  
  auto estop_msg = std_msgs::msg::Bool();
  estop_msg.data = true;
  global_estop_pub_->publish(estop_msg);

  response->success = true;
  response->message = "Emergency stop activated";

  RCLCPP_ERROR(this->get_logger(), "Emergency stop activated via service call");
}

void SafetyCoordinator::ResetEstopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;  // Unused parameter

  // Check if conditions allow reset
  bool can_reset = true;
  std::string reason = "";

  // Check if any boards still have active E-stops or critical issues
  for (const auto& [board_id, state] : board_safety_states_) {
    if (state.data_valid) {
      if (state.estop_active) {
        can_reset = false;
        reason = "Board " + std::to_string(board_id) + " E-stop still active";
        break;
      }
      if (state.communication_timeout) {
        can_reset = false;
        reason = "Board " + std::to_string(board_id) + " communication timeout";
        break;
      }
      if (state.safety_level >= global_safety_level_) {
        can_reset = false;
        reason = "Board " + std::to_string(board_id) + " critical safety issue";
        break;
      }
    }
  }

  if (can_reset || auto_reset_enabled_) {
    global_estop_active_ = false;
    
    auto estop_msg = std_msgs::msg::Bool();
    estop_msg.data = false;
    global_estop_pub_->publish(estop_msg);

    response->success = true;
    response->message = "E-stop reset successful";

    RCLCPP_INFO(this->get_logger(), "E-stop reset via service call");
  } else {
    response->success = false;
    response->message = "Cannot reset E-stop: " + reason;

    RCLCPP_WARN(this->get_logger(), "E-stop reset denied: %s", reason.c_str());
  }
}

bool SafetyCoordinator::IsCommTimeoutExpired(const BoardSafetyState& state) {
  auto current_time = this->now();
  auto elapsed = (current_time - state.last_update).seconds();
  return elapsed > safety_timeout_seconds_;
}

uint8_t SafetyCoordinator::GetSystemSafetyLevel() {
  uint8_t max_level = 0;
  for (const auto& [board_id, state] : board_safety_states_) {
    if (state.data_valid && state.safety_level > max_level) {
      max_level = state.safety_level;
    }
  }
  return max_level;
}

std::string SafetyCoordinator::GenerateSafetyStatusMessage() {
  if (global_estop_active_) {
    return "GLOBAL E-STOP ACTIVE - System halted";
  }

  uint8_t system_level = GetSystemSafetyLevel();
  size_t active_boards = 0;
  size_t timeout_boards = 0;

  for (const auto& [board_id, state] : board_safety_states_) {
    if (state.data_valid) {
      active_boards++;
      if (state.communication_timeout) {
        timeout_boards++;
      }
    }
  }

  std::string status = "System OK - " + std::to_string(active_boards) + " boards active";
  
  if (timeout_boards > 0) {
    status += ", " + std::to_string(timeout_boards) + " timeouts";
  }
  
  if (system_level > 0) {
    status += ", safety level " + std::to_string(system_level);
  }

  return status;
}

}  // namespace sigyn_to_sensor_v2
