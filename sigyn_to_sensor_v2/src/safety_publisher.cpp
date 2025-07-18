/**
 * @file safety_publisher.cpp
 * @brief Implementation of safety message publisher for TeensyV2 system
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/safety_publisher.h"
#include <std_msgs/msg/string.hpp>

namespace sigyn_to_sensor_v2 {

SafetyPublisher::SafetyPublisher(rclcpp::Node::SharedPtr node) : node_(node) {
  // Create publishers
  estop_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "teensy_v2/estop_status", 10);
  
  diagnostics_pub_ = node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "teensy_v2/diagnostics", 10);
  
  safety_coordination_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "teensy_v2/safety_coordination", 10);

  RCLCPP_INFO(node_->get_logger(), "SafetyPublisher initialized");
}

void SafetyPublisher::PublishEstopStatus(bool estop_active, uint8_t board_id) {
  auto msg = std_msgs::msg::Bool();
  msg.data = estop_active;
  
  estop_pub_->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), 
    "E-stop status from board %d: %s", 
    board_id, estop_active ? "ACTIVE" : "INACTIVE");
}

void SafetyPublisher::PublishSafetyDiagnostics(uint8_t board_id, uint8_t safety_level, 
                                              const std::string& message) {
  auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
  diag_array.header.stamp = node_->now();
  
  auto diag_status = diagnostic_msgs::msg::DiagnosticStatus();
  diag_status.name = "teensy_v2_board_" + std::to_string(board_id);
  diag_status.level = ConvertSafetyLevel(safety_level);
  diag_status.message = message;
  
  // Add key-value pairs
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "board_id";
  kv.value = std::to_string(board_id);
  diag_status.values.push_back(kv);
  
  kv.key = "safety_level";
  kv.value = std::to_string(safety_level);
  diag_status.values.push_back(kv);
  
  diag_array.status.push_back(diag_status);
  diagnostics_pub_->publish(diag_array);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Safety diagnostics from board %d [level %d]: %s", 
    board_id, safety_level, message.c_str());
}

void SafetyPublisher::PublishSafetyCoordination(uint8_t source_board, uint8_t target_board, 
                                               const std::string& command) {
  auto msg = std_msgs::msg::String();
  msg.data = "SRC:" + std::to_string(source_board) + 
             ",TGT:" + std::to_string(target_board) + 
             ",CMD:" + command;
  
  safety_coordination_pub_->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), 
    "Safety coordination: board %d -> %s%d: %s", 
    source_board, 
    target_board == 0 ? "ALL" : "board ", 
    target_board == 0 ? 0 : target_board,
    command.c_str());
}

uint8_t SafetyPublisher::ConvertSafetyLevel(uint8_t safety_level) {
  switch (safety_level) {
    case 0: return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case 1: return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case 2: return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    case 3: return diagnostic_msgs::msg::DiagnosticStatus::STALE;
    default: return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
}

}  // namespace sigyn_to_sensor_v2
