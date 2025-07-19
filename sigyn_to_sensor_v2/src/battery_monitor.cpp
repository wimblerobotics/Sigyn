/**
 * @file battery_monitor.cpp
 * @brief Implementation of battery monitoring ROS2 node for TeensyV2 system
 *
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/battery_monitor.h"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace sigyn_to_sensor_v2 {

BatteryMonitor::BatteryMonitor() : Node("battery_monitor") {
  // Initialize parameters
  InitializeParameters();

  // Create publishers
  battery_pub_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("teensy_v2/battery_state", 10);

  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "teensy_v2/battery_diagnostics", 10);

  // Create diagnostics timer
  diagnostics_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&BatteryMonitor::PublishDiagnostics, this));

  // Parameter change callback
  auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    this->ParameterCallback(event);
  };
  auto param_sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
      "/parameter_events", 10, param_callback);

  RCLCPP_INFO(this->get_logger(), "BatteryMonitor node initialized");
}

void BatteryMonitor::InitializeParameters() {
  // Declare parameters with default values
  this->declare_parameter("voltage_warn_threshold", 11.5);
  this->declare_parameter("voltage_error_threshold", 10.5);
  this->declare_parameter("current_warn_threshold", 15.0);
  this->declare_parameter("temperature_warn_threshold", 45.0);
  this->declare_parameter("temperature_error_threshold", 60.0);

  // Get parameter values
  voltage_warn_threshold_ = this->get_parameter("voltage_warn_threshold").as_double();
  voltage_error_threshold_ = this->get_parameter("voltage_error_threshold").as_double();
  current_warn_threshold_ = this->get_parameter("current_warn_threshold").as_double();
  temperature_warn_threshold_ = this->get_parameter("temperature_warn_threshold").as_double();
  temperature_error_threshold_ = this->get_parameter("temperature_error_threshold").as_double();

  RCLCPP_INFO(this->get_logger(), "Battery monitoring thresholds initialized");
}

void BatteryMonitor::ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  // Handle parameter updates (implementation can be expanded)
  for (const auto& changed_param : event->changed_parameters) {
    if (changed_param.name == "voltage_warn_threshold") {
      voltage_warn_threshold_ = changed_param.value.double_value;
    } else if (changed_param.name == "voltage_error_threshold") {
      voltage_error_threshold_ = changed_param.value.double_value;
    }
    // Add other parameter updates as needed
  }
}

void BatteryMonitor::ProcessBatteryData(uint8_t board_id, float voltage, float current,
                                        float temperature, uint8_t charge_percentage) {
  // Update battery state for this board
  auto& state = battery_states_[board_id];
  state.voltage = voltage;
  state.current = current;
  state.temperature = temperature;
  state.charge_percentage = charge_percentage;
  state.last_update = this->now();
  state.data_valid = true;

  // Publish battery state message
  auto battery_msg = sensor_msgs::msg::BatteryState();
  battery_msg.header.stamp = this->now();
  battery_msg.header.frame_id = "teensy_v2_board_" + std::to_string(board_id);

  battery_msg.voltage = voltage;
  battery_msg.current = current;
  battery_msg.temperature = temperature;
  battery_msg.percentage = static_cast<float>(charge_percentage) / 100.0f;
  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  battery_msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  battery_msg.present = true;

  battery_pub_->publish(battery_msg);

  RCLCPP_DEBUG(this->get_logger(), "Battery data from board %d: %.2fV, %.2fA, %.1fC, %d%%",
               board_id, voltage, current, temperature, charge_percentage);
}

void BatteryMonitor::PublishDiagnostics() {
  auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
  diag_array.header.stamp = this->now();

  for (const auto& [board_id, state] : battery_states_) {
    if (!state.data_valid) continue;

    auto diag_status = diagnostic_msgs::msg::DiagnosticStatus();
    diag_status.name = "teensy_v2_battery_board_" + std::to_string(board_id);
    diag_status.level = CheckBatteryHealth(state);

    // Set message based on health status
    switch (diag_status.level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        diag_status.message = "Battery healthy";
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        diag_status.message = "Battery warning - check voltage/temperature";
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        diag_status.message = "Battery error - critical levels detected";
        break;
      default:
        diag_status.message = "Battery status unknown";
        break;
    }

    // Add key-value diagnostics
    diagnostic_msgs::msg::KeyValue kv;

    kv.key = "voltage";
    kv.value = std::to_string(state.voltage);
    diag_status.values.push_back(kv);

    kv.key = "current";
    kv.value = std::to_string(state.current);
    diag_status.values.push_back(kv);

    kv.key = "temperature";
    kv.value = std::to_string(state.temperature);
    diag_status.values.push_back(kv);

    kv.key = "charge_percentage";
    kv.value = std::to_string(state.charge_percentage);
    diag_status.values.push_back(kv);

    diag_array.status.push_back(diag_status);
  }

  if (!diag_array.status.empty()) {
    diagnostics_pub_->publish(diag_array);
  }
}

uint8_t BatteryMonitor::CheckBatteryHealth(const BatteryState& state) {
  // Check voltage levels
  if (state.voltage < voltage_error_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (state.voltage < voltage_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  // Check temperature levels
  if (state.temperature > temperature_error_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (state.temperature > temperature_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  // Check current levels
  if (std::abs(state.current) > current_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

uint8_t BatteryMonitor::GetPowerState(uint8_t charge_percentage) {
  if (charge_percentage > 90) {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (charge_percentage > 20) {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else {
    return sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
}

}  // namespace sigyn_to_sensor_v2
