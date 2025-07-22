/**
 * @file performance_monitor.cpp
 * @brief Implementation of performance monitoring ROS2 node for TeensyV2 system
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/performance_monitor.h"
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace sigyn_to_sensor_v2 {

PerformanceMonitor::PerformanceMonitor() : Node("performance_monitor") {
  // Initialize parameters
  InitializeParameters();

  // Create publishers
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "teensy_v2/performance_diagnostics", 10);
  
  frequency_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "teensy_v2/loop_frequency", 10);
  
  // NOTE: execution_time_pub_ and memory_usage_pub_ are orphaned - ProcessPerformanceData() is never called
  // TODO: Remove these publishers or implement proper data flow from teensy_bridge
  // execution_time_pub_ = this->create_publisher<std_msgs::msg::Int32>(
  //   "teensy_v2/execution_time", 10);
  
  // memory_usage_pub_ = this->create_publisher<std_msgs::msg::Int32>(
  //   "teensy_v2/memory_usage", 10);

  // Create diagnostics timer
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    std::bind(&PerformanceMonitor::PublishDiagnostics, this));

  // Parameter change callback
  auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
    this->ParameterCallback(event);
  };
  auto param_sub = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", 10, param_callback);

  RCLCPP_INFO(this->get_logger(), "PerformanceMonitor node initialized");
}

void PerformanceMonitor::InitializeParameters() {
  // Declare parameters with default values
  this->declare_parameter("frequency_warn_threshold", 90.0);   // Hz
  this->declare_parameter("frequency_error_threshold", 70.0);  // Hz
  this->declare_parameter("execution_time_warn_threshold", 8000);   // us (8ms)
  this->declare_parameter("execution_time_error_threshold", 9500);  // us (9.5ms)
  this->declare_parameter("memory_warn_threshold", 80);  // %
  this->declare_parameter("memory_error_threshold", 95); // %

  // Get parameter values
  frequency_warn_threshold_ = this->get_parameter("frequency_warn_threshold").as_double();
  frequency_error_threshold_ = this->get_parameter("frequency_error_threshold").as_double();
  execution_time_warn_threshold_ = this->get_parameter("execution_time_warn_threshold").as_int();
  execution_time_error_threshold_ = this->get_parameter("execution_time_error_threshold").as_int();
  memory_warn_threshold_ = this->get_parameter("memory_warn_threshold").as_int();
  memory_error_threshold_ = this->get_parameter("memory_error_threshold").as_int();

  RCLCPP_INFO(this->get_logger(), "Performance monitoring thresholds initialized");
}

void PerformanceMonitor::ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
  // Handle parameter updates
  for (const auto& changed_param : event->changed_parameters) {
    if (changed_param.name == "frequency_warn_threshold") {
      frequency_warn_threshold_ = changed_param.value.double_value;
    } else if (changed_param.name == "frequency_error_threshold") {
      frequency_error_threshold_ = changed_param.value.double_value;
    } else if (changed_param.name == "execution_time_warn_threshold") {
      execution_time_warn_threshold_ = changed_param.value.integer_value;
    } else if (changed_param.name == "execution_time_error_threshold") {
      execution_time_error_threshold_ = changed_param.value.integer_value;
    }
    // Add other parameter updates as needed
  }
}

void PerformanceMonitor::ProcessPerformanceData(uint8_t board_id, float loop_frequency, 
                                               uint32_t avg_execution_time, uint32_t max_execution_time,
                                               uint8_t memory_usage, uint32_t violations_count) {
  // Update performance state for this board
  auto& state = performance_states_[board_id];
  state.loop_frequency = loop_frequency;
  state.avg_execution_time = avg_execution_time;
  state.max_execution_time = max_execution_time;
  state.memory_usage = memory_usage;
  state.last_violations_count = state.violations_count;
  state.violations_count = violations_count;
  state.last_update = this->now();
  state.data_valid = true;

  // Publish individual metrics for plotting/monitoring
  auto frequency_msg = std_msgs::msg::Float32();
  frequency_msg.data = loop_frequency;
  frequency_pub_->publish(frequency_msg);

  // Log violations if they increased
  if (violations_count > state.last_violations_count) {
    uint32_t new_violations = violations_count - state.last_violations_count;
    RCLCPP_WARN(this->get_logger(), 
      "Board %d: %u new performance violations detected (total: %u)",
      board_id, new_violations, violations_count);
  }

  RCLCPP_DEBUG(this->get_logger(), 
    "Performance data from board %d: %.1fHz, %uus avg, %uus max, %u%% mem, %u violations",
    board_id, loop_frequency, avg_execution_time, max_execution_time, memory_usage, violations_count);
}

void PerformanceMonitor::PublishDiagnostics() {
  auto diag_array = diagnostic_msgs::msg::DiagnosticArray();
  diag_array.header.stamp = this->now();

  for (const auto& [board_id, state] : performance_states_) {
    if (!state.data_valid) continue;

    auto diag_status = diagnostic_msgs::msg::DiagnosticStatus();
    diag_status.name = "teensy_v2_performance_board_" + std::to_string(board_id);
    diag_status.level = CheckPerformanceHealth(state);
    diag_status.message = GetPerformanceSeverityMessage(state);

    // Add key-value diagnostics
    diagnostic_msgs::msg::KeyValue kv;
    
    kv.key = "loop_frequency";
    kv.value = std::to_string(state.loop_frequency);
    diag_status.values.push_back(kv);
    
    kv.key = "avg_execution_time_us";
    kv.value = std::to_string(state.avg_execution_time);
    diag_status.values.push_back(kv);
    
    kv.key = "max_execution_time_us";
    kv.value = std::to_string(state.max_execution_time);
    diag_status.values.push_back(kv);
    
    kv.key = "memory_usage_percent";
    kv.value = std::to_string(state.memory_usage);
    diag_status.values.push_back(kv);
    
    kv.key = "violations_count";
    kv.value = std::to_string(state.violations_count);
    diag_status.values.push_back(kv);

    diag_array.status.push_back(diag_status);
  }

  if (!diag_array.status.empty()) {
    diagnostics_pub_->publish(diag_array);
  }
}

uint8_t PerformanceMonitor::CheckPerformanceHealth(const PerformanceState& state) {
  // Check frequency (lower is worse)
  if (state.loop_frequency < frequency_error_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (state.loop_frequency < frequency_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  // Check execution time (higher is worse)
  if (state.max_execution_time > execution_time_error_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (state.max_execution_time > execution_time_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  // Check memory usage (higher is worse)
  if (state.memory_usage > memory_error_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  if (state.memory_usage > memory_warn_threshold_) {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

std::string PerformanceMonitor::GetPerformanceSeverityMessage(const PerformanceState& state) {
  std::vector<std::string> issues;

  if (state.loop_frequency < frequency_error_threshold_) {
    issues.push_back("Critical frequency drop");
  } else if (state.loop_frequency < frequency_warn_threshold_) {
    issues.push_back("Low frequency");
  }

  if (state.max_execution_time > execution_time_error_threshold_) {
    issues.push_back("Critical execution time");
  } else if (state.max_execution_time > execution_time_warn_threshold_) {
    issues.push_back("High execution time");
  }

  if (state.memory_usage > memory_error_threshold_) {
    issues.push_back("Critical memory usage");
  } else if (state.memory_usage > memory_warn_threshold_) {
    issues.push_back("High memory usage");
  }

  if (issues.empty()) {
    return "Performance nominal";
  } else {
    std::string message = "Issues: ";
    for (size_t i = 0; i < issues.size(); ++i) {
      message += issues[i];
      if (i < issues.size() - 1) message += ", ";
    }
    return message;
  }
}

}  // namespace sigyn_to_sensor_v2
