/**
 * @file message_parser.cpp
 * @brief Implementation of TeensyV2 message parsing system
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "sigyn_to_sensor_v2/message_parser.h"

#include <sstream>
#include <algorithm>
#include <regex>

namespace sigyn_to_sensor_v2 {

MessageParser::MessageParser(rclcpp::Logger logger)
    : logger_(logger),
      current_board_id_(0),
      total_messages_received_(0),
      total_messages_parsed_(0),
      total_parsing_errors_(0),
      total_validation_errors_(0) {
}

void MessageParser::RegisterCallback(MessageType type, MessageCallback callback) {
  callbacks_[type] = callback;
}

bool MessageParser::ParseMessage(const std::string& message, rclcpp::Time timestamp) {
  total_messages_received_++;
  
  if (timestamp.nanoseconds() == 0) {
    timestamp = rclcpp::Clock().now();
  }
  
  // Validate message format
  if (!ValidateMessage(message)) {
    total_validation_errors_++;
    RCLCPP_WARN(logger_, "Message validation failed: %s", message.c_str());
    return false;
  }
  
  try {
    // Parse message type
    MessageType type = ParseMessageType(message);
    if (type == MessageType::UNKNOWN) {
      total_parsing_errors_++;
      RCLCPP_WARN(logger_, "Unknown message type: %s", message.c_str());
      return false;
    }
    
    // Find colon separator (validation ensures it exists)
    size_t colon_pos = message.find(':');
    MessageData data;
    
    // Get message content after the colon
    std::string content = message.substr(colon_pos + 1);
    
    RCLCPP_DEBUG(logger_, "ParseMessage: type=%d, content='%s'", static_cast<int>(type), content.c_str());
    
    // Parse content based on message type - now most messages use JSON format
    if (type == MessageType::PERFORMANCE || type == MessageType::VL53L0X || 
        type == MessageType::ROBOCLAW ||
        type == MessageType::BATTERY || type == MessageType::IMU || 
        type == MessageType::ODOM || type == MessageType::TEMPERATURE ||
        (content.find('{') == 0)) {
      // JSON format messages (most messages now use this format)
      RCLCPP_DEBUG(logger_, "ParseMessage: Using JSON parser for content");
      data = ParseComprehensiveJsonContent(content);
    } else if (type == MessageType::DIAGNOSTIC || type == MessageType::INIT || type == MessageType::CRITICAL) {
      // Check if this is a structured diagnostic message (DIAG:) or free-form (DEBUG:, etc.)
      std::string type_prefix = message.substr(0, colon_pos);
      // Remove board ID from type prefix for comparison
      if (!type_prefix.empty() && std::isdigit(type_prefix.back())) {
        type_prefix = type_prefix.substr(0, type_prefix.length() - 1);
      }
      
      RCLCPP_DEBUG(logger_, "ParseMessage: Diagnostic type_prefix='%s', full_message='%s'", type_prefix.c_str(), message.c_str());
      
      if (type_prefix == "DIAG") {
        // Structured diagnostic messages can be JSON or key:value format
        if (content.find('{') == 0) {
          RCLCPP_DEBUG(logger_, "ParseMessage: Parsing DIAG as JSON");
          data = ParseComprehensiveJsonContent(content);
        } else {
          RCLCPP_DEBUG(logger_, "ParseMessage: Parsing DIAG as key:value");
          data = ParseKeyValuePairs(content);
        }
      } else {
        // DEBUG, INFO, ERROR, etc. messages are free-form diagnostic text
        RCLCPP_DEBUG(logger_, "ParseMessage: Free-form diagnostic message, content='%s'", content.c_str());
        data["message"] = content;
        RCLCPP_DEBUG(logger_, "%s: %s", type_prefix.c_str(), content.c_str());
      }
    } else {
      // Fallback to key:value format for any remaining messages
      RCLCPP_DEBUG(logger_, "ParseMessage: Using key:value parser for content");
      data = ParseKeyValuePairs(content);
    }
    
    RCLCPP_DEBUG(logger_, "ParseMessage: Parsed data has %zu fields", data.size());
    for (const auto& kv : data) {
      RCLCPP_DEBUG(logger_, "  Data field: '%s' = '%s'", kv.first.c_str(), kv.second.c_str());
    }
    
    // Update statistics
    messages_by_type_[type]++;
    total_messages_parsed_++;
    
    // Call registered callback
    auto callback_it = callbacks_.find(type);
    if (callback_it != callbacks_.end()) {
      callback_it->second(data, timestamp);
    }
    
    return true;
    
  } catch (const std::exception& e) {
    total_parsing_errors_++;
    errors_by_type_[MessageType::UNKNOWN]++;
    RCLCPP_ERROR(logger_, "Exception parsing message '%s': %s", message.c_str(), e.what());
    return false;
  }
}

BatteryData MessageParser::ParseBatteryData(const MessageData& data) const {
  BatteryData battery;
  
  try {
    // Handle both old and new field names
    auto it = data.find("idx");  // New format
    if (it == data.end()) {
      it = data.find("id");      // Legacy format
    }
    if (it != data.end()) {
      battery.id = SafeStringToInt(it->second, 0);
    }
    
    it = data.find("V");         // New format (uppercase)
    if (it == data.end()) {
      it = data.find("v");       // Legacy format (lowercase)
    }
    if (it != data.end()) {
      battery.voltage = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("A");         // New format (uppercase)
    if (it == data.end()) {
      it = data.find("c");       // Legacy format (current)
    }
    if (it != data.end()) {
      battery.current = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("p");
    if (it != data.end()) {
      battery.power = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("charge");    // New format
    if (it == data.end()) {
      it = data.find("pct");     // Legacy format
    }
    if (it != data.end()) {
      battery.percentage = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("state");
    if (it != data.end()) {
      battery.state = it->second;
    }
    
    it = data.find("sensors");
    if (it != data.end()) {
      battery.sensors = it->second;
    }
    
    it = data.find("location");
    if (it != data.end()) {
      battery.location = it->second;
    }
    
    battery.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing battery data: %s", e.what());
    battery.valid = false;
  }
  
  return battery;
}

PerformanceData MessageParser::ParsePerformanceData(const MessageData& data) const {
  PerformanceData perf;
  
  try {
    auto it = data.find("freq");
    if (it != data.end()) {
      perf.loop_frequency = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("exec");
    if (it != data.end()) {
      perf.execution_time = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("viol");
    if (it != data.end()) {
      perf.violation_count = SafeStringToInt(it->second, 0);
    }
    
    it = data.find("modules");
    if (it != data.end()) {
      perf.module_count = SafeStringToInt(it->second, 0);
    }
    
    it = data.find("avg_freq");
    if (it != data.end()) {
      perf.avg_frequency = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("max_exec");
    if (it != data.end()) {
      perf.max_execution_time = SafeStringToDouble(it->second, 0.0);
    }
    
    perf.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing performance data: %s", e.what());
    perf.valid = false;
  }
  
  return perf;
}

IMUData MessageParser::ParseIMUData(const MessageData& data) const {
  IMUData imu;
  
  try {
    auto it = data.find("id");
    if (it != data.end()) {
      imu.sensor_id = SafeStringToInt(it->second, 0);
    }
    
    // Parse quaternion data
    it = data.find("qx");
    if (it != data.end()) {
      imu.qx = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("qy");
    if (it != data.end()) {
      imu.qy = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("qz");
    if (it != data.end()) {
      imu.qz = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("qw");
    if (it != data.end()) {
      imu.qw = SafeStringToDouble(it->second, 1.0);
    }
    
    // Parse gyroscope data
    it = data.find("gx");
    if (it != data.end()) {
      imu.gyro_x = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("gy");
    if (it != data.end()) {
      imu.gyro_y = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("gz");
    if (it != data.end()) {
      imu.gyro_z = SafeStringToDouble(it->second, 0.0);
    }
    
    // Parse accelerometer data
    it = data.find("ax");
    if (it != data.end()) {
      imu.accel_x = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("ay");
    if (it != data.end()) {
      imu.accel_y = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("az");
    if (it != data.end()) {
      imu.accel_z = SafeStringToDouble(it->second, 0.0);
    }
    
    // Parse status data
    it = data.find("calib");
    if (it != data.end()) {
      // Handle hex format (0x prefix) or decimal
      std::string calib_str = it->second;
      if (calib_str.find("0x") == 0) {
        imu.calibration_status = std::stoul(calib_str, nullptr, 16);
      } else {
        imu.calibration_status = SafeStringToInt(calib_str, 0);
      }
    }
    
    it = data.find("status");
    if (it != data.end()) {
      // Handle hex format (0x prefix) or decimal
      std::string status_str = it->second;
      if (status_str.find("0x") == 0) {
        imu.system_status = std::stoul(status_str, nullptr, 16);
      } else {
        imu.system_status = SafeStringToInt(status_str, 0);
      }
    }
    
    it = data.find("error");
    if (it != data.end()) {
      // Handle hex format (0x prefix) or decimal
      std::string error_str = it->second;
      if (error_str.find("0x") == 0) {
        imu.system_error = std::stoul(error_str, nullptr, 16);
      } else {
        imu.system_error = SafeStringToInt(error_str, 0);
      }
    }
    
    it = data.find("timestamp");
    if (it != data.end()) {
      imu.timestamp = SafeStringToInt(it->second, 0);
    }
    
    imu.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing IMU data: %s", e.what());
    imu.valid = false;
  }
  
  return imu;
}

SafetyData MessageParser::ParseSafetyData(const MessageData& data) const {
  SafetyData safety;
  
  try {
    auto it = data.find("state");
    if (it != data.end()) {
      safety.state = it->second;
    }
    
    it = data.find("hw_estop");
    if (it != data.end()) {
      safety.hardware_estop = SafeStringToBool(it->second, false);
    }
    
    it = data.find("inter_board");
    if (it != data.end()) {
      safety.inter_board_safety = SafeStringToBool(it->second, false);
    }
    
    it = data.find("active_conditions");
    if (it != data.end()) {
      safety.active_conditions = SafeStringToBool(it->second, false);
    }
    
    it = data.find("sources");
    if (it != data.end()) {
      safety.sources = ParseCommaSeparatedList(it->second);
    }
    
    safety.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing safety data: %s", e.what());
    safety.valid = false;
  }
  
  return safety;
}

EstopData MessageParser::ParseEstopData(const MessageData& data) const {
  EstopData estop;
  
  try {
    auto it = data.find("active");
    if (it != data.end()) {
      estop.active = SafeStringToBool(it->second, false);
    }
    
    it = data.find("source");
    if (it != data.end()) {
      estop.source = it->second;
    }
    
    it = data.find("reason");
    if (it != data.end()) {
      estop.reason = it->second;
    }
    
    it = data.find("value");
    if (it != data.end()) {
      estop.trigger_value = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("manual_reset");
    if (it != data.end()) {
      estop.manual_reset_required = SafeStringToBool(it->second, false);
    }
    
    it = data.find("time");
    if (it != data.end()) {
      estop.timestamp = static_cast<uint64_t>(SafeStringToInt(it->second, 0));
    }
    
    estop.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing E-stop data: %s", e.what());
    estop.valid = false;
  }
  
  return estop;
}

DiagnosticData MessageParser::ParseDiagnosticData(const MessageData& data) const {
  DiagnosticData diag;
  
  // Debug: Log the diagnostic data being parsed
  RCLCPP_DEBUG(logger_, "ParseDiagnosticData: Parsing data with %zu fields", data.size());
  std::string debug_fields = "Fields: ";
  for (const auto& kv : data) {
    debug_fields += "['" + kv.first + "'='" + kv.second + "'] ";
  }
  RCLCPP_DEBUG(logger_, "%s", debug_fields.c_str());
  
  try {
    // Check if this is structured diagnostic data (level/module/message format)
    auto level_it = data.find("level");
    auto module_it = data.find("module");
    auto msg_it = data.find("msg");
    auto message_it = data.find("message");
    
    if (level_it != data.end() || module_it != data.end() || msg_it != data.end() || message_it != data.end()) {
      // Structured diagnostic data
      if (level_it != data.end()) {
        diag.level = level_it->second;
      }
      
      if (module_it != data.end()) {
        diag.module = module_it->second;
      } else {
        // Default module name for diagnostic messages without explicit module
        diag.module = "teensy_sensor_board_" + std::to_string(current_board_id_);
      }
      
      if (msg_it != data.end()) {
        diag.message = msg_it->second;
      } else if (message_it != data.end()) {
        diag.message = message_it->second;
      }
      
      auto details_it = data.find("details");
      if (details_it != data.end()) {
        diag.details = details_it->second;
      }
      
      auto time_it = data.find("time");
      if (time_it != data.end()) {
        diag.timestamp = static_cast<uint64_t>(SafeStringToInt(time_it->second, 0));
      }
    } else {
      // Simple diagnostic message (DIAG: free-form text)
      auto message_it = data.find("message");
      if (message_it != data.end()) {
        // Use the free-form message as both module name and message content
        std::string full_message = message_it->second;
        
        // Try to extract module name from message (e.g., "BNO055Monitor PERF_VIOLATION: ...")
        size_t first_space = full_message.find(' ');
        if (first_space != std::string::npos) {
          diag.module = full_message.substr(0, first_space);
          diag.message = full_message;
        } else {
          diag.module = "teensy_sensor";
          diag.message = full_message;
        }
        
        // Default level to INFO for simple messages
        diag.level = "INFO";
      } else {
        // Check if this is a performance data message (has freq, mviol, etc.)
        auto freq_it = data.find("freq");
        auto mviol_it = data.find("mviol");
        auto fviol_it = data.find("fviol");
        
        if (freq_it != data.end() || mviol_it != data.end() || fviol_it != data.end()) {
          // This is performance data - construct a diagnostic message from it
          diag.module = "PerformanceMonitor";
          diag.level = "INFO";
          
          // Build message from performance data
          std::string perf_msg = "Performance: ";
          if (freq_it != data.end()) {
            perf_msg += "freq=" + freq_it->second + "Hz ";
          }
          if (mviol_it != data.end()) {
            perf_msg += "module_violations=" + mviol_it->second + " ";
          }
          if (fviol_it != data.end()) {
            perf_msg += "freq_violations=" + fviol_it->second;
          }
          
          diag.message = perf_msg;
        } else {
          RCLCPP_WARN(logger_, "Diagnostic data has no message content and no recognizable performance data");
          diag.valid = false;
          return diag;
        }
      }
    }
    
    diag.valid = true;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing diagnostic data: %s", e.what());
    diag.valid = false;
  }
  
  return diag;
}

sensor_msgs::msg::BatteryState MessageParser::ToBatteryStateMsg(const BatteryData& data,
                                                                 rclcpp::Time timestamp) const {
  sensor_msgs::msg::BatteryState msg;
  
  msg.header.stamp = timestamp;
  msg.header.frame_id = "battery_" + std::to_string(data.id);
  
  msg.voltage = static_cast<float>(data.voltage);
  msg.current = static_cast<float>(data.current);
  msg.charge = NAN;  // Not directly provided
  msg.capacity = NAN;  // Not directly provided
  msg.design_capacity = NAN;  // Not directly provided
  msg.percentage = static_cast<float>(data.percentage);
  
  // Set location field from battery data
  msg.location = data.location;
  
  // Map state string to enum
  if (data.state == "CHARGING") {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (data.state == "DISCHARGING") {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (data.state == "CRITICAL") {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else {
    msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  }
  
  msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.present = true;
  
  return msg;
}

sensor_msgs::msg::Imu MessageParser::ToImuMsg(const IMUData& data,
                                               rclcpp::Time timestamp) const {
  sensor_msgs::msg::Imu msg;
  
  msg.header.stamp = timestamp;
  msg.header.frame_id = "imu_" + std::to_string(data.sensor_id);
  
  // Set orientation (quaternion)
  msg.orientation.x = data.qx;
  msg.orientation.y = data.qy;
  msg.orientation.z = data.qz;
  msg.orientation.w = data.qw;
  
  // Set angular velocity (rad/s)
  msg.angular_velocity.x = data.gyro_x;
  msg.angular_velocity.y = data.gyro_y;
  msg.angular_velocity.z = data.gyro_z;
  
  // Set linear acceleration (m/s²)
  msg.linear_acceleration.x = data.accel_x;
  msg.linear_acceleration.y = data.accel_y;
  msg.linear_acceleration.z = data.accel_z;
  
  // Set covariance matrices (unknown/identity for now)
  // In a real implementation, these would be calibrated values
  for (int i = 0; i < 9; i++) {
    msg.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;  // Small diagonal covariance
    msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
    msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
  }
  
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray MessageParser::ToDiagnosticArrayMsg(const DiagnosticData& data,
                                                                          rclcpp::Time timestamp) const {
  diagnostic_msgs::msg::DiagnosticArray msg;
  
  msg.header.stamp = timestamp;
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = data.module;
  status.message = data.message;
  
  // Include board ID in hardware_id
  if (current_board_id_ > 0) {
    status.hardware_id = "teensy_v2_board_" + std::to_string(current_board_id_);
  } else {
    status.hardware_id = "teensy_v2";
  }
  
  // Map level string to enum
  if (data.level == "INFO") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  } else if (data.level == "WARN") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  } else if (data.level == "ERROR") {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
  }
  
  // Add board ID as a key-value pair
  if (current_board_id_ > 0) {
    diagnostic_msgs::msg::KeyValue board_kv;
    board_kv.key = "board_id";
    board_kv.value = std::to_string(current_board_id_);
    status.values.push_back(board_kv);
  }
  
  // Add details as key-value pairs
  if (!data.details.empty()) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "details";
    kv.value = data.details;
    status.values.push_back(kv);
  }
  
  // Add timestamp
  diagnostic_msgs::msg::KeyValue ts_kv;
  ts_kv.key = "timestamp";
  ts_kv.value = std::to_string(data.timestamp);
  status.values.push_back(ts_kv);
  
  msg.status.push_back(status);
  
  return msg;
}

diagnostic_msgs::msg::DiagnosticArray MessageParser::GetParsingStatistics() const {
  diagnostic_msgs::msg::DiagnosticArray msg;
  
  msg.header.stamp = rclcpp::Clock().now();
  
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "message_parser";
  status.message = "Message parsing statistics";
  status.hardware_id = "teensy_v2";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  
  // Add statistics as key-value pairs
  diagnostic_msgs::msg::KeyValue kv;
  
  kv.key = "total_received";
  kv.value = std::to_string(total_messages_received_);
  status.values.push_back(kv);
  
  kv.key = "total_parsed";
  kv.value = std::to_string(total_messages_parsed_);
  status.values.push_back(kv);
  
  kv.key = "parsing_errors";
  kv.value = std::to_string(total_parsing_errors_);
  status.values.push_back(kv);
  
  kv.key = "validation_errors";
  kv.value = std::to_string(total_validation_errors_);
  status.values.push_back(kv);
  
  // Calculate success rate
  double success_rate = 0.0;
  if (total_messages_received_ > 0) {
    success_rate = static_cast<double>(total_messages_parsed_) / total_messages_received_ * 100.0;
  }
  kv.key = "success_rate_percent";
  kv.value = std::to_string(success_rate);
  status.values.push_back(kv);
  
  msg.status.push_back(status);
  
  return msg;
}

void MessageParser::ResetStatistics() {
  total_messages_received_ = 0;
  total_messages_parsed_ = 0;
  total_parsing_errors_ = 0;
  total_validation_errors_ = 0;
  messages_by_type_.clear();
  errors_by_type_.clear();
}

MessageType MessageParser::ParseMessageType(const std::string& message) const {
  size_t colon_pos = message.find(':');
  if (colon_pos == std::string::npos) {
    // Messages without colons are invalid
    return MessageType::UNKNOWN;
  }
  
  std::string type_str_with_id = message.substr(0, colon_pos);
  
  // Extract board ID if present (messages now come as ODOM1:, BATT2:, etc.)
  std::string type_str = type_str_with_id;
  int board_id = 0;
  
  // Check if the type string ends with a digit (board ID)
  if (!type_str_with_id.empty() && std::isdigit(type_str_with_id.back())) {
    board_id = type_str_with_id.back() - '0';
    type_str = type_str_with_id.substr(0, type_str_with_id.length() - 1);
    
    // Store the extracted board ID for use in message processing
    current_board_id_ = board_id;
  } else {
    current_board_id_ = 0; // Unknown/unspecified board
  }
  
  return StringToMessageType(type_str);
}

MessageData MessageParser::ParseKeyValuePairs(const std::string& content) const {
  MessageData data;
  
  if (content.empty()) {
    return data;
  }
  
  std::istringstream stream(content);
  std::string pair;
  
  while (std::getline(stream, pair, ',')) {
    // Try colon separator first (new format)
    size_t separator_pos = pair.find(':');
    if (separator_pos == std::string::npos) {
      // Fall back to equals separator (legacy format)
      separator_pos = pair.find('=');
    }
    
    if (separator_pos != std::string::npos) {
      std::string key = pair.substr(0, separator_pos);
      std::string value = pair.substr(separator_pos + 1);
      
      // Trim whitespace
      key.erase(0, key.find_first_not_of(" \t"));
      key.erase(key.find_last_not_of(" \t") + 1);
      value.erase(0, value.find_first_not_of(" \t"));
      value.erase(value.find_last_not_of(" \t") + 1);
      
      data[key] = value;
    }
  }
  
  return data;
}

bool MessageParser::ValidateMessage(const std::string& message) const {
  // Debug: Log all messages being validated
  RCLCPP_DEBUG(logger_, "ValidateMessage: Checking message: '%s'", message.c_str());
  
  if (message.empty() || message.length() > 2048) {
    RCLCPP_DEBUG(logger_, "ValidateMessage: FAILED - empty or too long (length: %zu)", message.length());
    return false;
  }
  
  // Check for colon separator - all valid messages must have one
  size_t colon_pos = message.find(':');
  if (colon_pos == std::string::npos || colon_pos == 0) {
    RCLCPP_DEBUG(logger_, "ValidateMessage: FAILED - no colon separator found");
    return false;  // Reject messages without colons or with colon as first character
  }
  
  // Get message type and content
  std::string type_str = message.substr(0, colon_pos);
  std::string content = message.substr(colon_pos + 1);
  
  RCLCPP_DEBUG(logger_, "ValidateMessage: type_str='%s', content='%s'", type_str.c_str(), content.c_str());
  
  // Extract base message type (remove board ID suffix if present)
  std::string base_type = type_str;
  if (!type_str.empty() && std::isdigit(type_str.back())) {
    // Remove board ID digit from the end (e.g., "ODOM1" -> "ODOM")
    base_type = type_str.substr(0, type_str.length() - 1);
  }
  
  RCLCPP_DEBUG(logger_, "ValidateMessage: base_type='%s'", base_type.c_str());
  
  // Check if message type is recognized
  MessageType type = StringToMessageType(base_type);
  if (type == MessageType::UNKNOWN) {
    RCLCPP_DEBUG(logger_, "ValidateMessage: FAILED - unknown message type '%s'", base_type.c_str());
    return false;
  }
  
  // JSON messages should start with { 
  if (base_type == "PERF" || base_type == "VL53L0X" || base_type == "ROBOCLAW" || 
      base_type == "BATT" || base_type == "IMU" || base_type == "ODOM" || 
      base_type == "TEMP" || base_type == "TEMPERATURE" ||
      (content.find('{') == 0)) {
    return !content.empty() && content[0] == '{';
  } 
  
  // Diagnostic messages are free-form text (but must have recognized type)
  if (base_type == "DIAG" || base_type == "WARNING" || base_type == "DEBUG" || 
      base_type == "INFO" || base_type == "ERROR" || base_type == "CRITICAL" || 
      base_type == "INIT") {
    return !content.empty();
  }
  
  // All other messages should have some content
  return !content.empty();
}

MessageType MessageParser::StringToMessageType(const std::string& type_str) const {
  if (type_str == "BATT") return MessageType::BATTERY;
  if (type_str == "PERF") return MessageType::PERFORMANCE;
  if (type_str == "SAFETY") return MessageType::SAFETY;
  if (type_str == "IMU") return MessageType::IMU;
  if (type_str == "TEMP") return MessageType::TEMPERATURE;
  if (type_str == "TEMPERATURE") return MessageType::TEMPERATURE;
  if (type_str == "VL53L0X") return MessageType::VL53L0X;
  if (type_str == "ROBOCLAW") return MessageType::ROBOCLAW;
  if (type_str == "ODOM") return MessageType::ODOM;
  if (type_str == "ESTOP") return MessageType::ESTOP;
  if (type_str == "DIAG") return MessageType::DIAGNOSTIC;
  if (type_str == "CONFIG") return MessageType::CONFIG;
  if (type_str == "INIT") return MessageType::INIT;
  if (type_str == "CRITICAL") return MessageType::CRITICAL;
  if (type_str == "SDIR") return MessageType::SDIR;
  if (type_str == "SDLINE") return MessageType::SDLINE;
  if (type_str == "SDEOF") return MessageType::SDEOF;
  // Handle free-form diagnostic messages
  if (type_str == "WARNING" || type_str == "DEBUG" || type_str == "INFO" || 
      type_str == "ERROR" || type_str == "WARN") {
    return MessageType::DIAGNOSTIC;
  }
  return MessageType::UNKNOWN;
}

std::string MessageParser::MessageTypeToString(MessageType type) const {
  switch (type) {
    case MessageType::BATTERY: return "BATT";
    case MessageType::PERFORMANCE: return "PERF";
    case MessageType::SAFETY: return "SAFETY";
    case MessageType::IMU: return "IMU";
    case MessageType::TEMPERATURE: return "TEMP";
    case MessageType::VL53L0X: return "VL53L0X";
    case MessageType::ROBOCLAW: return "ROBOCLAW";
    case MessageType::ODOM: return "ODOM";
    case MessageType::ESTOP: return "ESTOP";
    case MessageType::DIAGNOSTIC: return "DIAG";
    case MessageType::CONFIG: return "CONFIG";
    case MessageType::INIT: return "INIT";
    case MessageType::CRITICAL: return "CRITICAL";
    case MessageType::SDIR: return "SDIR";
    case MessageType::SDLINE: return "SDLINE";
    case MessageType::SDEOF: return "SDEOF";
    default: return "UNKNOWN";
  }
}

double MessageParser::SafeStringToDouble(const std::string& str, double default_value) const {
  try {
    return std::stod(str);
  } catch (const std::exception&) {
    return default_value;
  }
}

int MessageParser::SafeStringToInt(const std::string& str, int default_value) const {
  try {
    return std::stoi(str);
  } catch (const std::exception&) {
    return default_value;
  }
}

bool MessageParser::SafeStringToBool(const std::string& str, bool default_value) const {
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
  
  if (lower_str == "true" || lower_str == "1" || lower_str == "yes" || lower_str == "on") {
    return true;
  } else if (lower_str == "false" || lower_str == "0" || lower_str == "no" || lower_str == "off") {
    return false;
  } else {
    return default_value;
  }
}

std::vector<std::string> MessageParser::ParseCommaSeparatedList(const std::string& str) const {
  std::vector<std::string> result;
  
  if (str.empty()) {
    return result;
  }
  
  std::istringstream stream(str);
  std::string item;
  
  while (std::getline(stream, item, '+')) {  // Using '+' as separator for E-stop sources
    // Trim whitespace
    item.erase(0, item.find_first_not_of(" \t"));
    item.erase(item.find_last_not_of(" \t") + 1);
    
    if (!item.empty()) {
      result.push_back(item);
    }
  }
  
  return result;
}

MessageData MessageParser::ParseComprehensiveJsonContent(const std::string& content) const {
  MessageData data;
  
  if (content.empty() || content.front() != '{' || content.back() != '}') {
    return data;
  }
  
  // Store raw JSON for debugging
  data["json"] = content;
  
  try {
    // Simple JSON parser - extract key-value pairs from {"key":value,"key2":"value2",...}
    std::string json_body = content.substr(1, content.length() - 2); // Remove { and }
    
    size_t pos = 0;
    while (pos < json_body.length()) {
      // Find key start
      size_t key_start = json_body.find('"', pos);
      if (key_start == std::string::npos) break;
      key_start++; // Skip opening quote
      
      // Find key end
      size_t key_end = json_body.find('"', key_start);
      if (key_end == std::string::npos) break;
      
      std::string key = json_body.substr(key_start, key_end - key_start);
      
      // Find colon
      size_t colon_pos = json_body.find(':', key_end);
      if (colon_pos == std::string::npos) break;
      colon_pos++; // Skip colon
      
      // Skip whitespace after colon
      while (colon_pos < json_body.length() && 
             (json_body[colon_pos] == ' ' || json_body[colon_pos] == '\t')) {
        colon_pos++;
      }
      
      // Parse value
      std::string value;
      if (colon_pos < json_body.length()) {
        if (json_body[colon_pos] == '"') {
          // String value
          colon_pos++; // Skip opening quote
          size_t value_end = json_body.find('"', colon_pos);
          if (value_end != std::string::npos) {
            value = json_body.substr(colon_pos, value_end - colon_pos);
            pos = value_end + 1;
          } else {
            break;
          }
        } else if (json_body[colon_pos] == '[') {
          // Array value - find matching closing bracket
          size_t bracket_count = 1;
          size_t array_start = colon_pos;
          colon_pos++; // Skip opening bracket
          
          while (colon_pos < json_body.length() && bracket_count > 0) {
            if (json_body[colon_pos] == '[') bracket_count++;
            else if (json_body[colon_pos] == ']') bracket_count--;
            colon_pos++;
          }
          
          if (bracket_count == 0) {
            value = json_body.substr(array_start, colon_pos - array_start);
            pos = colon_pos;
          } else {
            break;
          }
        } else {
          // Numeric or boolean value - find next comma or end
          size_t value_end = json_body.find(',', colon_pos);
          if (value_end == std::string::npos) {
            value_end = json_body.length();
          }
          value = json_body.substr(colon_pos, value_end - colon_pos);
          pos = value_end;
        }
        
        // Store the key-value pair
        data[key] = value;
      }
      
      // Find next key (skip comma and whitespace)
      while (pos < json_body.length() && 
             (json_body[pos] == ',' || json_body[pos] == ' ' || json_body[pos] == '\t')) {
        pos++;
      }
    }
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing JSON content: %s", e.what());
    // Clear extracted data but keep raw JSON for debugging
    data.clear();
    data["json"] = content;
  }
  
  return data;
}

MessageData MessageParser::ParseJsonContent(const std::string& content) const {
  MessageData data;
  
  if (content.empty()) {
    return data;
  }
  
  // For PERF messages, we'll store the entire JSON as a single "json" key
  // This allows the consumer to handle JSON parsing if needed
  data["json"] = content;
  
  // Also extract some commonly used values for easy access
  try {
    // Simple JSON value extraction without full parser
    // Look for common PERF fields: freq, target_freq, mod_viol, freq_viol
    
    auto extractJsonValue = [&content](const std::string& key) -> std::string {
      std::string search = "\"" + key + "\":";
      size_t pos = content.find(search);
      if (pos == std::string::npos) {
        // Try without quotes for numeric keys
        search = "\"" + key + "\":";
        pos = content.find(search);
        if (pos == std::string::npos) {
          return "";
        }
      }
      
      pos += search.length();
      
      // Skip whitespace
      while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) {
        pos++;
      }
      
      // Extract value (up to comma, closing brace, or end)
      size_t end_pos = pos;
      while (end_pos < content.length() && 
             content[end_pos] != ',' && 
             content[end_pos] != '}' && 
             content[end_pos] != ']') {
        end_pos++;
      }
      
      std::string value = content.substr(pos, end_pos - pos);
      
      // Trim quotes if present
      if (!value.empty() && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.length() - 2);
      }
      
      return value;
    };
    
    // Extract common PERF fields
    std::string freq = extractJsonValue("freq");
    if (!freq.empty()) data["freq"] = freq;
    
    std::string target_freq = extractJsonValue("tfreq");  // Fixed: use actual field name "tfreq"
    if (!target_freq.empty()) data["target_freq"] = target_freq;
    
    std::string mod_viol = extractJsonValue("mviol");  // Fixed: use actual field name "mviol"
    if (!mod_viol.empty()) data["mod_viol"] = mod_viol;
    
    std::string freq_viol = extractJsonValue("fviol");  // Fixed: use actual field name "fviol"
    if (!freq_viol.empty()) data["freq_viol"] = freq_viol;
    
    // Extract VL53L0X fields
    std::string total_sensors = extractJsonValue("total_sensors");
    if (!total_sensors.empty()) data["total_sensors"] = total_sensors;
    
    std::string active_sensors = extractJsonValue("active_sensors");
    if (!active_sensors.empty()) data["active_sensors"] = active_sensors;
    
    std::string min_distance = extractJsonValue("min_distance");
    if (!min_distance.empty()) data["min_distance"] = min_distance;
    
    std::string max_distance = extractJsonValue("max_distance");
    if (!max_distance.empty()) data["max_distance"] = max_distance;
    
    std::string obstacles = extractJsonValue("obstacles");
    if (!obstacles.empty()) data["obstacles"] = obstacles;
    
    // Extract distances array - special handling needed
    std::string search = "\"distances\":";
    size_t pos = content.find(search);
    if (pos != std::string::npos) {
      pos += search.length();
      // Skip whitespace
      while (pos < content.length() && (content[pos] == ' ' || content[pos] == '\t')) {
        pos++;
      }
      // Look for opening bracket
      if (pos < content.length() && content[pos] == '[') {
        size_t end_pos = content.find(']', pos);
        if (end_pos != std::string::npos) {
          std::string distances_array = content.substr(pos, end_pos - pos + 1);
          data["distances"] = distances_array;
        }
      }
    }
    
    // Extract TEMPERATURE fields
    std::string temperatures = extractJsonValue("temperatures");
    if (!temperatures.empty()) data["temperatures"] = temperatures;
    
    std::string avg_temp = extractJsonValue("avg_temp");
    if (!avg_temp.empty()) data["avg_temp"] = avg_temp;
    
    std::string max_temp = extractJsonValue("max_temp");
    if (!max_temp.empty()) data["max_temp"] = max_temp;
    
    std::string min_temp = extractJsonValue("min_temp");
    if (!min_temp.empty()) data["min_temp"] = min_temp;
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error extracting JSON values: %s", e.what());
    // Still keep the raw JSON even if extraction fails
  }
  
  return data;
}

TemperatureData MessageParser::ParseTemperatureData(const MessageData& data, bool is_array_format) const {
  TemperatureData temp_data;
  temp_data.valid = true;
  
  try {
    if (is_array_format) {
      // Parse TEMPERATURE message (JSON array format)
      temp_data.total_sensors = SafeStringToInt(data.at("total_sensors"), 0);
      temp_data.active_sensors = SafeStringToInt(data.at("active_sensors"), 0);
      
      // Parse temperatures array if present
      auto it = data.find("temperatures");
      if (it != data.end()) {
        std::string temp_array = it->second;
        // Simple array parsing - look for numbers and null values
        std::istringstream stream(temp_array);
        std::string token;
        while (std::getline(stream, token, ',')) {
          // Remove brackets and whitespace
          token.erase(std::remove_if(token.begin(), token.end(), 
                     [](char c) { return c == '[' || c == ']' || c == ' '; }), token.end());
          
          if (token == "null" || token.empty()) {
            temp_data.temperatures.push_back(std::numeric_limits<double>::quiet_NaN());
          } else {
            temp_data.temperatures.push_back(SafeStringToDouble(token, 0.0));
          }
        }
      }
      
      // Parse optional statistics
      temp_data.reading_rate_hz = SafeStringToDouble(data.find("rate_hz") != data.end() ? data.at("rate_hz") : "0", 0.0);
      temp_data.total_readings = SafeStringToInt(data.find("readings") != data.end() ? data.at("readings") : "0", 0);
      temp_data.total_errors = SafeStringToInt(data.find("errors") != data.end() ? data.at("errors") : "0", 0);
      
    } else {
      // Parse TEMP message (individual sensor format)
      RCLCPP_DEBUG(logger_, "Parsing TEMP individual format, data size: %zu", data.size());
      for (const auto& kv : data) {
        RCLCPP_DEBUG(logger_, "  Key: '%s', Value: '%s'", kv.first.c_str(), kv.second.c_str());
      }
      
      temp_data.sensor_id = SafeStringToInt(data.find("id") != data.end() ? data.at("id") : "0", 0);
      temp_data.temperature_c = SafeStringToDouble(data.find("temp") != data.end() ? data.at("temp") : "0", 0.0);
      temp_data.status = data.find("status") != data.end() ? data.at("status") : "UNKNOWN";
      
      RCLCPP_DEBUG(logger_, "Parsed: sensor_id=%d, temperature_c=%.2f, status=%s", 
                   temp_data.sensor_id, temp_data.temperature_c, temp_data.status.c_str());
    }
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing temperature data: %s", e.what());
    temp_data.valid = false;
  }
  
  return temp_data;
}

VL53L0XData MessageParser::ParseVL53L0XData(const MessageData& data) const {
  VL53L0XData vl53_data;
  vl53_data.valid = true;
  
  try {
    vl53_data.total_sensors = SafeStringToInt(data.find("total_sensors") != data.end() ? data.at("total_sensors") : "0", 0);
    vl53_data.active_sensors = SafeStringToInt(data.find("active_sensors") != data.end() ? data.at("active_sensors") : "0", 0);
    vl53_data.min_distance_mm = static_cast<uint16_t>(SafeStringToInt(data.find("min_distance") != data.end() ? data.at("min_distance") : "65535", 65535));
    vl53_data.max_distance_mm = static_cast<uint16_t>(SafeStringToInt(data.find("max_distance") != data.end() ? data.at("max_distance") : "0", 0));
    
    // Parse obstacles boolean
    std::string obstacles_str = data.find("obstacles") != data.end() ? data.at("obstacles") : "false";
    vl53_data.obstacles_detected = (obstacles_str == "true");
    
    // Parse distances array
    auto it = data.find("distances");
    if (it != data.end()) {
      std::string dist_array = it->second;
      // Parse JSON array format: [45,53,40,44,1007,616,1031,8191]
      // Remove brackets
      if (!dist_array.empty() && dist_array.front() == '[') {
        dist_array = dist_array.substr(1);
      }
      if (!dist_array.empty() && dist_array.back() == ']') {
        dist_array = dist_array.substr(0, dist_array.length() - 1);
      }
      
      // Split by comma
      std::istringstream stream(dist_array);
      std::string token;
      while (std::getline(stream, token, ',')) {
        // Remove whitespace
        token.erase(std::remove_if(token.begin(), token.end(), 
                   [](char c) { return c == ' ' || c == '\t'; }), token.end());
        
        if (token == "null" || token.empty()) {
          vl53_data.distances_mm.push_back(0);  // 0 indicates invalid reading
        } else {
          vl53_data.distances_mm.push_back(static_cast<uint16_t>(SafeStringToInt(token, 0)));
        }
      }
    }
    
    // Parse optional statistics
    vl53_data.measurement_rate_hz = SafeStringToDouble(data.find("rate_hz") != data.end() ? data.at("rate_hz") : "0", 0.0);
    vl53_data.total_measurements = SafeStringToInt(data.find("measurements") != data.end() ? data.at("measurements") : "0", 0);
    vl53_data.total_errors = SafeStringToInt(data.find("errors") != data.end() ? data.at("errors") : "0", 0);
    
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Error parsing VL53L0X data: %s", e.what());
    vl53_data.valid = false;
  }
  
  return vl53_data;
}

} // namespace sigyn_to_sensor_v2
