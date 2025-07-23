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
    
    // Parse content based on message type
    if (type == MessageType::PERFORMANCE || type == MessageType::VL53L0X || 
        type == MessageType::ROBOCLAW ||
        (type == MessageType::TEMPERATURE && content.find('{') == 0)) {
      // PERF, VL53L0X, ROBOCLAW, and TEMPERATURE (array format) messages use JSON format
      data = ParseJsonContent(content);
    } else if (type == MessageType::DIAGNOSTIC || type == MessageType::INIT || type == MessageType::CRITICAL) {
      // DIAG, INIT, and CRITICAL messages are free-form diagnostic text
      data["message"] = content;
      RCLCPP_DEBUG(logger_, "%s: %s", 
                   message.substr(0, colon_pos).c_str(), content.c_str());
    } else {
      // All other messages (including TEMP individual format) use key:value format
      data = ParseKeyValuePairs(content);
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
  
  try {
    auto it = data.find("level");
    if (it != data.end()) {
      diag.level = it->second;
    }
    
    it = data.find("module");
    if (it != data.end()) {
      diag.module = it->second;
    }
    
    it = data.find("msg");
    if (it != data.end()) {
      diag.message = it->second;
    }
    
    it = data.find("details");
    if (it != data.end()) {
      diag.details = it->second;
    }
    
    it = data.find("time");
    if (it != data.end()) {
      diag.timestamp = static_cast<uint64_t>(SafeStringToInt(it->second, 0));
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
  status.hardware_id = "teensy_v2";
  
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
  
  std::string type_str = message.substr(0, colon_pos);
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
  if (message.empty() || message.length() > 2048) {
    return false;
  }
  
  // Check for colon separator - all valid messages must have one
  size_t colon_pos = message.find(':');
  if (colon_pos == std::string::npos || colon_pos == 0) {
    return false;  // Reject messages without colons or with colon as first character
  }
  
  // Get message type and content
  std::string type_str = message.substr(0, colon_pos);
  std::string content = message.substr(colon_pos + 1);
  
  // Check if message type is recognized
  MessageType type = StringToMessageType(type_str);
  if (type == MessageType::UNKNOWN) {
    return false;
  }
  
  // JSON messages should start with { 
  if (type_str == "PERF" || type_str == "VL53L0X" || type_str == "ROBOCLAW" || 
      (type_str == "TEMPERATURE" && !content.empty() && content[0] == '{')) {
    return !content.empty() && content[0] == '{';
  } 
  
  // Diagnostic messages are free-form text (but must have recognized type)
  if (type_str == "DIAG" || type_str == "WARNING" || type_str == "DEBUG" || 
      type_str == "INFO" || type_str == "ERROR" || type_str == "CRITICAL" || 
      type_str == "INIT") {
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
    
    std::string target_freq = extractJsonValue("target_freq");
    if (!target_freq.empty()) data["target_freq"] = target_freq;
    
    std::string mod_viol = extractJsonValue("mod_viol");
    if (!mod_viol.empty()) data["mod_viol"] = mod_viol;
    
    std::string freq_viol = extractJsonValue("freq_viol");
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
