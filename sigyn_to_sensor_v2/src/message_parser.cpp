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
      strict_validation_(true),
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
    
    // Find colon separator
    size_t colon_pos = message.find(':');
    if (colon_pos == std::string::npos) {
      total_parsing_errors_++;
      return false;
    }
    
    // Parse key-value pairs
    std::string content = message.substr(colon_pos + 1);
    MessageData data = ParseKeyValuePairs(content);
    
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
    auto it = data.find("id");
    if (it != data.end()) {
      battery.id = SafeStringToInt(it->second, 0);
    }
    
    it = data.find("v");
    if (it != data.end()) {
      battery.voltage = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("c");
    if (it != data.end()) {
      battery.current = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("p");
    if (it != data.end()) {
      battery.power = SafeStringToDouble(it->second, 0.0);
    }
    
    it = data.find("pct");
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
    size_t equals_pos = pair.find('=');
    if (equals_pos != std::string::npos) {
      std::string key = pair.substr(0, equals_pos);
      std::string value = pair.substr(equals_pos + 1);
      
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
  if (message.empty() || message.length() > 256) {
    return false;
  }
  
  // Check for required colon separator
  size_t colon_pos = message.find(':');
  if (colon_pos == std::string::npos || colon_pos == 0) {
    return false;
  }
  
  // Basic format validation
  if (strict_validation_) {
    // More stringent validation for production use
    std::regex pattern(R"([A-Z][A-Z0-9_]*:[a-zA-Z0-9_]+=.*(,[a-zA-Z0-9_]+=.*)*)", 
                       std::regex_constants::optimize);
    return std::regex_match(message, pattern);
  }
  
  return true;
}

MessageType MessageParser::StringToMessageType(const std::string& type_str) const {
  if (type_str == "BATT") return MessageType::BATTERY;
  if (type_str == "PERF") return MessageType::PERFORMANCE;
  if (type_str == "SAFETY") return MessageType::SAFETY;
  if (type_str == "ESTOP") return MessageType::ESTOP;
  if (type_str == "DIAG") return MessageType::DIAGNOSTIC;
  if (type_str == "CONFIG") return MessageType::CONFIG;
  return MessageType::UNKNOWN;
}

std::string MessageParser::MessageTypeToString(MessageType type) const {
  switch (type) {
    case MessageType::BATTERY: return "BATT";
    case MessageType::PERFORMANCE: return "PERF";
    case MessageType::SAFETY: return "SAFETY";
    case MessageType::ESTOP: return "ESTOP";
    case MessageType::DIAGNOSTIC: return "DIAG";
    case MessageType::CONFIG: return "CONFIG";
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

}  // namespace sigyn_to_sensor_v2
