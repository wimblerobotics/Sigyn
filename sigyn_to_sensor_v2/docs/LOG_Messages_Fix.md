# TeensyV2 Message Parser Fix - LOG Messages Support

**Date:** 2025-01-22  
**Issue:** TeensyV2 sends WARN, ERROR, and other diagnostic message types, but ROS2 parser was silently dropping them.

## **Problem Identified**

After auditing the actual TeensyV2 source code, I discovered that the ROS2 message parser was **only recognizing a subset** of the message types actually sent by TeensyV2:

### **TeensyV2 Actually Sends:**
- **Structured Data:** `BATT`, `PERF`, `IMU`, `SAFETY`, `ESTOP`, `DIAG`
- **Log Messages:** `INFO`, `WARN`, `ERROR`, `DEBUG`, `CRITICAL`, `FATAL`, `FAULT`, `SAFETY_CRITICAL`, `INIT`

### **Parser Was Only Handling:**
- `BATT`, `PERF`, `IMU`, `SAFETY`, `ESTOP`, `DIAG` only
- **All other message types** were parsed as `MessageType::UNKNOWN` and **silently dropped**

## **Critical Messages Being Lost:**
- ❌ **`FAULT:` messages** - System fault notifications
- ❌ **`FATAL:` messages** - Fatal error conditions  
- ❌ **`CRITICAL:` messages** - Critical system alerts
- ❌ **`SAFETY_CRITICAL:` messages** - Safety-critical events
- ❌ **`ERROR:` messages** - Error conditions
- ❌ **`WARN:` messages** - Warning notifications

## **Solution Implemented**

### **1. Added New Message Type**
```cpp
enum class MessageType {
  // ...existing types...
  DIAGNOSTIC,   ///< Structured diagnostic messages (DIAG)
  LOG,          ///< Log/diagnostic text messages (INFO, WARN, ERROR, etc.)
  // ...
};
```

### **2. Updated Parser Recognition**
```cpp
MessageType MessageParser::StringToMessageType(const std::string& type_str) const {
  // Structured data messages
  if (type_str == "BATT") return MessageType::BATTERY;
  // ...existing mappings...
  
  // Structured diagnostic messages  
  if (type_str == "DIAG") return MessageType::DIAGNOSTIC;
  
  // Log/diagnostic text messages (like rosout)
  if (type_str == "INFO") return MessageType::LOG;
  if (type_str == "WARN") return MessageType::LOG;
  if (type_str == "ERROR") return MessageType::LOG;
  if (type_str == "DEBUG") return MessageType::LOG;
  if (type_str == "CRITICAL") return MessageType::LOG;
  if (type_str == "FATAL") return MessageType::LOG;
  if (type_str == "FAULT") return MessageType::LOG;
  if (type_str == "SAFETY_CRITICAL") return MessageType::LOG;
  if (type_str == "INIT") return MessageType::LOG;
  
  return MessageType::UNKNOWN;
}
```

### **3. Added New Publisher**
```cpp
// New publisher for TeensyV2 diagnostic/log messages (like rosout)
teensy_diagnostics_pub_ = this->create_publisher<std_msgs::msg::String>(
  "~/teensy_v2/diagnostics_out", 10);
```

### **4. Added LOG Message Handling**
```cpp
void TeensyBridge::HandleLogMessage(const MessageData& data, rclcpp::Time timestamp) {
  auto level_it = data.find("level");
  auto message_it = data.find("message");
  
  if (level_it != data.end() && message_it != data.end()) {
    // Format like rosout: "LEVEL:message content"
    std::string formatted_message = level_it->second + ":" + message_it->second;
    
    auto msg = std_msgs::msg::String();
    msg.data = formatted_message;
    teensy_diagnostics_pub_->publish(msg);
    
    // Also log to ROS2 logger with appropriate level
    if (level_it->second == "ERROR" || level_it->second == "FATAL" || 
        level_it->second == "CRITICAL" || level_it->second == "FAULT") {
      RCLCPP_ERROR(this->get_logger(), "TeensyV2 %s: %s", 
                   level_it->second.c_str(), message_it->second.c_str());
    } else if (level_it->second == "WARN" || level_it->second == "SAFETY_CRITICAL") {
      RCLCPP_WARN(this->get_logger(), "TeensyV2 %s: %s", 
                  level_it->second.c_str(), message_it->second.c_str());
    } else if (level_it->second == "INFO" || level_it->second == "INIT") {
      RCLCPP_INFO(this->get_logger(), "TeensyV2 %s: %s", 
                  level_it->second.c_str(), message_it->second.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "TeensyV2 %s: %s", 
                   level_it->second.c_str(), message_it->second.c_str());
    }
  }
}
```

### **5. Added Parser Logic for LOG Messages**
```cpp
} else if (type == MessageType::LOG) {
  // LOG messages are simple text messages from TeensyV2 modules
  // Store the original message type and content for formatted output
  std::string type_str = message.substr(0, colon_pos);
  data["level"] = type_str;
  data["message"] = content;
  RCLCPP_DEBUG(logger_, "%s: %s", type_str.c_str(), content.c_str());
}
```

## **New Topic Created**

### **`~/teensy_v2/diagnostics_out`**
- **Message Type:** `std_msgs/String`
- **Format:** `"LEVEL:message content"` (like rosout)
- **Examples:**
  - `"ERROR:BNO055Monitor: Sensor initialization failed"`
  - `"CRITICAL:Multiple consecutive frequency violations"`
  - `"WARN:Loop execution time exceeded 10ms"`
  - `"FAULT:System fault detected, emergency stop activated"`
  - `"SAFETY_CRITICAL:Battery critical condition detected!"`

## **Dual Logging Strategy**

1. **ROS2 Topic:** All log messages published to `~/teensy_v2/diagnostics_out` 
2. **ROS2 Logger:** Messages also logged with appropriate ROS2 log levels:
   - `RCLCPP_ERROR()` for ERROR, FATAL, CRITICAL, FAULT
   - `RCLCPP_WARN()` for WARN, SAFETY_CRITICAL  
   - `RCLCPP_INFO()` for INFO, INIT
   - `RCLCPP_DEBUG()` for DEBUG and others

## **Result**

✅ **All TeensyV2 diagnostic messages are now captured**  
✅ **Critical safety messages no longer lost**  
✅ **Formatted output similar to rosout**  
✅ **Available on dedicated topic for monitoring tools**  
✅ **Integrated with ROS2 logging system**  

## **Example Messages Now Visible**

From actual TeensyV2 source code:
```cpp
// These were being DROPPED before the fix:
SerialManager::getInstance().sendMessage("ERROR", "BNO055Monitor: I2C multiplexer not available");
SerialManager::getInstance().sendMessage("CRITICAL", "Multiple consecutive frequency violations - system overload detected!");
SerialManager::getInstance().sendMessage("FAULT", fault_msg.c_str());
SerialManager::getInstance().sendMessage("SAFETY_CRITICAL", safety_msg.c_str());
SerialManager::getInstance().sendMessage("WARN", warn_msg);

// Now published as:
// Topic: /sigyn/teensy_bridge/teensy_v2/diagnostics_out
// "ERROR:BNO055Monitor: I2C multiplexer not available"
// "CRITICAL:Multiple consecutive frequency violations - system overload detected!"
// "FAULT:System fault detected, emergency stop activated"
// "SAFETY_CRITICAL:Battery critical condition detected!"
// "WARN:Loop execution time exceeded 10ms (12500 us)"
```

This fix ensures that **no critical diagnostic information from TeensyV2 is lost**, and all messages are available for monitoring, debugging, and safety analysis.
