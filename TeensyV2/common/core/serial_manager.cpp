/**
 * @file serial_manager.cpp
 * @brief Implementation of efficient serial communication for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "serial_manager.h"

namespace sigyn_teensy {

// Static member definitions
SerialManager* SerialManager::instance_ = nullptr;

SerialManager::SerialManager()
    : is_initialized_(false),
      queue_head_(0),
      queue_tail_(0),
      queue_size_(0),
      callback_count_(0),
      tx_message_count_(0),
      rx_message_count_(0),
      error_count_(0),
      last_activity_ms_(0) {
  incoming_buffer_.reserve(kMaxMessageLength *
                           2);  // Reserve space for efficiency
}

SerialManager& SerialManager::GetInstance() {
  if (instance_ == nullptr) {
    instance_ = new SerialManager();
  }
  return *instance_;
}

bool SerialManager::Initialize(uint32_t timeout_ms) {
  if (is_initialized_) {
    return true;  // Already initialized
  }

  Serial.begin(kBaudRate);

  // Wait for serial connection with timeout
  uint32_t start_time = millis();
  while (!Serial && (millis() - start_time) < timeout_ms) {
    delay(10);  // Brief delay during initialization is acceptable
  }

  is_initialized_ = Serial;
  if (is_initialized_) {
    last_activity_ms_ = millis();
    SendMessage("INIT", "serial_ready=true");
  }

  return is_initialized_;
}

bool SerialManager::SendMessage(const char* message_type, const char* data) {
  // if (!is_initialized_) {
  //   Initialize(1000);  // Ensure initialization
  //   if (!is_initialized_) {
  //     return false;  // Initialization failed
  //   }
  // }

  // Format message: TYPE:data\n
  char message_buffer[kMaxMessageLength];
  int length_buffered = snprintf(message_buffer, sizeof(message_buffer),
                                 "%s:%s\n", message_type, data);

  if (length_buffered < 0 || length_buffered >= (int)sizeof(message_buffer)) {
    error_count_++;
    return false;  // Message too long or formatting error
  }

  size_t bytes_written = Serial.write(message_buffer, (size_t)length_buffered);
  if (bytes_written == (size_t)length_buffered) {
    // Message sent successfully
    tx_message_count_++;
    return true;
  } else {
    // Would block or error occurred
    error_count_++;
    return false;
  }
}

void SerialManager::ProcessIncomingMessages() {
  if (!is_initialized_) {
    return;
  }

  // Process incoming data
  while (Serial.available()) {
    char c = Serial.read();
    last_activity_ms_ = millis();

    if (c == '\n' || c == '\r') {
      // Complete message received
      if (incoming_buffer_.length() > 0) {
        ParseAndRouteMessage(incoming_buffer_);
        incoming_buffer_ = "";
        rx_message_count_++;
      }
    } else if (incoming_buffer_.length() < kMaxMessageLength - 1) {
      // Add character to buffer if there's space
      incoming_buffer_ += c;
    } else {
      // Buffer overflow - discard message and count as error
      incoming_buffer_ = "";
      error_count_++;
    }
  }
}

bool SerialManager::IsConnected() const {
  if (!is_initialized_) {
    return false;
  }

  // // Consider connection healthy if we've had activity recently
  // uint32_t current_time = millis();
  // return (current_time - last_activity_ms_) < 5000;  // 5 second timeout
  return true;
}

void SerialManager::GetStatistics(char* stats_json, size_t buffer_size) const {
  snprintf(stats_json, buffer_size,
           "{\"tx_count\":%lu,\"rx_count\":%lu,\"queue_size\":%zu,\"errors\":%"
           "lu,\"connected\":%s}",
           tx_message_count_, rx_message_count_, queue_size_, error_count_,
           IsConnected() ? "true" : "false");
}

bool SerialManager::RegisterCallback(const char* message_type,
                                     MessageCallback callback) {
  if (callback_count_ >= kMaxCallbacks) {
    return false;  // No space for more callbacks
  }

  // Store callback registration
  strncpy(callbacks_[callback_count_].message_type, message_type,
          sizeof(callbacks_[callback_count_].message_type) - 1);
  callbacks_[callback_count_]
      .message_type[sizeof(callbacks_[callback_count_].message_type) - 1] =
      '\0';
  callbacks_[callback_count_].callback = callback;
  callback_count_++;

  return true;
}

void SerialManager::ParseAndRouteMessage(const String& message) {
  // Parse message format: TYPE:data
  int colon_pos = message.indexOf(':');
  if (colon_pos == -1) {
    error_count_++;
    return;  // Invalid format
  }

  String message_type = message.substring(0, colon_pos);
  String data = message.substring(colon_pos + 1);

  // Route to registered callbacks
  for (size_t i = 0; i < callback_count_; ++i) {
    if (message_type.equals(callbacks_[i].message_type)) {
      callbacks_[i].callback(message_type.c_str(), data.c_str());
      return;  // Message handled
    }
  }

  // Handle common system messages directly
  if (message_type.equals("CMD_VEL")) {
    // Motor velocity commands - route to motor controller
    // This would typically be handled by a registered callback
  } else if (message_type.equals("ESTOP")) {
    // Emergency stop commands - route to safety coordinator
    // This would typically be handled by a registered callback
  } else if (message_type.equals("CONFIG")) {
    // Configuration updates - route to configuration manager
    // This would typically be handled by a registered callback
  } else {
    // Unknown message type
    char unknown_msg[64];
    snprintf(unknown_msg, sizeof(unknown_msg), "type=%s", message_type.c_str());
    SendMessage("UNKNOWN_MSG", unknown_msg);
  }
}

}  // namespace sigyn_teensy
