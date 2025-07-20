/**
 * @file serial_manager.cpp
 * @brief Implementation of efficient serial communication for TeensyV2
 *
 * @author Wimble Robotics
 * @date 2025
 */

#include "serial_manager.h"

namespace sigyn_teensy {

SerialManager::SerialManager()
    : buffer_pos_(0), queue_head_(0), queue_tail_(0), queue_count_(0) {}

SerialManager& SerialManager::getInstance() {
  static SerialManager instance;
  return instance;
}

void SerialManager::handleCommand(const char* command, const char* args) {
  // Placeholder for command handling logic
  // In a real implementation, this would parse 'args' and trigger actions
}

void SerialManager::initialize(uint32_t timeout_ms) {
  Serial.begin(kBaudRate);
  uint32_t start_time = millis();
  while (!Serial && (millis() - start_time) < timeout_ms) {
    // Wait for serial connection
  }
}

void SerialManager::parseMessage(const char* message) {
  const char* colon = strchr(message, ':');
  if (colon) {
    // Simple parsing for "COMMAND:args" format
    // In a real implementation, you might have more robust parsing
    // and a way to register handlers for different commands.
    handleCommand(message, colon + 1);
  }
}

void SerialManager::processIncomingMessages() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      incoming_buffer_[buffer_pos_] = '\0';
      if (buffer_pos_ > 0) {
        parseMessage(incoming_buffer_);
      }
      buffer_pos_ = 0;
    } else if (buffer_pos_ < kMaxMessageLength - 1) {
      incoming_buffer_[buffer_pos_++] = c;
    }
  }
  sendQueuedMessages();
}

void SerialManager::sendMessage(const char* type, const char* payload) {
  if (queue_count_ < kMaxQueueSize) {
    snprintf(message_queue_[queue_tail_], kMaxMessageLength, "%s:%s\n", type,
             payload);
    queue_tail_ = (queue_tail_ + 1) % kMaxQueueSize;
    queue_count_++;
  }
  sendQueuedMessages();
}

void SerialManager::sendQueuedMessages() {
  while (queue_count_ > 0 && Serial.availableForWrite() > 0) {
    Serial.print(message_queue_[queue_head_]);
    queue_head_ = (queue_head_ + 1) % kMaxQueueSize;
    queue_count_--;
  }
}

}  // namespace sigyn_teensy
