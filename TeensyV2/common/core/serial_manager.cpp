// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file serial_manager.cpp
 * @brief Implementation of efficient serial communication for TeensyV2
 *
 * This file implements the SerialManager class that handles all serial
 * communication between the Teensy microcontrollers and the ROS2 system.
 * The implementation focuses on low-latency, high-throughput communication
 * with structured message formats suitable for real-time robotics applications.
 *
 * Key Implementation Features:
 *
 * **Message Format:**
 * All messages use a structured key-value format for reliable parsing:
 * - Format: TYPE:key1=val1,key2=val2,...
 * - Examples: BATT:id=0,v=39.8,p=0.82,c=1.2,state=OK
 * - Benefits: Human-readable, robust parsing, easy extension
 *
 * **Performance Optimizations:**
 * - Non-blocking I/O operations to maintain real-time performance
 * - Efficient string handling with fixed-size buffers
 * - Message queuing for burst scenarios without blocking
 * - Minimal memory allocation during operation
 *
 * **Communication Strategy:**
 * - Outgoing: Immediate transmission with optional queuing
 * - Incoming: Line-based parsing with command dispatch
 * - Error handling: Robust parsing with graceful degradation
 * - Flow control: Automatic throttling during overload conditions
 *
 * **Integration Points:**
 * - Module system: Automatic integration with all Module subclasses
 * - Safety system: High-priority safety message transmission
 * - Performance monitoring: Minimal impact on measured timings
 * - Configuration: Runtime parameter updates via incoming commands
 *
 * The implementation provides a clean, efficient interface while maintaining
 * the real-time characteristics essential for robotic control systems.
 *
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 */

#include "serial_manager.h"

#include "config.h"
#if ENABLE_SD_LOGGING
#include "modules/storage/sd_logger.h"
#endif
#if ENABLE_SAFETY
#include "modules/safety/safety_coordinator.h"
#endif

#include <cstdio>
#include <cstring>

namespace sigyn_teensy {

SerialManager::SerialManager() : buffer_pos_(0), queue_head_(0), queue_tail_(0), queue_count_(0) {
  memset(incoming_buffer_, 0, sizeof(incoming_buffer_));
  memset(message_queue_, 0, sizeof(message_queue_));
}

SerialManager& SerialManager::getInstance() {
  static SerialManager instance;
  return instance;
}

void SerialManager::handleCommand(const char* command, const char* args) {
  if (!command) {
    return;
  }

  // Parse incoming commands and route them to appropriate modules
  const char* colon = strchr(command, ':');
  const size_t cmd_len = colon ? static_cast<size_t>(colon - command) : strlen(command);

  char cmd_type[16] = {0};
  const size_t cmd_copy_len = (cmd_len < (sizeof(cmd_type) - 1)) ? cmd_len : (sizeof(cmd_type) - 1);
  if (cmd_copy_len > 0) {
    memcpy(cmd_type, command, cmd_copy_len);
    cmd_type[cmd_copy_len] = '\0';
  }

  if ((cmd_len == 5) && (strncmp(command, "TWIST", 5) == 0)) {
    // Store TWIST command for RoboClawMonitor to process
    // Use a simple approach: send the command data as a special message type
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "TWIST command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);

    // We'll create a mechanism for modules to check for commands
    // For now, set a static variable that RoboClawMonitor can check
    setLatestTwistCommand(args ? args : "");

  } else if ((cmd_len == 5) && (strncmp(command, "SDDIR", 5) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "SDDIR command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // Store SDDIR command for SDLogger to process
    setLatestSDDirCommand(args ? args : "");

  } else if ((cmd_len == 6) && (strncmp(command, "SDFILE", 6) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "SDFILE command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // Store SDFILE command for SDLogger to process
    setLatestSDFileCommand(args ? args : "");

  } else if ((cmd_len == 6) && (strncmp(command, "CONFIG", 6) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "CONFIG command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // TODO: Implement configuration updates

  } else if ((cmd_len == 6) && (strncmp(command, "STATUS", 6) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "STATUS request received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // TODO: Send comprehensive status report

  } else if ((cmd_len == 5) && (strncmp(command, "ESTOP", 5) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "ESTOP command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
#if ENABLE_SAFETY
    SafetyCoordinator::getInstance().setEstopCommand(args);
#endif
  } else if ((cmd_len == 9) && (strncmp(command, "CALIBRATE", 9) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "CALIBRATE command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // TODO: Route to appropriate sensor module

  } else {
    char diag[64] = {0};
    snprintf(diag, sizeof(diag), "Unknown command type: %s", cmd_type);
    sendDiagnosticMessage("ERROR", "SerialManager", diag);
  }
}

void SerialManager::initialize(uint32_t timeout_ms) {
  Serial.begin(kBaudRate);
  uint32_t start_time = millis();

  // Wait for serial connection with timeout
  // This prevents infinite blocking on boards without USB connection
  while (!Serial && (millis() - start_time) < timeout_ms) {
    // Non-blocking wait for serial to become ready
    // Essential for autonomous operation where USB may not be connected
  }
}

void SerialManager::parseMessage(const char* message) {
  const char* colon = strchr(message, ':');
  if (colon) {
    // Parse structured message format: "COMMAND:args"
    // This design allows for extensible command handling while
    // maintaining compatibility with existing ROS2 message parsing
    handleCommand(message, colon + 1);
  }
  // Note: Invalid messages are silently ignored to prevent
  // disruption from malformed or corrupted serial data
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
}

void SerialManager::sendMessage(const char* type, const char* payload) {
  // Revert to immediate send: format line and write directly to USB CDC
  char line[kMaxMessageLength];
  snprintf(line, sizeof(line), "%s%d:%s\n", type, BOARD_ID, payload);

#if ENABLE_SD_LOGGING
  SDLogger::getInstance().logFormatted("%s%d:%s", type, BOARD_ID, payload);
#endif

  Serial.print(line);
}

void SerialManager::sendDiagnosticMessage(const char* level, const char* module, const char* message) {
  char json_payload[kMaxMessageLength - 20];  // Reserve space for type and board ID

  // Escape quotes and control characters in the message for JSON safety
  char escaped_message[kMaxMessageLength / 2];
  const char* src = message;
  char* dst = escaped_message;
  size_t max_escaped = sizeof(escaped_message) - 1;

  while (*src && (dst - escaped_message) < max_escaped - 2) {  // -2 for potential escape sequences
    if (*src == '"') {
      *dst++ = '\\';
      if ((dst - escaped_message) < max_escaped - 1) {
        *dst++ = '"';
      }
    } else if (*src == '\t') {
      *dst++ = '\\';
      if ((dst - escaped_message) < max_escaped - 1) {
        *dst++ = 't';
      }
    } else if (*src == '\n') {
      *dst++ = '\\';
      if ((dst - escaped_message) < max_escaped - 1) {
        *dst++ = 'n';
      }
    } else if (*src == '\r') {
      *dst++ = '\\';
      if ((dst - escaped_message) < max_escaped - 1) {
        *dst++ = 'r';
      }
    } else if (*src == '\\') {
      *dst++ = '\\';
      if ((dst - escaped_message) < max_escaped - 1) {
        *dst++ = '\\';
      }
    } else {
      *dst++ = *src;
    }
    src++;
  }
  *dst = '\0';

  // Create JSON diagnostic message
  snprintf(json_payload, sizeof(json_payload),
           "{\"level\":\"%s\",\"module\":\"%s\",\"message\":\"%s\",\"timestamp\":%lu}", level, module, escaped_message,
           static_cast<unsigned long>(millis()));

  sendMessage("DIAG", json_payload);
}

void SerialManager::sendQueuedMessages() {
  // No-op: immediate send path (queue disabled)
}

void SerialManager::setLatestTwistCommand(const char* twist_data) {
  snprintf(latest_twist_command_, sizeof(latest_twist_command_), "%s", twist_data ? twist_data : "");
  has_new_twist_command_ = true;
}

const char* SerialManager::getLatestTwistCommand() const { return latest_twist_command_; }

bool SerialManager::hasNewTwistCommand() {
  if (has_new_twist_command_) {
    has_new_twist_command_ = false;  // Mark as processed
    return true;
  }
  return false;
}

void SerialManager::setLatestSDDirCommand(const char* sddir_data) {
  snprintf(latest_sddir_command_, sizeof(latest_sddir_command_), "%s", sddir_data ? sddir_data : "");
  has_new_sddir_command_ = true;
}

const char* SerialManager::getLatestSDDirCommand() const { return latest_sddir_command_; }

bool SerialManager::hasNewSDDirCommand() {
  if (has_new_sddir_command_) {
    has_new_sddir_command_ = false;  // Mark as processed
    return true;
  }
  return false;
}

void SerialManager::setLatestSDFileCommand(const char* sdfile_data) {
  snprintf(latest_sdfile_command_, sizeof(latest_sdfile_command_), "%s", sdfile_data ? sdfile_data : "");
  has_new_sdfile_command_ = true;
}

const char* SerialManager::getLatestSDFileCommand() const { return latest_sdfile_command_; }

bool SerialManager::hasNewSDFileCommand() {
  if (has_new_sdfile_command_) {
    has_new_sdfile_command_ = false;  // Mark as processed
    return true;
  }
  return false;
}

}  // namespace sigyn_teensy
