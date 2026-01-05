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

  } else if ((cmd_len == 7) && (strncmp(command, "SDPRUNE", 7) == 0)) {
    char diag[256] = {0};
    snprintf(diag, sizeof(diag), "SDPRUNE command received: %s", args ? args : "");
    sendDiagnosticMessage("DEBUG", "SerialManager", diag);
    // Store SDPRUNE command for SDLogger to process
    setLatestSDPruneCommand(args ? args : "");

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

void SerialManager::setLatestSDPruneCommand(const char* sdprune_data) {
  snprintf(latest_sdprune_command_, sizeof(latest_sdprune_command_), "%s", sdprune_data ? sdprune_data : "");
  has_new_sdprune_command_ = true;
}

const char* SerialManager::getLatestSDPruneCommand() const { return latest_sdprune_command_; }

bool SerialManager::hasNewSDPruneCommand() {
  if (has_new_sdprune_command_) {
    has_new_sdprune_command_ = false;
    return true;
  }
  return false;
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
  // Format full line (including newline) into a temporary buffer.
  char line[kMaxMessageLength];
  const int written = snprintf(line, sizeof(line), "%s%d:%s\n", type, BOARD_ID, payload ? payload : "");

  // If the message would be truncated, do not send a partial line. Partial lines can:
  // - omit the terminating newline (gluing the next message onto this one)
  // - cut JSON escape sequences in half (e.g. trailing '\\')
  // which manifests as JSON parse errors on the ROS side.
  if (written < 0 || static_cast<size_t>(written) >= sizeof(line)) {
    char diag_line[256] = {0};
    const unsigned long now_ms = static_cast<unsigned long>(millis());
    const size_t payload_len = payload ? strlen(payload) : 0U;
    const unsigned long required = (written > 0) ? static_cast<unsigned long>(written) : 0UL;
    const unsigned long max_bytes = static_cast<unsigned long>(sizeof(line) - 1U);
    // Emit a compact error that always fits and always ends with '\n'.
    (void)snprintf(
      diag_line, sizeof(diag_line),
      "DIAG%d:{\"level\":\"ERROR\",\"module\":\"SerialManager\",\"message\":\"Dropped overlong %s payload_len=%lu required=%lu max=%lu\",\"timestamp\":%lu}\n",
      BOARD_ID,
      type ? type : "",
      static_cast<unsigned long>(payload_len),
      required,
      max_bytes,
      now_ms);
    Serial.print(diag_line);
    return;
  }

#if ENABLE_SD_LOGGING
  SDLogger::getInstance().logFormatted("%s%d:%s", type, BOARD_ID, payload);
#endif

  // Best effort to drain any backlog before enqueueing more.
  sendQueuedMessages();

  // Enqueue the line for non-blocking transmit.
  if (queue_count_ >= kMaxQueueSize) {
    total_dropped_++;
    classifyAndCountDrop_(type);
    maybeReportQueueFull_();
    return;
  }

  // Copy the full line (including trailing '\n' and terminating NUL).
  const size_t copy_len = static_cast<size_t>(written);
  const size_t slot = queue_tail_;
  memcpy(message_queue_[slot], line, copy_len);
  message_queue_[slot][copy_len] = '\0';

  queue_tail_ = (queue_tail_ + 1) % kMaxQueueSize;
  queue_count_++;
  if (queue_count_ > queue_high_water_mark_) {
    queue_high_water_mark_ = queue_count_;
  }

  // Opportunistically transmit immediately.
  sendQueuedMessages();
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

  const unsigned long timestamp_ms = static_cast<unsigned long>(millis());

  // Create JSON diagnostic message
  const int diag_written = snprintf(
    json_payload,
    sizeof(json_payload),
    "{\"level\":\"%s\",\"module\":\"%s\",\"message\":\"%s\",\"timestamp\":%lu}",
    level,
    module,
    escaped_message,
    timestamp_ms);

  // If snprintf truncated the diagnostic JSON, it will almost certainly be invalid JSON.
  // Never emit an invalid JSON frame: it can poison the ROS-side parser and cause
  // subsequent messages to be misinterpreted as "chunks".
  if (diag_written < 0 || static_cast<size_t>(diag_written) >= sizeof(json_payload)) {
    char diag_line[256] = {0};
    const unsigned long max_bytes = static_cast<unsigned long>(kMaxMessageLength - 1U);
    const unsigned long payload_max = static_cast<unsigned long>(sizeof(json_payload) - 1U);
    char module_short[32] = {0};
    char level_short[8] = {0};
    snprintf(module_short, sizeof(module_short), "%s", module ? module : "");
    snprintf(level_short, sizeof(level_short), "%s", level ? level : "");
    (void)snprintf(
      diag_line,
      sizeof(diag_line),
      "DIAG%d:{\"level\":\"ERROR\",\"module\":\"SerialManager\",\"message\":\"Dropped truncated DIAG origin=%s/%s payload_max=%lu max=%lu\",\"timestamp\":%lu}\n",
      BOARD_ID,
      module_short,
      level_short,
      payload_max,
      max_bytes,
      timestamp_ms);
    Serial.print(diag_line);
    return;
  }

  // Guardrail: if this DIAG frame would be too large, don't call sendMessage("DIAG", ...)
  // because the fallback would only tell us "Dropped overlong DIAG..." without the origin.
  {
    const size_t payload_len = strlen(json_payload);
    size_t digits = 1U;
    unsigned int tmp = static_cast<unsigned int>(BOARD_ID);
    while (tmp >= 10U) {
      tmp /= 10U;
      ++digits;
    }
    // Required bytes excluding NUL terminator.
    const size_t required = strlen("DIAG") + digits + 1U + payload_len + 1U;
    if (required >= kMaxMessageLength) {
      char diag_line[256] = {0};
      const unsigned long max_bytes = static_cast<unsigned long>(kMaxMessageLength - 1U);
      const unsigned long req = static_cast<unsigned long>(required);
      char module_short[32] = {0};
      char level_short[8] = {0};
      snprintf(module_short, sizeof(module_short), "%s", module ? module : "");
      snprintf(level_short, sizeof(level_short), "%s", level ? level : "");
      (void)snprintf(
        diag_line, sizeof(diag_line),
        "DIAG%d:{\"level\":\"ERROR\",\"module\":\"SerialManager\",\"message\":\"Dropped overlong DIAG origin=%s/%s payload_len=%lu required=%lu max=%lu\",\"timestamp\":%lu}\n",
        BOARD_ID,
        module_short,
        level_short,
        static_cast<unsigned long>(payload_len),
        req,
        max_bytes,
        timestamp_ms);
      Serial.print(diag_line);
      return;
    }
  }

  sendMessage("DIAG", json_payload);
}

void SerialManager::sendQueuedMessages() {
  while (queue_count_ > 0) {
    char* msg = message_queue_[queue_head_];
    const size_t msg_len = strnlen(msg, kMaxMessageLength);
    if (msg_len == 0) {
      // Defensive: empty slot; drop it.
      queue_head_ = (queue_head_ + 1) % kMaxQueueSize;
      queue_count_--;
      send_offset_ = 0;
      continue;
    }

    if (send_offset_ >= msg_len) {
      // Finished this message.
      queue_head_ = (queue_head_ + 1) % kMaxQueueSize;
      queue_count_--;
      send_offset_ = 0;
      continue;
    }

    const int avail = Serial.availableForWrite();
    if (avail <= 0) {
      break;
    }

    const size_t remaining = msg_len - send_offset_;
    const size_t to_write = (remaining < static_cast<size_t>(avail)) ? remaining : static_cast<size_t>(avail);
    const size_t wrote = Serial.write(reinterpret_cast<const uint8_t*>(msg + send_offset_), to_write);
    if (wrote == 0) {
      break;
    }
    send_offset_ += wrote;
  }
}

void SerialManager::processOutgoingMessages() {
  sendQueuedMessages();
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
