// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file serial_manager.h
 * @brief Efficient serial communication manager for TeensyV2
 *
 * Provides a single, optimized interface for sending and receiving messages
 * between Teensy boards and the ROS2 system. Focuses on low latency and
 * high throughput for real-time operation.
 *
 * Design Goals:
 * - Single SendMessage() interface for all outgoing communication
 * - Efficient, parseable message format
 * - Non-blocking operations to maintain real-time performance
 * - Message queuing for high throughput scenarios
 * - Error detection and recovery
 *
 * Message Format:
 * All messages use the format: TYPE:key1=val1,key2=val2,...
 *
 * Examples:
 * - BATT:id=0,v=39.8,p=0.82,c=1.2,state=OK
 * - IMU:id=0,qx=0.1,qy=0.2,qz=0.3,qw=0.9
 * - ESTOP:active=true,source=motor,reason=overcurrent
 *
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#ifdef UNIT_TEST
#include "Arduino.h"  // Mock Arduino for testing
#else
#include <Arduino.h>  // Real Arduino SDK
#endif

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

namespace sigyn_teensy {

  /**
   * @brief Serial communication manager for efficient Teensy ↔ ROS2 messaging.
   *
   * Handles all serial communication using a single, optimized interface.
   * Maintains real-time performance through non-blocking operations and
   * efficient message formatting.
   *
   * Key features:
   * - Single SendMessage() method for all outgoing messages
   * - Structured key-value format for reliable parsing
   * - Non-blocking I/O operations
   * - Message queuing for burst scenarios
   * - Incoming message parsing and routing
   *
   * Thread Safety:
   * This class is designed for single-threaded embedded use.
   * All operations should be called from the main loop thread.
   *
   * Example usage:
   * @code
   * SerialManager& serial = SerialManager::GetInstance();
   *
   * // Send battery status
   * serial.SendMessage("BATT", "id=0,v=39.8,p=0.82,c=1.2,state=OK");
   *
   * // Send IMU data
   * serial.SendMessage("IMU", "id=0,qx=0.1,qy=0.2,qz=0.3,qw=0.9");
   *
   * // Process incoming messages (call regularly)
   * serial.ProcessIncomingMessages();
   * @endcode
   */
  class SerialManager {
  public:
    /**
     * @brief Maximum length of a single message including terminator.
      * Increased to 4096 to support larger PERF JSON messages and long SD/DIAG payloads.
     */
        static constexpr size_t kMaxMessageLength = 4096;

    /**
     * @brief Maximum size of the outgoing message queue.
      * Sized to absorb short bursts (e.g., SD directory listings) without
      * blocking the real-time loop.
     */
        static constexpr size_t kMaxQueueSize = 32;

    /**
     * @brief Serial baud rate for communication.
     */
    static constexpr uint32_t kBaudRate = 921600;

    /**
     * @brief Get the singleton instance of SerialManager.
     *
     * @return Reference to the singleton SerialManager instance
     */
    static SerialManager& getInstance();

    /**
     * @brief Initialize serial communication.
     *
     * Sets up the serial port with appropriate settings for high-speed
     * communication. Should be called once during system initialization.
     *
     * @param[in] timeout_ms Maximum time to wait for serial connection
     * (milliseconds)
     */
    void initialize(uint32_t timeout_ms = 2000);

    /**
     * @brief Send a message over the serial interface.
     *
     * Formats and sends a message with a specified type and payload.
     * If the serial port is busy, the message is queued for later transmission.
     *
     * @param[in] type Message type (e.g., "BATT", "IMU")
     * @param[in] payload Message payload (e.g., "id=0,v=39.8")
     */
    void sendMessage(const char* type, const char* payload);

    /**
     * @brief Send a diagnostic message in JSON format.
     *
     * Converts diagnostic messages to standardized JSON format for consistent parsing.
     * All diagnostic messages (INFO, ERROR, WARN, etc.) should use this method.
     *
     * @param[in] level Message level (e.g., "INFO", "ERROR", "WARN")
     * @param[in] module Source module name (e.g., "RoboClawMonitor", "TemperatureMonitor")
     * @param[in] message The diagnostic message content
     */
    void sendDiagnosticMessage(const char* level, const char* module, const char* message);

    /**
     * @brief Process all incoming messages from the serial buffer.
     *
     * Reads available data, parses complete messages, and routes them
     * to the appropriate handlers. Should be called frequently from the
     * main loop to ensure responsive communication.
     */
    void processIncomingMessages();

    /**
     * @brief Drain queued outgoing messages.
     *
     * Call this regularly from the main loop to ensure queued data is
     * transmitted even when no new messages are being produced.
     */
    void processOutgoingMessages();

    /**
     * @brief Set the latest TWIST command for modules to access.
     *
     * @param[in] twist_data The TWIST command data
     */
    void setLatestTwistCommand(const char* twist_data);

    /**
     * @brief Get the latest TWIST command data.
     *
      * @return Pointer to internal TWIST command buffer (never null)
     */
     const char* getLatestTwistCommand() const;

    /**
     * @brief Check if there's a new TWIST command and mark it as processed.
     *
     * @return True if there was a new command, false otherwise
     */
    bool hasNewTwistCommand();

    // --- SD Command Methods ---

    /**
     * @brief Set the latest SDDIR command.
     * @param[in] sddir_data The SDDIR command data
     */
    void setLatestSDDirCommand(const char* sddir_data);

    /**
     * @brief Get the latest SDDIR command.
      * @return Pointer to internal SDDIR command buffer (never null)
     */
     const char* getLatestSDDirCommand() const;

    /**
     * @brief Check if there's a new SDDIR command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewSDDirCommand();

    /**
     * @brief Set the latest SDFILE command.
     * @param[in] sdfile_data The SDFILE command data
     */
    void setLatestSDFileCommand(const char* sdfile_data);

    /**
     * @brief Get the latest SDFILE command.
      * @return Pointer to internal SDFILE command buffer (never null)
     */
     const char* getLatestSDFileCommand() const;

    /**
     * @brief Check if there's a new SDFILE command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewSDFileCommand();

    /**
     * @brief Set the latest SDPRUNE command.
     * @param[in] sdprune_data The SDPRUNE command data (e.g. preserve count)
     */
    void setLatestSDPruneCommand(const char* sdprune_data);

    /**
     * @brief Get the latest SDPRUNE command.
     * @return Pointer to internal SDPRUNE command buffer (never null)
     */
    const char* getLatestSDPruneCommand() const;

    /**
     * @brief Check if there's a new SDPRUNE command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewSDPruneCommand();

    // --- Stepper Motor Command Methods ---

    /**
     * @brief Set the latest STEPPOS command.
     * @param[in] steppos_data The STEPPOS command data (e.g., "elevator:0.15,extender:0.08")
     */
    void setLatestStepPosCommand(const char* steppos_data);

    /**
     * @brief Get the latest STEPPOS command.
     * @return Pointer to internal STEPPOS command buffer (never null)
     */
    const char* getLatestStepPosCommand() const;

    /**
     * @brief Check if there's a new STEPPOS command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewStepPosCommand();

    /**
     * @brief Set the latest STEPHOME command.
     * @param[in] stephome_data The STEPHOME command data (typically empty)
     */
    void setLatestStepHomeCommand(const char* stephome_data);

    /**
     * @brief Get the latest STEPHOME command.
     * @return Pointer to internal STEPHOME command buffer (never null)
     */
    const char* getLatestStepHomeCommand() const;

    /**
     * @brief Check if there's a new STEPHOME command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewStepHomeCommand();

    /**
     * @brief Set the latest STEPSTATUS command.
     * @param[in] stepstatus_data The STEPSTATUS command data (typically empty)
     */
    void setLatestStepStatusCommand(const char* stepstatus_data);

    /**
     * @brief Get the latest STEPSTATUS command.
     * @return Pointer to internal STEPSTATUS command buffer (never null)
     */
    const char* getLatestStepStatusCommand() const;

    /**
     * @brief Check if there's a new STEPSTATUS command available.
     * @return true if a new command is available, false otherwise
     */
    bool hasNewStepStatusCommand();

  private:
    /**
     * @brief Private constructor for singleton pattern.
     */
    SerialManager();

    /**
     * @brief Destructor.
     */
    ~SerialManager() = default;

    // Delete copy constructor and assignment operator
    SerialManager(const SerialManager&) = delete;
    SerialManager& operator=(const SerialManager&) = delete;

    /**
     * @brief Send all queued messages.
     *
     * Transmits messages from the outgoing queue until the queue is empty
     * or the serial port is busy.
     */
    void sendQueuedMessages();

    /**
     * @brief Parse a raw message string into components.
     *
     * @param[in] message The raw message string to parse
     */
    void parseMessage(const char* message);

    /**
     * @brief Handle a parsed command.
     *
     * @param[in] command The command to handle
     * @param[in] args The arguments for the command
     */
    void handleCommand(const char* command, const char* args);

    // --- Member Variables ---

    /**
     * @brief Buffer for incoming serial data.
     */
    char incoming_buffer_[kMaxMessageLength];

    /**
     * @brief Current position in the incoming buffer.
     */
    size_t buffer_pos_ = 0;

    /**
     * @brief Queue for outgoing messages.
     */
    char message_queue_[kMaxQueueSize][kMaxMessageLength];

    /**
     * @brief Head of the message queue.
     */
    size_t queue_head_ = 0;

    /**
     * @brief Tail of the message queue.
     */
    size_t queue_tail_ = 0;

    /**
     * @brief Number of messages currently in the queue.
     */
    size_t queue_count_ = 0;

    /**
     * @brief Byte offset into the current head message being transmitted.
     */
    size_t send_offset_ = 0;

    /**
     * @brief Queue state/counters for diagnostics
     */
    size_t queue_high_water_mark_ = 0;    ///< Max observed queued messages
    uint32_t total_dropped_ = 0;          ///< Total dropped due to full queue
    uint32_t dropped_imu_ = 0;            ///< Dropped IMU messages
    uint32_t dropped_batt_ = 0;           ///< Dropped BATT messages
    uint32_t dropped_perf_ = 0;           ///< Dropped PERF messages
    uint32_t dropped_diag_ = 0;           ///< Dropped DIAG/DEBUG messages
    uint32_t dropped_odom_ = 0;           ///< Dropped ODOM messages
    uint32_t dropped_roboclaw_ = 0;       ///< Dropped ROBOCLAW messages
    uint32_t dropped_safety_ = 0;         ///< Dropped SAFETY messages
    uint32_t dropped_temperature_ = 0;    ///< Dropped TEMPERATURE messages
    uint32_t dropped_vl53l0x_ = 0;        ///< Dropped VL53L0X messages
    uint32_t dropped_sd_ = 0;             ///< Dropped SD-related (SDIR/SDLINE/SDEOF) messages
    uint32_t dropped_other_ = 0;          ///< Dropped other types
    uint32_t last_queue_full_report_ms_ = 0; ///< Last time we emitted QUEUE_FULL

    // --- Command Storage ---

    static constexpr size_t kMaxTwistCommandLen = 256;
    static constexpr size_t kMaxSDDirCommandLen = 256;
    static constexpr size_t kMaxSDFileCommandLen = 256;
    static constexpr size_t kMaxSDPruneCommandLen = 64;
    static constexpr size_t kMaxStepPosCommandLen = 128;
    static constexpr size_t kMaxStepHomeCommandLen = 64;
    static constexpr size_t kMaxStepStatusCommandLen = 64;

    /**
     * @brief Storage for the latest TWIST command.
     */
    char latest_twist_command_[kMaxTwistCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new TWIST command.
     */
    bool has_new_twist_command_ = false;

    /**
     * @brief Storage for the latest SDDIR command.
     */
    char latest_sddir_command_[kMaxSDDirCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new SDDIR command.
     */
    bool has_new_sddir_command_ = false;

    /**
     * @brief Storage for the latest SDFILE command.
     */
    char latest_sdfile_command_[kMaxSDFileCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new SDFILE command.
     */
    bool has_new_sdfile_command_ = false;

    /**
     * @brief Storage for the latest SDPRUNE command.
     */
    char latest_sdprune_command_[kMaxSDPruneCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new SDPRUNE command.
     */
    bool has_new_sdprune_command_ = false;

    /**
     * @brief Storage for the latest STEPPOS command.
     */
    char latest_steppos_command_[kMaxStepPosCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new STEPPOS command.
     */
    bool has_new_steppos_command_ = false;

    /**
     * @brief Storage for the latest STEPHOME command.
     */
    char latest_stephome_command_[kMaxStepHomeCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new STEPHOME command.
     */
    bool has_new_stephome_command_ = false;

    /**
     * @brief Storage for the latest STEPSTATUS command.
     */
    char latest_stepstatus_command_[kMaxStepStatusCommandLen] = {0};

    /**
     * @brief Flag indicating if there's a new STEPSTATUS command.
     */
    bool has_new_stepstatus_command_ = false;

    // Helper to classify message type into counters
    inline void classifyAndCountDrop_(const char* type) {
      if (!type) { dropped_other_++; return; }
      if (strncmp(type, "IMU", 3) == 0) { dropped_imu_++; }
      else if (strncmp(type, "BATT", 4) == 0) { dropped_batt_++; }
      else if (strncmp(type, "PERF", 4) == 0 || strncmp(type, "PERF2", 5) == 0) { dropped_perf_++; }
      else if (strncmp(type, "DIAG", 4) == 0 || strncmp(type, "DEBUG", 5) == 0) { dropped_diag_++; }
      else if (strncmp(type, "ODOM", 4) == 0) { dropped_odom_++; }
      else if (strncmp(type, "ROBOCLAW", 8) == 0) { dropped_roboclaw_++; }
      else if (strncmp(type, "SAFETY", 6) == 0) { dropped_safety_++; }
      else if (strncmp(type, "TEMPERATURE", 11) == 0) { dropped_temperature_++; }
      else if (strncmp(type, "VL53L0X", 7) == 0) { dropped_vl53l0x_++; }
      else if (strncmp(type, "SDIR", 4) == 0 || strncmp(type, "SDLINE", 6) == 0 || strncmp(type, "SDEOF", 5) == 0) { dropped_sd_++; }
      else { dropped_other_++; }
    }

    // Emit a throttled queue full diagnostic
    inline void maybeReportQueueFull_() {
      uint32_t now = millis();
      if (now - last_queue_full_report_ms_ < 1000) return; // throttle to 1 Hz
      last_queue_full_report_ms_ = now;
      char json[512];
      snprintf(json, sizeof(json),
        "{\"qsize\":%u,\"qhwm\":%u,\"dropped\":%lu,\"imu\":%lu,\"batt\":%lu,\"perf\":%lu,\"diag\":%lu,\"odom\":%lu,\"roboclaw\":%lu,\"safety\":%lu,\"temp\":%lu,\"vl53\":%lu,\"sd\":%lu,\"other\":%lu}",
        (unsigned)queue_count_, (unsigned)queue_high_water_mark_,
        (unsigned long)total_dropped_, (unsigned long)dropped_imu_, (unsigned long)dropped_batt_,
        (unsigned long)dropped_perf_, (unsigned long)dropped_diag_, (unsigned long)dropped_odom_,
        (unsigned long)dropped_roboclaw_, (unsigned long)dropped_safety_, (unsigned long)dropped_temperature_,
        (unsigned long)dropped_vl53l0x_, (unsigned long)dropped_sd_, (unsigned long)dropped_other_);
      sendDiagnosticMessage("QUEUE_FULL", "SerialManager", json);
    }
  };

}  // namespace sigyn_teensy
