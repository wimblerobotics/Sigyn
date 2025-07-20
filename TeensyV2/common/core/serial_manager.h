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

#include <Arduino.h>
#include <cstdint>
#include <cstddef>
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
   */
  static constexpr size_t kMaxMessageLength = 512;

  /**
   * @brief Maximum size of the outgoing message queue.
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
   * @brief Process all incoming messages from the serial buffer.
   *
   * Reads available data, parses complete messages, and routes them
   * to the appropriate handlers. Should be called frequently from the
   * main loop to ensure responsive communication.
   */
  void processIncomingMessages();

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
};

}  // namespace sigyn_teensy
