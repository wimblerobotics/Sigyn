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
 * @author Sigyn Robotics
 * @date 2025
 */

#pragma once

#include "Arduino.h"

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
  static constexpr size_t kMaxMessageLength = 256;

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
  static SerialManager& GetInstance();

  /**
   * @brief Initialize serial communication.
   * 
   * Sets up the serial port with appropriate settings for high-speed
   * communication. Should be called once during system initialization.
   * 
   * @param[in] timeout_ms Maximum time to wait for serial connection (milliseconds)
   * @return true if initialization successful, false otherwise
   */
  bool Initialize(uint32_t timeout_ms = 5000);

  /**
   * @brief Send a structured message to the ROS2 system.
   * 
   * This is the primary interface for all outgoing communication.
   * Messages are queued if necessary to maintain real-time performance.
   * 
   * Message format: TYPE:key1=val1,key2=val2,...
   * 
   * @param[in] message_type Type identifier (e.g., "BATT", "IMU", "ESTOP")
   * @param[in] data Key-value pairs in format "key1=val1,key2=val2,..."
   * @return true if message queued successfully, false if queue full
   * 
   * @note Message types should be short (≤8 characters) for efficiency
   * @note Total message length must be ≤ kMaxMessageLength
   * 
   * Example:
   * @code
   * SendMessage("BATT", "id=0,v=39.8,p=0.82,c=1.2,state=OK");
   * @endcode
   */
  bool SendMessage(const char* message_type, const char* data);

  /**
   * @brief Process incoming messages from ROS2 system.
   * 
   * Reads available serial data, parses complete messages, and routes
   * them to appropriate handlers. Should be called regularly from the
   * main loop to maintain communication responsiveness.
   * 
   * This method is non-blocking and will process all available data
   * without waiting for additional input.
   */
  void ProcessIncomingMessages();

  /**
   * @brief Flush any queued outgoing messages.
   * 
   * Sends as many queued messages as possible without blocking.
   * Called automatically by ProcessIncomingMessages(), but can be
   * called explicitly if needed.
   */
  void FlushOutgoingQueue();

  /**
   * @brief Check if serial connection is active and healthy.
   * 
   * @return true if serial connection is ready for communication
   */
  bool IsConnected() const;

  /**
   * @brief Get statistics about message throughput and queue status.
   * 
   * @param[out] stats_json Buffer to write JSON-formatted statistics
   * @param[in] buffer_size Size of output buffer
   * 
   * Format: {"tx_count":1234,"rx_count":567,"queue_size":2,"errors":0}
   */
  void GetStatistics(char* stats_json, size_t buffer_size) const;

  /**
   * @brief Register a callback for specific incoming message types.
   * 
   * Allows modules to receive notification when specific message types
   * are received from the ROS2 system.
   * 
   * @param[in] message_type Type to listen for (e.g., "CMD_VEL", "CONFIG")
   * @param[in] callback Function to call when message is received
   * 
   * @note Maximum of 16 callbacks can be registered
   */
  typedef void (*MessageCallback)(const char* message_type, const char* data);
  bool RegisterCallback(const char* message_type, MessageCallback callback);

 private:
  /**
   * @brief Private constructor for singleton pattern.
   */
  SerialManager();

  /**
   * @brief Structure for queued outgoing messages.
   */
  struct QueuedMessage {
    char buffer[kMaxMessageLength];
    size_t length;
  };

  /**
   * @brief Structure for registered message callbacks.
   */
  struct CallbackRegistration {
    char message_type[16];
    MessageCallback callback;
  };

  /**
   * @brief Parse a complete incoming message and route to handlers.
   * 
   * @param[in] message Complete message string (without newline)
   */
  void ParseAndRouteMessage(const String& message);

  /**
   * @brief Send a single message immediately if possible.
   * 
   * @param[in] message_buffer Complete formatted message
   * @param[in] message_length Length of message
   * @return true if sent successfully, false if would block
   */
  bool SendImmediateMessage(const char* message_buffer, size_t message_length);

  /**
   * @brief Add message to outgoing queue.
   * 
   * @param[in] message_buffer Complete formatted message
   * @param[in] message_length Length of message
   * @return true if queued successfully, false if queue full
   */
  bool QueueMessage(const char* message_buffer, size_t message_length);

  // Singleton instance
  static SerialManager* instance_;

  // Communication state
  bool is_initialized_;
  String incoming_buffer_;

  // Message queuing
  QueuedMessage outgoing_queue_[kMaxQueueSize];
  size_t queue_head_;  ///< Index of next message to send
  size_t queue_tail_;  ///< Index where next message will be queued
  size_t queue_size_;  ///< Current number of messages in queue

  // Message callbacks
  static constexpr size_t kMaxCallbacks = 16;
  CallbackRegistration callbacks_[kMaxCallbacks];
  size_t callback_count_;

  // Statistics
  uint32_t tx_message_count_;
  uint32_t rx_message_count_;
  uint32_t error_count_;
  uint32_t last_activity_ms_;

  // Prevent copying
  SerialManager(const SerialManager&) = delete;
  SerialManager& operator=(const SerialManager&) = delete;
};

}  // namespace sigyn_teensy
