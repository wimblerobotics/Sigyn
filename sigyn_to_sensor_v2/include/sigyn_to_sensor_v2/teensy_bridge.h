/**
 * @file teensy_bridge.h
 * @brief Main TeensyV2 communication bridge class
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "sigyn_to_sensor_v2/message_parser.h"

namespace sigyn_to_sensor_v2 {

/**
 * @brief Main communication bridge between TeensyV2 and ROS2
 */
class TeensyBridge : public rclcpp::Node {
public:
  TeensyBridge();
  ~TeensyBridge();

private:
  void InitializeParameters();
  void InitializePublishersAndSubscribers();
  void StartSerialCommunication();
  void StopSerialCommunication();
  
  // Serial communication thread
  void SerialReaderThread();
  
  // Message handlers
  void HandleBatteryMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandlePerformanceMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleSafetyMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleEstopMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleDiagnosticMessage(const MessageData& data, rclcpp::Time timestamp);
  
  // ROS2 callbacks
  void EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ConfigCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  
  // Timer callbacks
  void StatusTimerCallback();
  void DiagnosticsTimerCallback();
  
  // Parameters
  std::string board1_port_;
  std::string board2_port_;
  std::string board3_port_;
  int baud_rate_;
  double connection_timeout_;
  double reconnect_timeout_;
  
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_status_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_cmd_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  
  // Message parser
  std::unique_ptr<MessageParser> message_parser_;
  
  // Serial communication
  std::atomic<bool> serial_running_;
  std::thread serial_thread_;
  
  // File descriptors for serial ports
  int board1_fd_;
  int board2_fd_;
  int board3_fd_;
};

}  // namespace sigyn_to_sensor_v2
