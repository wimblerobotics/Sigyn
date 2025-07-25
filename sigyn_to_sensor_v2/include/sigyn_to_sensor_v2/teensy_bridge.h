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
#include <mutex>
#include <queue>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "sigyn_interfaces/srv/teensy_sd_get_dir.hpp"
#include "sigyn_interfaces/srv/teensy_sd_get_file.hpp"

#include "sigyn_to_sensor_v2/message_parser.h"

namespace sigyn_to_sensor_v2 {

// Structure to hold pending service requests
struct PendingServiceRequest {
  std::string request_id;
  std::string service_type; // "get_dir" or "get_file"
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> dir_response;
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> file_response;
  rclcpp::Time request_time;
  rclcpp::Time last_activity_time; // Last time we received data for this request
  std::shared_ptr<std::promise<bool>> completion_promise;
  std::string accumulated_content; // For file dump responses
};

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
  void HandleIMUMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleEstopMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleDiagnosticMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleTemperatureMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleVL53L0XMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleRoboClawMessage(const MessageData& data, rclcpp::Time timestamp);
  void HandleOdomMessage(const MessageData& data, rclcpp::Time timestamp);
  
  // ROS2 callbacks
  void EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ConfigCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  // Service handlers
  void HandleSdGetDirRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response);
  void HandleSdGetFileRequest(
      const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response);
  
  // Serial message handlers for service responses
  void HandleSdirResponse(const std::string& data);
  void HandleSdlineResponse(const std::string& data);
  void HandleSdeofResponse(const std::string& data);
  
  // Outgoing message queue management
  void SendQueuedMessages();
  
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
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_sensor0_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_sensor1_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_motor0_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_motor1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor0_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor3_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor4_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor5_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor6_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr vl53l0x_sensor7_pub_;
  
  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Services
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetDir>::SharedPtr sd_getdir_service_;
  rclcpp::Service<sigyn_interfaces::srv::TeensySdGetFile>::SharedPtr sd_getfile_service_;
  
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  
  // Message parser
  std::unique_ptr<MessageParser> message_parser_;
  
  // Serial communication
  std::atomic<bool> serial_running_;
  std::thread serial_thread_;
  
  // Outgoing message queue
  std::queue<std::string> outgoing_message_queue_;
  std::mutex outgoing_queue_mutex_;
  
  // Service request management
  std::queue<PendingServiceRequest> pending_service_requests_;
  std::mutex service_mutex_;
  
  // File descriptors for serial ports
  int board1_fd_;
  int board2_fd_;
  int board3_fd_;
};

}  // namespace sigyn_to_sensor_v2
