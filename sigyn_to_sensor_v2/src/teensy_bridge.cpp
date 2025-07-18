/**
 * @file teensy_bridge.cpp
 * @brief Implementation of TeensyV2 communication bridge
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "sigyn_to_sensor_v2/teensy_bridge.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>

using namespace std::chrono_literals;

namespace sigyn_to_sensor_v2 {

TeensyBridge::TeensyBridge() 
    : Node("teensy_bridge"),
      serial_running_(false),
      board1_fd_(-1),
      board2_fd_(-1) {
  
  RCLCPP_INFO(this->get_logger(), "Initializing TeensyV2 Bridge Node");
  
  InitializeParameters();
  InitializePublishersAndSubscribers();
  
  // Initialize message parser
  message_parser_ = std::make_unique<MessageParser>(this->get_logger());
  
  // Register message callbacks
  message_parser_->RegisterCallback(MessageType::BATTERY,
    [this](const MessageData& data, rclcpp::Time timestamp) {
      HandleBatteryMessage(data, timestamp);
    });
    
  message_parser_->RegisterCallback(MessageType::PERFORMANCE,
    [this](const MessageData& data, rclcpp::Time timestamp) {
      HandlePerformanceMessage(data, timestamp);
    });
    
  message_parser_->RegisterCallback(MessageType::SAFETY,
    [this](const MessageData& data, rclcpp::Time timestamp) {
      HandleSafetyMessage(data, timestamp);
    });
    
  message_parser_->RegisterCallback(MessageType::ESTOP,
    [this](const MessageData& data, rclcpp::Time timestamp) {
      HandleEstopMessage(data, timestamp);
    });
    
  message_parser_->RegisterCallback(MessageType::DIAGNOSTIC,
    [this](const MessageData& data, rclcpp::Time timestamp) {
      HandleDiagnosticMessage(data, timestamp);
    });
  
  // Start serial communication
  StartSerialCommunication();
  
  RCLCPP_INFO(this->get_logger(), "TeensyV2 Bridge Node initialization complete");
}

TeensyBridge::~TeensyBridge() {
  StopSerialCommunication();
}

void TeensyBridge::InitializeParameters() {
  // Declare parameters with default values
  this->declare_parameter("board1_port", "/dev/ttyACM0");
  this->declare_parameter("board2_port", "/dev/ttyACM1");
  this->declare_parameter("baud_rate", 921600);
  this->declare_parameter("connection_timeout", 5.0);
  this->declare_parameter("reconnect_timeout", 2.0);
  
  // Get parameter values
  board1_port_ = this->get_parameter("board1_port").as_string();
  board2_port_ = this->get_parameter("board2_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  connection_timeout_ = this->get_parameter("connection_timeout").as_double();
  reconnect_timeout_ = this->get_parameter("reconnect_timeout").as_double();
  
  RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
  RCLCPP_INFO(this->get_logger(), "  Board 1 port: %s", board1_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Board 2 port: %s", board2_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Baud rate: %d", baud_rate_);
}

void TeensyBridge::InitializePublishersAndSubscribers() {
  // Publishers
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
    "~/battery/status", 10);
    
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/diagnostics", 10);
    
  estop_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "~/safety/estop_status", 10);
  
  // Subscribers
  estop_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "~/commands/estop", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      EstopCommandCallback(msg);
    });
    
  config_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
    "~/commands/config", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      ConfigCommandCallback(msg);
    });
  
  // Timers
  status_timer_ = this->create_wall_timer(
    1000ms, [this]() { StatusTimerCallback(); });
    
  diagnostics_timer_ = this->create_wall_timer(
    5000ms, [this]() { DiagnosticsTimerCallback(); });
}

void TeensyBridge::StartSerialCommunication() {
  if (serial_running_) {
    return;
  }
  
  serial_running_ = true;
  serial_thread_ = std::thread(&TeensyBridge::SerialReaderThread, this);
  
  RCLCPP_INFO(this->get_logger(), "Serial communication started");
}

void TeensyBridge::StopSerialCommunication() {
  if (!serial_running_) {
    return;
  }
  
  serial_running_ = false;
  
  if (serial_thread_.joinable()) {
    serial_thread_.join();
  }
  
  if (board1_fd_ >= 0) {
    close(board1_fd_);
    board1_fd_ = -1;
  }
  
  if (board2_fd_ >= 0) {
    close(board2_fd_);
    board2_fd_ = -1;
  }
  
  RCLCPP_INFO(this->get_logger(), "Serial communication stopped");
}

void TeensyBridge::SerialReaderThread() {
  char buffer[1024];
  std::string line_buffer;
  
  while (serial_running_) {
    // Try to open serial ports if not connected
    if (board1_fd_ < 0) {
      board1_fd_ = open(board1_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (board1_fd_ >= 0) {
        // Configure serial port
        struct termios tty;
        if (tcgetattr(board1_fd_, &tty) == 0) {
          cfsetospeed(&tty, B921600);
          cfsetispeed(&tty, B921600);
          tty.c_cflag |= (CLOCAL | CREAD);
          tty.c_cflag &= ~PARENB;
          tty.c_cflag &= ~CSTOPB;
          tty.c_cflag &= ~CSIZE;
          tty.c_cflag |= CS8;
          tty.c_iflag &= ~(IXON | IXOFF | IXANY);
          tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
          tty.c_oflag &= ~OPOST;
          tcsetattr(board1_fd_, TCSANOW, &tty);
          
          RCLCPP_INFO(this->get_logger(), "Connected to Board 1: %s", board1_port_.c_str());
        }
      }
    }
    
    // Read from Board 1
    if (board1_fd_ >= 0) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(board1_fd_, &read_fds);
      
      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;  // 100ms timeout
      
      int result = select(board1_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
      
      if (result > 0 && FD_ISSET(board1_fd_, &read_fds)) {
        ssize_t bytes_read = read(board1_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
          buffer[bytes_read] = '\0';
          line_buffer += buffer;
          
          // Process complete lines
          size_t pos;
          while ((pos = line_buffer.find('\n')) != std::string::npos) {
            std::string line = line_buffer.substr(0, pos);
            line_buffer.erase(0, pos + 1);
            
            if (!line.empty()) {
              // Parse message
              auto timestamp = this->now();
              if (!message_parser_->ParseMessage(line, timestamp)) {
                RCLCPP_WARN(this->get_logger(), "Failed to parse message: %s", line.c_str());
              }
            }
          }
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Board 1 read error: %s", strerror(errno));
          close(board1_fd_);
          board1_fd_ = -1;
        }
      } else if (result < 0) {
        RCLCPP_WARN(this->get_logger(), "Board 1 select error: %s", strerror(errno));
        close(board1_fd_);
        board1_fd_ = -1;
      }
    }
    
    // Small delay to prevent busy waiting
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void TeensyBridge::HandleBatteryMessage(const MessageData& data, rclcpp::Time timestamp) {
  auto battery_data = message_parser_->ParseBatteryData(data);
  if (battery_data.valid) {
    auto msg = message_parser_->ToBatteryStateMsg(battery_data, timestamp);
    battery_pub_->publish(msg);
  }
}

void TeensyBridge::HandlePerformanceMessage(const MessageData& data, rclcpp::Time timestamp) {
  // For now, just log performance data
  // In a complete implementation, this would publish to a custom performance message
  auto it = data.find("freq");
  if (it != data.end()) {
    RCLCPP_DEBUG(this->get_logger(), "Performance: freq=%s Hz", it->second.c_str());
  }
}

void TeensyBridge::HandleSafetyMessage(const MessageData& data, rclcpp::Time timestamp) {
  // Extract safety state and publish E-stop status
  auto state_it = data.find("state");
  auto conditions_it = data.find("active_conditions");
  
  if (state_it != data.end() && conditions_it != data.end()) {
    bool estop_active = (state_it->second == "ESTOP" || state_it->second == "SHUTDOWN") ||
                       (conditions_it->second == "true");
    
    auto msg = std_msgs::msg::Bool();
    msg.data = estop_active;
    estop_status_pub_->publish(msg);
  }
}

void TeensyBridge::HandleEstopMessage(const MessageData& data, rclcpp::Time timestamp) {
  // Log E-stop events
  auto active_it = data.find("active");
  auto source_it = data.find("source");
  auto reason_it = data.find("reason");
  
  if (active_it != data.end() && source_it != data.end() && reason_it != data.end()) {
    if (active_it->second == "true") {
      RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED: %s - %s", 
                  source_it->second.c_str(), reason_it->second.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "E-STOP CLEARED: %s - %s", 
                  source_it->second.c_str(), reason_it->second.c_str());
    }
  }
}

void TeensyBridge::HandleDiagnosticMessage(const MessageData& data, rclcpp::Time timestamp) {
  auto diagnostic_data = message_parser_->ParseDiagnosticData(data);
  if (diagnostic_data.valid) {
    auto msg = message_parser_->ToDiagnosticArrayMsg(diagnostic_data, timestamp);
    diagnostics_pub_->publish(msg);
  }
}

void TeensyBridge::EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  // Send E-stop command to embedded system
  std::string command;
  if (msg->data) {
    command = "ESTOP:trigger=true,reason=Software command\n";
  } else {
    command = "ESTOP:reset=true,source=SOFTWARE\n";
  }
  
  // Send to Board 1 (main controller)
  if (board1_fd_ >= 0) {
    ssize_t bytes_written = write(board1_fd_, command.c_str(), command.length());
    if (bytes_written != static_cast<ssize_t>(command.length())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send E-stop command to Board 1");
    }
  }
}

void TeensyBridge::ConfigCommandCallback(const std_msgs::msg::String::SharedPtr msg) {
  // Forward configuration command to embedded system
  std::string command = msg->data + "\n";
  
  // Send to Board 1 (main controller)
  if (board1_fd_ >= 0) {
    ssize_t bytes_written = write(board1_fd_, command.c_str(), command.length());
    if (bytes_written != static_cast<ssize_t>(command.length())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send config command to Board 1");
    }
  }
}

void TeensyBridge::StatusTimerCallback() {
  // Publish connection status
  static int call_count = 0;
  call_count++;
  
  if (call_count % 10 == 0) {  // Every 10 seconds
    RCLCPP_INFO(this->get_logger(), "Status: Board1=%s", 
                (board1_fd_ >= 0) ? "connected" : "disconnected");
  }
}

void TeensyBridge::DiagnosticsTimerCallback() {
  // Publish parsing statistics
  if (message_parser_) {
    auto stats_msg = message_parser_->GetParsingStatistics();
    diagnostics_pub_->publish(stats_msg);
  }
}

}  // namespace sigyn_to_sensor_v2
