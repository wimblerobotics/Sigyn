// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

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
#include <sstream>
#include <cstring>
#include <sstream>
#include <chrono>
#include <sys/ioctl.h>
#include <algorithm>

using namespace std::chrono_literals;

namespace sigyn_to_sensor_v2 {

  // Thread-local variable to track current board context
  thread_local int current_board_id = 0;

  TeensyBridge::TeensyBridge()
    : Node("teensy_bridge"),
    serial_running_(false),
    board1_fd_(-1),
    board2_fd_(-1),
    board3_fd_(-1) {

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

    message_parser_->RegisterCallback(MessageType::IMU,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleIMUMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::ESTOP,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleEstopMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::DIAGNOSTIC,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleDiagnosticMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::TEMPERATURE,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleTemperatureMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::VL53L0X,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleVL53L0XMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::ROBOCLAW,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleRoboClawMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::ODOM,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        HandleOdomMessage(data, timestamp);
      });

    message_parser_->RegisterCallback(MessageType::SDIR,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        auto content_it = data.find("content");
        if (content_it != data.end()) {
          HandleSdirResponse(content_it->second);
        }
      });

    message_parser_->RegisterCallback(MessageType::SDLINE,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        auto content_it = data.find("content");
        if (content_it != data.end()) {
          HandleSdlineResponse(content_it->second);
        }
      });

    message_parser_->RegisterCallback(MessageType::SDEOF,
      [this](const MessageData& data, rclcpp::Time timestamp) {
        auto content_it = data.find("content");
        HandleSdeofResponse(content_it != data.end() ? content_it->second : "");
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
    this->declare_parameter("board1_port", "/dev/teensy_sensor");
    this->declare_parameter("board2_port", "/dev/teensy_sensor2");
    this->declare_parameter("board3_port", "/dev/teensy_gripper");
    this->declare_parameter("baud_rate", 921600);
    this->declare_parameter("connection_timeout", 5.0);
    this->declare_parameter("reconnect_timeout", 2.0);

    // Get parameter values
    board1_port_ = this->get_parameter("board1_port").as_string();
    board2_port_ = this->get_parameter("board2_port").as_string();
    board3_port_ = this->get_parameter("board3_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    connection_timeout_ = this->get_parameter("connection_timeout").as_double();
    reconnect_timeout_ = this->get_parameter("reconnect_timeout").as_double();

    RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
    RCLCPP_INFO(this->get_logger(), "  Board 1 port: %s", board1_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Board 2 port: %s", board2_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Board 3 port: %s", board3_port_.c_str());
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

    // IMU publishers for dual sensors
    imu_sensor0_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "~/imu/sensor_0", 10);

    imu_sensor1_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "~/imu/sensor_1", 10);

    // Odometry publisher for wheel odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/sigyn/wheel_odom", 10);

    // Temperature publishers for motor sensors
    temperature_motor0_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature/motor_0", 10);

    temperature_motor1_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>(
      "~/temperature/motor_1", 10);

    // VL53L0X range sensor publishers
    vl53l0x_sensor0_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_0", 10);

    vl53l0x_sensor1_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_1", 10);

    vl53l0x_sensor2_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_2", 10);

    vl53l0x_sensor3_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_3", 10);

    vl53l0x_sensor4_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_4", 10);

    vl53l0x_sensor5_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_5", 10);

    vl53l0x_sensor6_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_6", 10);

    vl53l0x_sensor7_pub_ = this->create_publisher<sensor_msgs::msg::Range>(
      "~/range/vl53l0x_7", 10);

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

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/sigyn/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        CmdVelCallback(msg);
      });

    cmd_vel_gripper_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_gripper", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        CmdVelGripperCallback(msg);
      });

    // Create callback group for services
    service_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // Services
    sd_getdir_service_ = this->create_service<sigyn_interfaces::srv::TeensySdGetDir>(
      "teensy_sensor_sd_getdir",
      [this](const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
        std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response) {
          HandleSdGetDirRequest(request, response);
      },
      rmw_qos_profile_services_default,
      service_callback_group_);

    sd_getfile_service_ = this->create_service<sigyn_interfaces::srv::TeensySdGetFile>(
      "teensy_sensor_sd_getfile",
      [this](const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
        std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response) {
          HandleSdGetFileRequest(request, response);
      },
      rmw_qos_profile_services_default,
      service_callback_group_);

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

    if (board3_fd_ >= 0) {
      close(board3_fd_);
      board3_fd_ = -1;
    }

    RCLCPP_INFO(this->get_logger(), "Serial communication stopped");
  }

  void TeensyBridge::SerialReaderThread() {
    char buffer[1024];
    std::string line_buffer1, line_buffer2, line_buffer3;

    auto trim_crlf = [](std::string& s) {
      // Trim trailing CR and whitespace
      while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) {
        s.pop_back();
      }
      // Trim leading whitespace
      size_t start = 0;
      while (start < s.size() && (s[start] == ' ' || s[start] == '\t')) {
        ++start;
      }
      if (start > 0) s.erase(0, start);
      };

    while (serial_running_) {
      // Try to open serial ports if not connected
      if (board1_fd_ < 0) {
        int flags = O_RDWR | O_NOCTTY; // Blocking I/O; use select() to wake us
        board1_fd_ = open(board1_port_.c_str(), flags);
        if (board1_fd_ >= 0) {
          // Configure serial port (raw mode)
          struct termios tty;
          if (tcgetattr(board1_fd_, &tty) == 0) {
            cfmakeraw(&tty);
            // Baud (ignored by USB CDC but set anyway)
#ifdef B921600
            cfsetospeed(&tty, B921600);
            cfsetispeed(&tty, B921600);
#endif
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(HUPCL);
            // Disable XON/XOFF and CR/LF translations explicitly
            tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
            // Read returns as soon as 1 byte available
            tty.c_cc[VMIN] = 1;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(board1_fd_, TCSANOW, &tty) != 0) {
              RCLCPP_WARN(this->get_logger(), "tcsetattr failed for Board 1: %s", strerror(errno));
            }
            // Flush any pending I/O
            tcflush(board1_fd_, TCIOFLUSH);
            // Assert DTR/RTS so Teensy starts transmitting
            int mcs = TIOCM_DTR | TIOCM_RTS;
            (void)ioctl(board1_fd_, TIOCMBIS, &mcs);
            RCLCPP_INFO(this->get_logger(), "Connected to Board 1: %s", board1_port_.c_str());
          }
        }
      }

      if (board2_fd_ < 0) {
        int flags = O_RDWR | O_NOCTTY; // Blocking I/O; use select() to wake us
        board2_fd_ = open(board2_port_.c_str(), flags);
        if (board2_fd_ >= 0) {
          struct termios tty;
          if (tcgetattr(board2_fd_, &tty) == 0) {
            cfmakeraw(&tty);
#ifdef B921600
            cfsetospeed(&tty, B921600);
            cfsetispeed(&tty, B921600);
#endif
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(HUPCL);
            tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
            tty.c_cc[VMIN] = 1;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(board2_fd_, TCSANOW, &tty) != 0) {
              RCLCPP_WARN(this->get_logger(), "tcsetattr failed for Board 2: %s", strerror(errno));
            }
            tcflush(board2_fd_, TCIOFLUSH);
            int mcs = TIOCM_DTR | TIOCM_RTS;
            (void)ioctl(board2_fd_, TIOCMBIS, &mcs);
            RCLCPP_INFO(this->get_logger(), "Connected to Board 2: %s", board2_port_.c_str());
          }
        }
      }

      if (board3_fd_ < 0) {
        int flags = O_RDWR | O_NOCTTY; // Blocking I/O; use select() to wake us
        board3_fd_ = open(board3_port_.c_str(), flags);
        if (board3_fd_ >= 0) {
          struct termios tty;
          if (tcgetattr(board3_fd_, &tty) == 0) {
            cfmakeraw(&tty);
#ifdef B921600
            cfsetospeed(&tty, B921600);
            cfsetispeed(&tty, B921600);
#endif
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(HUPCL);
            tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
            tty.c_cc[VMIN] = 1;
            tty.c_cc[VTIME] = 0;
            if (tcsetattr(board3_fd_, TCSANOW, &tty) != 0) {
              RCLCPP_WARN(this->get_logger(), "tcsetattr failed for Board 3: %s", strerror(errno));
            }
            tcflush(board3_fd_, TCIOFLUSH);
            int mcs = TIOCM_DTR | TIOCM_RTS;
            (void)ioctl(board3_fd_, TIOCMBIS, &mcs);
            RCLCPP_INFO(this->get_logger(), "Connected to Board 3: %s", board3_port_.c_str());
          }
        }
      }

      fd_set read_fds;
      FD_ZERO(&read_fds);
      int max_fd = -1;

      if (board1_fd_ >= 0) {
        FD_SET(board1_fd_, &read_fds);
        max_fd = std::max(max_fd, board1_fd_);
      }
      if (board2_fd_ >= 0) {
        FD_SET(board2_fd_, &read_fds);
        max_fd = std::max(max_fd, board2_fd_);
      }
      if (board3_fd_ >= 0) {
        FD_SET(board3_fd_, &read_fds);
        max_fd = std::max(max_fd, board3_fd_);
      }

      if (max_fd == -1) {
        std::this_thread::sleep_for(1s);
        continue;
      }

      struct timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;  // 100ms timeout

      int result = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);

      if (result > 0) {
        if (board1_fd_ >= 0 && FD_ISSET(board1_fd_, &read_fds)) {
          ssize_t bytes_read = read(board1_fd_, buffer, sizeof(buffer) - 1);
          if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            line_buffer1 += buffer;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Board1 read %zd bytes", bytes_read);

            // Process complete lines
            size_t pos;
            while ((pos = line_buffer1.find('\n')) != std::string::npos) {
              std::string line = line_buffer1.substr(0, pos);
              line_buffer1.erase(0, pos + 1);
              trim_crlf(line);
              if (!line.empty()) {
                current_board_id = 1;
                auto timestamp = this->now();
                if (!message_parser_->ParseJsonMessage(line, timestamp)) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON message from board 1: '%s'", line.c_str());
                }
              }
            }
          }
          else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "Board 1 read error: %s", strerror(errno));
            close(board1_fd_);
            board1_fd_ = -1;
          }
        }
        if (board2_fd_ >= 0 && FD_ISSET(board2_fd_, &read_fds)) {
          ssize_t bytes_read = read(board2_fd_, buffer, sizeof(buffer) - 1);
          if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            line_buffer2 += buffer;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Board2 read %zd bytes", bytes_read);

            size_t pos;
            while ((pos = line_buffer2.find('\n')) != std::string::npos) {
              std::string line = line_buffer2.substr(0, pos);
              line_buffer2.erase(0, pos + 1);
              trim_crlf(line);
              if (!line.empty()) {
                current_board_id = 2;
                auto timestamp = this->now();
                if (!message_parser_->ParseJsonMessage(line, timestamp)) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON message from board 2: '%s'", line.c_str());
                }
              }
            }
          }
          else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "Board 2 read error: %s", strerror(errno));
            close(board2_fd_);
            board2_fd_ = -1;
          }
        }
        if (board3_fd_ >= 0 && FD_ISSET(board3_fd_, &read_fds)) {
          ssize_t bytes_read = read(board3_fd_, buffer, sizeof(buffer) - 1);
          if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            line_buffer3 += buffer;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Board3 read %zd bytes", bytes_read);

            size_t pos;
            while ((pos = line_buffer3.find('\n')) != std::string::npos) {
              std::string line = line_buffer3.substr(0, pos);
              line_buffer3.erase(0, pos + 1);
              trim_crlf(line);
              if (!line.empty()) {
                current_board_id = 3;
                auto timestamp = this->now();
                if (!message_parser_->ParseJsonMessage(line, timestamp)) {
                  RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON message from board 3: '%s'", line.c_str());
                }
              }
            }
          }
          else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(this->get_logger(), "Board 3 read error: %s", strerror(errno));
            close(board3_fd_);
            board3_fd_ = -1;
          }
        }
      }
      else if (result < 0) {
        RCLCPP_WARN(this->get_logger(), "select error: %s", strerror(errno));
        if (board1_fd_ >= 0) close(board1_fd_);
        if (board2_fd_ >= 0) close(board2_fd_);
        if (board3_fd_ >= 0) close(board3_fd_);
        board1_fd_ = -1;
        board2_fd_ = -1;
        board3_fd_ = -1;
      }

      // Send any queued messages (like cmd_vel)
      if (board1_fd_ >= 0) {
        SendQueuedMessages();
      }

      // Send any queued gripper messages
      if (board3_fd_ >= 0) {
        SendGripperMessages();
      }

      // Small delay to prevent busy waiting
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void TeensyBridge::HandleBatteryMessage(const MessageData& data, rclcpp::Time timestamp) {
    auto battery_data = message_parser_->ParseBatteryData(data);
    if (battery_data.valid) {
      auto msg = message_parser_->ToBatteryStateMsg(battery_data, timestamp);
      battery_pub_->publish(msg);
    }
  }

  void TeensyBridge::HandleIMUMessage(const MessageData& data, rclcpp::Time timestamp) {
    auto imu_data = message_parser_->ParseIMUData(data);
    if (imu_data.valid) {
      auto msg = message_parser_->ToImuMsg(imu_data, timestamp);

      // Publish to the appropriate sensor topic
      if (imu_data.sensor_id == 0) {
        imu_sensor0_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published IMU data for sensor 0");
      }
      else if (imu_data.sensor_id == 1) {
        imu_sensor1_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published IMU data for sensor 1");
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Invalid IMU sensor ID: %d", imu_data.sensor_id);
      }
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Received invalid IMU data");
    }
  }

  void TeensyBridge::HandlePerformanceMessage(const MessageData& data, rclcpp::Time timestamp) {
    // Parse performance data and convert to diagnostic message for monitoring
    // Board health monitoring is now handled on Teensy side by safety_coordinator
    auto performance_data = message_parser_->ParsePerformanceData(data);
    if (performance_data.valid) {
      // Create a diagnostic message from performance data
      sigyn_to_sensor_v2::DiagnosticData diagnostic_data;
      diagnostic_data.valid = true;
      diagnostic_data.level = "INFO";
      diagnostic_data.module = "performance_monitor";
      diagnostic_data.message = "TeensyV2 performance metrics";

      // Add performance details
      std::ostringstream details;
      details << "freq=" << performance_data.loop_frequency
        << ", exec=" << performance_data.execution_time
        << ", violations=" << performance_data.violation_count
        << ", modules=" << performance_data.module_count;
      diagnostic_data.details = details.str();

      auto msg = message_parser_->ToDiagnosticArrayMsg(diagnostic_data, timestamp);
      diagnostics_pub_->publish(msg);
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
      }
      else {
        RCLCPP_INFO(this->get_logger(), "E-STOP CLEARED: %s - %s",
          source_it->second.c_str(), reason_it->second.c_str());
      }
    }
  }

  void TeensyBridge::HandleDiagnosticMessage(const MessageData& data, rclcpp::Time timestamp) {
    // Debug logging for empty diagnostic investigation
    RCLCPP_DEBUG(this->get_logger(), "HandleDiagnosticMessage called from board %d", current_board_id);
    RCLCPP_DEBUG(this->get_logger(), "  Data map size: %zu", data.size());
    for (const auto& kv : data) {
      RCLCPP_DEBUG(this->get_logger(), "  Key: '%s', Value: '%s'", kv.first.c_str(), kv.second.c_str());
    }

    auto diagnostic_data = message_parser_->ParseDiagnosticData(data);
    if (diagnostic_data.valid) {
      // Check for ESTOP information in CRITICAL diagnostic messages
      if (diagnostic_data.level == "CRITICAL") {
        // Look for ESTOP patterns in the message content
        std::string message = diagnostic_data.message;
        if (message.find("active:true") != std::string::npos) {
          // Extract ESTOP information and treat as ESTOP message
          std::string source = "UNKNOWN";
          std::string reason = "UNKNOWN";

          // Parse source field
          size_t source_pos = message.find("source:");
          if (source_pos != std::string::npos) {
            size_t source_start = source_pos + 7; // Length of "source:"
            size_t source_end = message.find(",", source_start);
            if (source_end != std::string::npos) {
              source = message.substr(source_start, source_end - source_start);
            }
            else {
              source = message.substr(source_start);
            }
          }

          // Parse reason field
          size_t reason_pos = message.find("reason:");
          if (reason_pos != std::string::npos) {
            size_t reason_start = reason_pos + 7; // Length of "reason:"
            size_t reason_end = message.find(",", reason_start);
            if (reason_end != std::string::npos) {
              reason = message.substr(reason_start, reason_end - reason_start);
            }
            else {
              reason = message.substr(reason_start);
            }
          }

          // Log ESTOP event
          RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED (from diagnostic): %s - %s",
            source.c_str(), reason.c_str());

          // Publish ESTOP status
          auto estop_msg = std_msgs::msg::Bool();
          estop_msg.data = true;
          estop_status_pub_->publish(estop_msg);
        }
      }

      auto msg = message_parser_->ToDiagnosticArrayMsg(diagnostic_data, timestamp);

      // Debug the generated diagnostic message
      RCLCPP_DEBUG(this->get_logger(), "Publishing diagnostic message with %zu status entries", msg.status.size());
      for (size_t i = 0; i < msg.status.size(); ++i) {
        const auto& status = msg.status[i];
        RCLCPP_DEBUG(this->get_logger(), "  Status[%zu]: name='%s', message='%s', hardware_id='%s', level=%d",
          i, status.name.c_str(), status.message.c_str(), status.hardware_id.c_str(), status.level);
        if (status.name.empty() || status.message.empty()) {
          RCLCPP_WARN(this->get_logger(), "EMPTY DIAGNOSTIC DETECTED! Board %d generated empty diagnostic entry", current_board_id);
        }
      }

      diagnostics_pub_->publish(msg);
    }
    else {
      RCLCPP_DEBUG(this->get_logger(), "ParseDiagnosticData returned invalid data, not publishing");
    }
  }

  void TeensyBridge::HandleTemperatureMessage(const MessageData& data, rclcpp::Time timestamp) {
    // Handle aggregate TEMPERATURE message: TEMPERATURE1:{"total_sensors":8,"active_sensors":2,"temperatures":[25.4,26.1,null,null,null,null,null,null],...}
    auto temperature_data = message_parser_->ParseTemperatureData(data);

    if (temperature_data.valid && !temperature_data.temperatures.empty()) {
      // Process individual sensor readings from the temperatures array
      for (size_t i = 0; i < temperature_data.temperatures.size() && i < 2; i++) {
        double temp_value = temperature_data.temperatures[i];

        // Skip null/invalid temperatures
        if (std::isnan(temp_value)) {
          continue;
        }

        // Create individual sensor message
        sensor_msgs::msg::Temperature temp_msg;
        temp_msg.header.stamp = timestamp;
        temp_msg.header.frame_id = "motor_" + std::to_string(i);
        temp_msg.temperature = temp_value;
        temp_msg.variance = 0.1;  // Conservative variance estimate

        // Route to appropriate publisher based on sensor index
        if (i == 0 && temperature_motor0_pub_) {
          temperature_motor0_pub_->publish(temp_msg);
        }
        else if (i == 1 && temperature_motor1_pub_) {
          temperature_motor1_pub_->publish(temp_msg);
        }
      }

      RCLCPP_DEBUG(this->get_logger(), "Processed TEMPERATURE message: %d sensors, %zu temperature values",
        temperature_data.total_sensors, temperature_data.temperatures.size());
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Invalid temperature data or empty temperatures array");
    }
  }

  void TeensyBridge::HandleVL53L0XMessage(const MessageData& data, rclcpp::Time timestamp) {
    auto vl53l0x_data = message_parser_->ParseVL53L0XData(data);

    if (vl53l0x_data.valid) {
      // Publish range data for each sensor in the distances array
      for (size_t i = 0; i < vl53l0x_data.distances_mm.size() && i < 8; ++i) {
        if (vl53l0x_data.distances_mm[i] > 0) {  // 0 indicates invalid reading
          sensor_msgs::msg::Range range_msg;

          // Correct timestamp using age_us to get actual measurement time
          rclcpp::Time corrected_timestamp = timestamp;
          if (i < vl53l0x_data.age_us.size() && vl53l0x_data.age_us[i] > 0) {
            // Subtract the age to get the actual measurement time
            corrected_timestamp = timestamp - rclcpp::Duration::from_nanoseconds(
              static_cast<int64_t>(vl53l0x_data.age_us[i]) * 1000);
          }

          range_msg.header.stamp = corrected_timestamp;
          range_msg.header.frame_id = "vl53l0x_" + std::to_string(i);
          range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
          range_msg.field_of_view = 0.44;  // ~25 degrees in radians for VL53L0X
          range_msg.min_range = 0.03;      // 3cm minimum range
          range_msg.max_range = 2.0;       // 2m maximum range

          // Convert mm to meters
          range_msg.range = vl53l0x_data.distances_mm[i] / 1000.0;

          // Route to appropriate publisher based on sensor index
          auto publishers = std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>{
            vl53l0x_sensor0_pub_, vl53l0x_sensor1_pub_, vl53l0x_sensor2_pub_, vl53l0x_sensor3_pub_,
            vl53l0x_sensor4_pub_, vl53l0x_sensor5_pub_, vl53l0x_sensor6_pub_, vl53l0x_sensor7_pub_
          };

          if (i < publishers.size() && publishers[i]) {
            publishers[i]->publish(range_msg);
            RCLCPP_DEBUG(this->get_logger(), "Published range for sensor %zu: %.3f m", i, range_msg.range);
          }
        }
      }
    }
    else {
      RCLCPP_DEBUG(this->get_logger(), "Received invalid VL53L0X data");
    }
  }

  void TeensyBridge::HandleRoboClawMessage(const MessageData& data, rclcpp::Time timestamp) {
    // Parse RoboClaw data and convert to diagnostic message for now
    // In the future, this could be extended to publish motor controller specific messages
    auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
    diag_msg.header.stamp = timestamp;

    auto roboclaw_status = diagnostic_msgs::msg::DiagnosticStatus();
    roboclaw_status.name = "roboclaw_motor_controller";
    roboclaw_status.hardware_id = "roboclaw_v2";

    // Check for error conditions
    auto error_it = data.find("Error");
    auto error_decoded_it = data.find("ErrorDecoded");

    if (error_it != data.end() && error_it->second != "0") {
      roboclaw_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      if (error_decoded_it != data.end()) {
        roboclaw_status.message = "RoboClaw Error: " + error_decoded_it->second;
      }
      else {
        roboclaw_status.message = "RoboClaw Error Code: " + error_it->second;
      }
    }
    else {
      roboclaw_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      roboclaw_status.message = "RoboClaw operating normally";
    }

    // Add all RoboClaw data as key-value pairs
    for (const auto& kv : data) {
      auto roboclaw_kv = diagnostic_msgs::msg::KeyValue();
      roboclaw_kv.key = kv.first;
      roboclaw_kv.value = kv.second;
      roboclaw_status.values.push_back(roboclaw_kv);
    }

    diag_msg.status.push_back(roboclaw_status);
    diagnostics_pub_->publish(diag_msg);
  }

  void TeensyBridge::HandleOdomMessage(const MessageData& data, rclcpp::Time timestamp) {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    try {
      // Parse position
      auto px_it = data.find("px");
      auto py_it = data.find("py");
      if (px_it != data.end() && py_it != data.end()) {
        odom_msg.pose.pose.position.x = std::stod(px_it->second);
        odom_msg.pose.pose.position.y = std::stod(py_it->second);
        odom_msg.pose.pose.position.z = 0.0;
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Missing position data in odometry message");
      }

      // Parse orientation quaternion
      auto ox_it = data.find("ox");
      auto oy_it = data.find("oy");
      auto oz_it = data.find("oz");
      auto ow_it = data.find("ow");
      if (ox_it != data.end() && oy_it != data.end() && oz_it != data.end() && ow_it != data.end()) {
        odom_msg.pose.pose.orientation.x = std::stod(ox_it->second);
        odom_msg.pose.pose.orientation.y = std::stod(oy_it->second);
        odom_msg.pose.pose.orientation.z = std::stod(oz_it->second);
        odom_msg.pose.pose.orientation.w = std::stod(ow_it->second);
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Missing orientation data in odometry message");
      }

      // Parse velocity
      auto vx_it = data.find("vx");
      auto vy_it = data.find("vy");
      auto wz_it = data.find("wz");
      if (vx_it != data.end() && vy_it != data.end() && wz_it != data.end()) {
        odom_msg.twist.twist.linear.x = std::stod(vx_it->second);
        odom_msg.twist.twist.linear.y = std::stod(vy_it->second);
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = std::stod(wz_it->second);
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Missing velocity data in odometry message");
      }

      // Set covariance matrices (based on wheel odometry characteristics)
      // Position covariance (uncertainty grows with distance traveled)
      odom_msg.pose.covariance[0] = 0.001;  // x
      odom_msg.pose.covariance[7] = 0.001;  // y
      odom_msg.pose.covariance[35] = 0.001; // yaw

      // Velocity covariance (encoder-based, relatively accurate)
      odom_msg.twist.covariance[0] = 0.0001;  // vx
      odom_msg.twist.covariance[7] = 0.0001;  // vy
      odom_msg.twist.covariance[35] = 0.0001; // wz

      odom_pub_->publish(odom_msg);

    }
    catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to parse odometry data: %s", e.what());
    }
  }

  void TeensyBridge::EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // Send E-stop command to embedded system
    std::string command;
    if (msg->data) {
      command = "ESTOP:trigger=true,reason=Software command\n";
    }
    else {
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
      RCLCPP_INFO(this->get_logger(), "Status: Board1=%s, Board2=%s, Board3=%s",
        (board1_fd_ >= 0) ? "connected" : "disconnected",
        (board2_fd_ >= 0) ? "connected" : "disconnected",
        (board3_fd_ >= 0) ? "connected" : "disconnected");
    }
  }

  void TeensyBridge::DiagnosticsTimerCallback() {
    // Only publish parsing statistics if we have received any messages
    if (message_parser_) {
      // Check if there's any meaningful data to report
      auto stats_msg = message_parser_->GetParsingStatistics();

      // Debug logging for empty diagnostic investigation
      RCLCPP_DEBUG(this->get_logger(), "DiagnosticsTimerCallback: stats_msg has %zu status entries", stats_msg.status.size());
      for (size_t i = 0; i < stats_msg.status.size(); ++i) {
        const auto& status = stats_msg.status[i];
        RCLCPP_DEBUG(this->get_logger(), "  Stats[%zu]: name='%s', message='%s', hardware_id='%s'",
          i, status.name.c_str(), status.message.c_str(), status.hardware_id.c_str());
        if (status.name.empty() || status.message.empty()) {
          RCLCPP_WARN(this->get_logger(), "EMPTY DIAGNOSTIC IN STATS! Timer callback generated empty diagnostic entry");
        }
      }

      // Only publish if we have received at least one message
      bool has_data = false;
      for (const auto& status : stats_msg.status) {
        for (const auto& kv : status.values) {
          if (kv.key == "total_received" && std::stoi(kv.value) > 0) {
            has_data = true;
            break;
          }
        }
        if (has_data) break;
      }

      if (has_data) {
        RCLCPP_DEBUG(this->get_logger(), "Publishing parsing statistics diagnostic message");
        diagnostics_pub_->publish(stats_msg);
      }
      else {
        // Log that we're not receiving data, but don't spam with empty diagnostics
        static auto last_no_data_log = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_no_data_log).count() >= 30) {
          RCLCPP_WARN(this->get_logger(), "No data received from Teensy devices - diagnostic publishing suspended");
          last_no_data_log = now;
        }
      }
    }
  }

  void TeensyBridge::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Queue the twist message - all serial communication happens in the main loop
    // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.3f, angular.z=%.3f", 
    //             msg->linear.x, msg->angular.z);

    std::ostringstream oss;
    oss << "TWIST:linear_x:" << msg->linear.x << ",angular_z:" << msg->angular.z << "\n";

    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
    outgoing_message_queue_.push(oss.str());

    // RCLCPP_INFO(this->get_logger(), "Queued TWIST command: %s", oss.str().c_str());
  }

  void TeensyBridge::CmdVelGripperCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Queue the twist message for the gripper board (board3)
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel_gripper: linear.x=%.3f, angular.z=%.3f",
      msg->linear.x, msg->angular.z);

    std::ostringstream oss;
    oss << "TWIST:linear_x:" << msg->linear.x << ",angular_z:" << msg->angular.z << "\n";

    std::lock_guard<std::mutex> lock(gripper_queue_mutex_);
    gripper_message_queue_.push(oss.str());

    RCLCPP_INFO(this->get_logger(), "Queued gripper TWIST command: %s", oss.str().c_str());
  }

  void TeensyBridge::SendQueuedMessages() {
    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);

    while (!outgoing_message_queue_.empty()) {
      const std::string& message = outgoing_message_queue_.front();

      // RCLCPP_INFO(this->get_logger(), "Sending message to Teensy: '%s'", message.c_str());

      ssize_t bytes_written = write(board1_fd_, message.c_str(), message.length());
      if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
        break;
      }
      else {
        // RCLCPP_INFO(this->get_logger(), "Sent %zd bytes to Teensy", bytes_written);
        // Force immediate transmission
        fsync(board1_fd_);
      }

      outgoing_message_queue_.pop();
    }
  }

  void TeensyBridge::SendGripperMessages() {
    std::lock_guard<std::mutex> lock(gripper_queue_mutex_);

    while (!gripper_message_queue_.empty()) {
      const std::string& message = gripper_message_queue_.front();

      RCLCPP_INFO(this->get_logger(), "Sending gripper message: '%s'", message.c_str());

      if (board3_fd_ >= 0) {
        ssize_t bytes_written = write(board3_fd_, message.c_str(), message.length());
        if (bytes_written < 0) {
          RCLCPP_ERROR(this->get_logger(), "Failed to write to gripper board (board3)");
          break;
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Sent %zd bytes to gripper board", bytes_written);
          // Force immediate transmission
          fsync(board3_fd_);
        }
      }
      else {
        RCLCPP_WARN(this->get_logger(), "Gripper board (board3) not connected, discarding message");
      }

      gripper_message_queue_.pop();
    }
  }

  void TeensyBridge::HandleSdGetDirRequest(
    const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
    std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response) {

    (void)request;  // Unused parameter

    // RCLCPP_INFO(this->get_logger(), "SD GetDir service called");

    if (board1_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "SD GetDir failed: Serial connection not available");
      response->success = false;
      response->error_message = "Serial connection not available";
      response->directory_listing = "";
      return;
    }

    // RCLCPP_INFO(this->get_logger(), "Sending SDDIR command to Teensy");

    // Create a promise for async completion
    auto completion_promise = std::make_shared<std::promise<bool>>();
    auto completion_future = completion_promise->get_future();

    // Store the service request for async completion
    {
      std::lock_guard<std::mutex> lock(service_mutex_);
      pending_service_requests_.push({
        "sdir_request",
        "get_dir",
        response,
        nullptr,
        this->get_clock()->now(),
        this->get_clock()->now(),
        completion_promise,
        ""
        });
    }

    // Queue the serial message
    {
      std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
      outgoing_message_queue_.push("SDDIR:\n");
      // RCLCPP_INFO(this->get_logger(), "Queued SDDIR message for sending");
    }

    // Wait for completion with timeout
    auto status = completion_future.wait_for(std::chrono::seconds(10));

    if (status == std::future_status::timeout) {
      RCLCPP_ERROR(this->get_logger(), "SD GetDir request timed out after 10 seconds");
      // Handle timeout
      std::lock_guard<std::mutex> lock(service_mutex_);

      // Remove the request from the queue if still there
      std::queue<PendingServiceRequest> temp_queue;
      while (!pending_service_requests_.empty()) {
        auto req = pending_service_requests_.front();
        pending_service_requests_.pop();
        if (req.request_id != "sdir_request") {
          temp_queue.push(req);
        }
      }
      pending_service_requests_ = temp_queue;

      response->success = false;
      response->error_message = "Request timed out";
      response->directory_listing = "";
    }
    else {
      // RCLCPP_INFO(this->get_logger(), "SD GetDir request completed successfully");
    }
  }

  void TeensyBridge::HandleSdGetFileRequest(
    const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
    std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response) {

    if (board1_fd_ < 0) {
      response->success = false;
      response->error_message = "Serial connection not available";
      response->file_contents = "";
      return;
    }

    if (request->filename.empty()) {
      response->success = false;
      response->error_message = "Filename cannot be empty";
      response->file_contents = "";
      return;
    }

    // Create a promise for async completion
    auto completion_promise = std::make_shared<std::promise<bool>>();
    auto completion_future = completion_promise->get_future();

    // Store the service request for async completion
    {
      std::lock_guard<std::mutex> lock(service_mutex_);
      pending_service_requests_.push({
        "sdfile_request",
        "get_file",
        nullptr,
        response,
        this->get_clock()->now(),
        this->get_clock()->now(),
        completion_promise,
        ""
        });
    }

    // Queue the serial message
    {
      std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
      std::string message = "SDFILE:" + request->filename + "\n";
      outgoing_message_queue_.push(message);
    }

    // Wait for completion with timeout
    auto status = completion_future.wait_for(std::chrono::seconds(30));

    if (status == std::future_status::timeout) {
      // Handle timeout
      std::lock_guard<std::mutex> lock(service_mutex_);

      // Remove the request from the queue if still there
      std::queue<PendingServiceRequest> temp_queue;
      while (!pending_service_requests_.empty()) {
        auto req = pending_service_requests_.front();
        pending_service_requests_.pop();
        if (req.request_id != "sdfile_request") {
          temp_queue.push(req);
        }
      }
      pending_service_requests_ = temp_queue;

      response->success = false;
      response->error_message = "Request timed out";
      response->file_contents = "";
    }
  }

  void TeensyBridge::HandleSdirResponse(const std::string& data) {
    // RCLCPP_INFO(this->get_logger(), "Received SDIR response: '%s'", data.c_str());

    std::lock_guard<std::mutex> lock(service_mutex_);

    if (!pending_service_requests_.empty() &&
      pending_service_requests_.front().service_type == "get_dir") {

      // For directory requests, complete immediately after SDIR response
      auto request = pending_service_requests_.front();
      pending_service_requests_.pop();

      // Set the directory listing and mark as successful
      request.dir_response->directory_listing = data;
      request.dir_response->success = true;
      request.dir_response->error_message = "";

      // Signal completion
      request.completion_promise->set_value(true);
      // 
      RCLCPP_INFO(this->get_logger(), "Completed directory listing request");
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Received SDIR response but no matching pending request");
    }
  }

  void TeensyBridge::HandleSdlineResponse(const std::string& data) {
    std::lock_guard<std::mutex> lock(service_mutex_);

    if (!pending_service_requests_.empty() &&
      pending_service_requests_.front().service_type == "get_file") {
      auto& request = pending_service_requests_.front();
      request.accumulated_content += data + "\n";
      request.last_activity_time = this->get_clock()->now();
    }
  }

  void TeensyBridge::HandleSdeofResponse(const std::string& data) {
    (void)data;  // Mark parameter as intentionally unused
    std::lock_guard<std::mutex> lock(service_mutex_);

    if (!pending_service_requests_.empty()) {
      auto request = pending_service_requests_.front();
      pending_service_requests_.pop();

      if (request.service_type == "get_dir") {
        request.dir_response->success = true;
        request.dir_response->error_message = "";
        // Directory listing was accumulated in HandleSdirResponse
      }
      else if (request.service_type == "get_file") {
        request.file_response->success = true;
        request.file_response->error_message = "";
        request.file_response->file_contents = request.accumulated_content;
      }

      // Signal completion
      request.completion_promise->set_value(true);
    }
  }

}  // namespace sigyn_to_sensor_v2
