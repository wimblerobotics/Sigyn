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
#include <array>
#include <optional>

using namespace std::chrono_literals;

namespace sigyn_to_sensor_v2
{

  // Thread-local variable to track current board context
thread_local int current_board_id = 0;
thread_local std::string current_serial_frame;

namespace {
std::string sanitize_frame_for_log(std::string_view input, size_t max_len)
{
  std::string out;
  out.reserve(std::min(max_len, input.size()));
  for (size_t i = 0; i < input.size() && out.size() < max_len; ++i) {
    unsigned char c = static_cast<unsigned char>(input[i]);
    if (c == '\t') {
      out += "\\t";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\n') {
      out += "\\n";
    } else if (c < 0x20 || c == 0x7F) {
      out += '?';
    } else {
      out.push_back(static_cast<char>(c));
    }
  }
  return out;
}
}  // namespace

TeensyBridge::TeensyBridge()
: Node("teensy_bridge"),
  serial_running_(false),
  board1_fd_(-1),
  board2_fd_(-1),
  board3_fd_(-1)
{

  RCLCPP_INFO(this->get_logger(), "Initializing TeensyV2 Bridge Node");

  InitializeParameters();
  InitializePublishersAndSubscribers();

    // Initialize message parser
  message_parser_ = std::make_unique<MessageParser>(this->get_logger());

    // Register message callbacks
  message_parser_->RegisterCallback(MessageType::BATTERY,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleBatteryMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::PERFORMANCE,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandlePerformanceMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::SAFETY,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleSafetyMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::IMU,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleIMUMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::ESTOP,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleEstopMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::DIAGNOSTIC,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleDiagnosticMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::FAULT,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleFaultMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::TEMPERATURE,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleTemperatureMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::VL53L0X,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleVL53L0XMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::ROBOCLAW,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      HandleRoboClawMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::ODOM,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      (void)timestamp;   // Suppress unused parameter warning
      HandleOdomMessage(data, timestamp);
      });

  message_parser_->RegisterCallback(MessageType::SDIR,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      (void)timestamp;   // Suppress unused parameter warning
      auto content_it = data.find("content");
      if (content_it != data.end()) {
        HandleSdirResponse(content_it->second);
      }
      });

  message_parser_->RegisterCallback(MessageType::SDLINE,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      (void)timestamp;   // Suppress unused parameter warning
      auto content_it = data.find("content");
      if (content_it != data.end()) {
        HandleSdlineResponse(content_it->second);
      }
      });

  message_parser_->RegisterCallback(MessageType::SDEOF,
    [this](const MessageData & data, rclcpp::Time timestamp) {
      (void)timestamp;   // Suppress unused parameter warning
      auto content_it = data.find("content");
      HandleSdeofResponse(content_it != data.end() ? content_it->second : "");
      });

    // Start serial communication
  StartSerialCommunication();

  RCLCPP_INFO(this->get_logger(), "TeensyV2 Bridge Node initialization complete");
}

TeensyBridge::~TeensyBridge()
{
  StopSerialCommunication();
}

void TeensyBridge::InitializeParameters()
{
    // Declare parameters with default values
  this->declare_parameter("board1_port", "/dev/teensy_sensor");
  this->declare_parameter("board2_port", "/dev/teensy_sensor2");
  this->declare_parameter("board3_port", "/dev/teensy_gripper");
  this->declare_parameter("baud_rate", 921600);
  this->declare_parameter("connection_timeout", 5.0);
  this->declare_parameter("reconnect_timeout", 2.0);

    // Clamp how far we backdate VL53L0X stamps using age_us.
    // Large backdating can exceed TF buffer history and cause message-filter drops.
  this->declare_parameter("vl53_age_correction_max_us", 500000);

    // SD maintenance
  this->declare_parameter("sd_prune_keep_last_topic", "~/commands/sd_prune_keep_last");

    // SD service timeouts (prune can take a while)
  // Directory listings should be quick; keep the default tight to avoid hanging callers.
  this->declare_parameter("sd_getdir_timeout_sec", 10);
  this->declare_parameter("sd_getfile_timeout_sec", 60);

  // Debug: when true, log the raw Teensy serial frame for E-STOP-related events.
  this->declare_parameter("log_estop_raw_lines", false);

    // Get parameter values
  board1_port_ = this->get_parameter("board1_port").as_string();
  board2_port_ = this->get_parameter("board2_port").as_string();
  board3_port_ = this->get_parameter("board3_port").as_string();
  baud_rate_ = this->get_parameter("baud_rate").as_int();
  connection_timeout_ = this->get_parameter("connection_timeout").as_double();
  reconnect_timeout_ = this->get_parameter("reconnect_timeout").as_double();

  vl53_age_correction_max_us_ = static_cast<uint32_t>(
    this->get_parameter("vl53_age_correction_max_us").as_int());

  sd_prune_keep_last_topic_ = this->get_parameter("sd_prune_keep_last_topic").as_string();

  sd_getdir_timeout_sec_ = static_cast<uint32_t>(
    this->get_parameter("sd_getdir_timeout_sec").as_int());
  sd_getfile_timeout_sec_ = static_cast<uint32_t>(
    this->get_parameter("sd_getfile_timeout_sec").as_int());

  log_estop_raw_lines_ = this->get_parameter("log_estop_raw_lines").as_bool();

  RCLCPP_INFO(this->get_logger(), "Parameters initialized:");
  RCLCPP_INFO(this->get_logger(), "  Board 1 port: %s", board1_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Board 2 port: %s", board2_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Board 3 port: %s", board3_port_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Baud rate: %d", baud_rate_);
  RCLCPP_INFO(this->get_logger(), "  SD prune topic: %s", sd_prune_keep_last_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "  SD getdir timeout (s): %u", sd_getdir_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  SD getfile timeout (s): %u", sd_getfile_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "  VL53 age correction max (us): %u",
    vl53_age_correction_max_us_);
  RCLCPP_INFO(this->get_logger(), "  Log E-STOP raw lines: %s", log_estop_raw_lines_ ? "true" : "false");
}

void TeensyBridge::InitializePublishersAndSubscribers()
{
    // Publishers
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "~/battery/status", 10);

  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "~/diagnostics", 10);

  estop_status_pub_ = this->create_publisher<sigyn_interfaces::msg::EStopStatus>(
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

  sd_prune_keep_last_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
      sd_prune_keep_last_topic_, 10,
    [this](const std_msgs::msg::UInt16::SharedPtr msg) {
      SdPruneKeepLastCallback(msg);
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
      rclcpp::ServicesQoS(),
      service_callback_group_);

  sd_getfile_service_ = this->create_service<sigyn_interfaces::srv::TeensySdGetFile>(
      "teensy_sensor_sd_getfile",
    [this](const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
    std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response) {
      HandleSdGetFileRequest(request, response);
      },
      rclcpp::ServicesQoS(),
      service_callback_group_);

  reset_fault_service_ = this->create_service<sigyn_interfaces::srv::ResetFault>(
      "~/safety/reset_fault",
    [this](const std::shared_ptr<sigyn_interfaces::srv::ResetFault::Request> request,
      std::shared_ptr<sigyn_interfaces::srv::ResetFault::Response> response) {
      HandleResetFault(request, response);
      },
      rclcpp::ServicesQoS(), service_callback_group_);

    // Timers
  status_timer_ = this->create_wall_timer(
      1000ms, [this]() {StatusTimerCallback();});

  diagnostics_timer_ = this->create_wall_timer(
      5000ms, [this]() {DiagnosticsTimerCallback();});
}

void TeensyBridge::SdPruneKeepLastCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (board1_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(), "SD prune request dropped: board1 not connected");
    return;
  }

    // Send SDPRUNE:<count> to Teensy (board1). Teensy side will never delete the current log file.
  {
    std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
    outgoing_message_queue_.push("SDPRUNE:" + std::to_string(msg->data) + "\n");
  }

  RCLCPP_INFO(this->get_logger(), "Queued SDPRUNE keep_last=%u", (unsigned)msg->data);
}

void TeensyBridge::StartSerialCommunication()
{
  if (serial_running_) {
    return;
  }

  serial_running_ = true;
  serial_thread_ = std::thread(&TeensyBridge::SerialReaderThread, this);

  RCLCPP_INFO(this->get_logger(), "Serial communication started");
}

void TeensyBridge::StopSerialCommunication()
{
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

void TeensyBridge::SerialReaderThread()
{
  char buffer[1024];
  std::string line_buffer1, line_buffer2, line_buffer3;

  auto sanitize_for_log = [](std::string_view input, size_t max_len) -> std::string {
      std::string out;
      out.reserve(std::min(max_len, input.size()));
      for (size_t i = 0; i < input.size() && out.size() < max_len; ++i) {
        unsigned char c = static_cast<unsigned char>(input[i]);
        if (c == '\t') {
          out += "\\t";
        } else if (c == '\r') {
          out += "\\r";
        } else if (c == '\n') {
          out += "\\n";
        } else if (c < 0x20 || c == 0x7F) {
          out += '?';
        } else {
          out.push_back(static_cast<char>(c));
        }
      }
      return out;
    };

  auto find_first_control_char = [](std::string_view s) -> std::optional<std::pair<size_t, unsigned char>> {
      for (size_t i = 0; i < s.size(); ++i) {
        unsigned char c = static_cast<unsigned char>(s[i]);
        // In JSON strings, unescaped control characters (0x00-0x1F) are invalid.
        // Tabs/newlines/carriage returns are still control chars; they must be escaped as \t/\n/\r.
        if (c < 0x20) {
          return std::make_pair(i, c);
        }
      }
      return std::nullopt;
    };

  auto trim_crlf = [](std::string & s) {
      // Trim trailing CR and whitespace
      while (!s.empty() &&
        (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t'))
      {
        s.pop_back();
      }
      // Trim leading whitespace
      size_t start = 0;
      while (start < s.size() && (s[start] == ' ' || s[start] == '\t')) {
        ++start;
      }
      if (start > 0) {
        s.erase(0, start);
      }
    };

  auto dispatch_serial_line =
    [this, &sanitize_for_log, &find_first_control_char](
      const std::string & raw_line,
      int board_id,
      const char * board_label)
    {
      if (raw_line.empty()) {
        return;
      }

      // If we ever see NUL bytes in the assembled line, the stream is corrupted.
      // NUL is not valid in our line-based JSON protocol and can cause apparent truncation.
      if (raw_line.find('\0') != std::string::npos) {
        RCLCPP_WARN(this->get_logger(),
          "Dropping line containing NUL from %s (board %d). len=%zu head='%s'",
          board_label,
          board_id,
          raw_line.size(),
          sanitize_for_log(raw_line, 160).c_str());
        return;
      }

      const std::array<std::string_view, 14> prefixes = {
        "BATT", "PERF", "IMU", "TEMPERATURE", "VL53L0X", "ROBOCLAW", "ODOM",
        "SAFETY", "ESTOP", "DIAG", "FAULT", "SDIR", "SDLINE", "SDEOF"
      };

      auto is_header_at = [&](std::string_view s, size_t pos, std::string_view prefix) -> bool {
          if (pos + prefix.size() >= s.size()) {
            return false;
          }

          if (s.compare(pos, prefix.size(), prefix) != 0) {
            return false;
          }

          size_t idx = pos + prefix.size();
          if (idx < s.size() && std::isdigit(static_cast<unsigned char>(s[idx]))) {
            ++idx;
          }

          if (idx >= s.size() || s[idx] != ':') {
            return false;
          }

          // All TeensyV2 messages we parse are JSON objects.
          return (idx + 1U < s.size() && s[idx + 1U] == '{');
        };

      auto find_next_header = [&](std::string_view s, size_t start_pos) -> size_t {
          size_t best = std::string_view::npos;

          for (const auto & prefix : prefixes) {
            size_t pos = s.find(prefix, start_pos);
            while (pos != std::string_view::npos) {
              if (is_header_at(s, pos, prefix)) {
                if (best == std::string_view::npos || pos < best) {
                  best = pos;
                }
                break;
              }
              pos = s.find(prefix, pos + 1);
            }
          }
          return best;
        };

      std::string_view remaining(raw_line);
      while (!remaining.empty()) {
        size_t next = find_next_header(remaining, 1);
        std::string_view chunk = (next == std::string_view::npos) ? remaining : remaining.substr(0,
          next);

        if (!chunk.empty()) {
          current_board_id = board_id;
          auto timestamp = this->now();
          std::string msg(chunk);
          current_serial_frame = msg;

          // Fast-path reject for obviously-truncated JSON frames.
          // If we attempt to parse these, we spam parse errors and can starve other work.
          {
            size_t colon_pos = msg.find(':');
            if (colon_pos != std::string::npos) {
              std::string_view json_part(msg.c_str() + colon_pos + 1U, msg.size() - (colon_pos + 1U));
              while (!json_part.empty() &&
                (json_part.back() == ' ' || json_part.back() == '\t' || json_part.back() == '\r'))
              {
                json_part.remove_suffix(1);
              }
              if (!json_part.empty() && json_part.front() == '{' && json_part.back() != '}') {
                RCLCPP_WARN(this->get_logger(),
                  "Dropping truncated JSON from %s (board %d, type prefix '%.*s', len=%zu, tail='%s')",
                  board_label,
                  board_id,
                  static_cast<int>(std::min<size_t>(colon_pos, 16U)),
                  msg.c_str(),
                  msg.size(),
                  sanitize_for_log(std::string_view(msg).substr(msg.size() > 80 ? msg.size() - 80 : 0),
                    80)
                    .c_str());

                // Continue scanning; other messages may be present later in the line.
                if (next == std::string_view::npos) {
                  break;
                }
                remaining.remove_prefix(next);
                continue;
              }
            }
          }

          if (!message_parser_->ParseJsonMessage(msg, timestamp)) {
            RCLCPP_INFO_THROTTLE(
              this->get_logger(), *this->get_clock(), 5000,
              "Failed to parse JSON message from %s: '%s'", board_label,
              sanitize_for_log(msg, 220).c_str());
          }
        }

        if (next == std::string_view::npos) {
          break;
        }
        remaining.remove_prefix(next);
      }
    };

  while (serial_running_) {
      // Try to open serial ports if not connected
    if (board1_fd_ < 0) {
      int flags = O_RDWR | O_NOCTTY;   // Blocking I/O; use select() to wake us
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
      int flags = O_RDWR | O_NOCTTY;   // Blocking I/O; use select() to wake us
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
      int flags = O_RDWR | O_NOCTTY;   // Blocking I/O; use select() to wake us
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
    timeout.tv_usec = 100000;    // 100ms timeout

    int result = select(max_fd + 1, &read_fds, nullptr, nullptr, &timeout);

    if (result > 0) {
      if (board1_fd_ >= 0 && FD_ISSET(board1_fd_, &read_fds)) {
        ssize_t bytes_read = read(board1_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
          // Append by byte-count (not C-string) to avoid truncation if the stream contains NUL.
          line_buffer1.append(buffer, static_cast<size_t>(bytes_read));
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Board1 read %zd bytes", bytes_read);

            // Process complete lines
          size_t pos;
          while ((pos = line_buffer1.find('\n')) != std::string::npos) {
            std::string line = line_buffer1.substr(0, pos);
            line_buffer1.erase(0, pos + 1);
            trim_crlf(line);
            if (!line.empty()) {
              dispatch_serial_line(line, 1, "board 1");
            }
          }
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Board 1 read error: %s", strerror(errno));
          close(board1_fd_);
          board1_fd_ = -1;
        }
      }
      if (board2_fd_ >= 0 && FD_ISSET(board2_fd_, &read_fds)) {
        ssize_t bytes_read = read(board2_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
          line_buffer2.append(buffer, static_cast<size_t>(bytes_read));
          RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Board2 read %zd bytes", bytes_read);

          size_t pos;
          while ((pos = line_buffer2.find('\n')) != std::string::npos) {
            std::string line = line_buffer2.substr(0, pos);
            line_buffer2.erase(0, pos + 1);
            trim_crlf(line);
            if (!line.empty()) {
              dispatch_serial_line(line, 2, "board 2");
            }
          }
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Board 2 read error: %s", strerror(errno));
          close(board2_fd_);
          board2_fd_ = -1;
        }
      }
      if (board3_fd_ >= 0 && FD_ISSET(board3_fd_, &read_fds)) {
        ssize_t bytes_read = read(board3_fd_, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
          line_buffer3.append(buffer, static_cast<size_t>(bytes_read));
          RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
              "Board3 read %zd bytes", bytes_read);

          size_t pos;
          while ((pos = line_buffer3.find('\n')) != std::string::npos) {
            std::string line = line_buffer3.substr(0, pos);
            line_buffer3.erase(0, pos + 1);
            trim_crlf(line);
            if (!line.empty()) {
              dispatch_serial_line(line, 3, "board 3");
            }
          }
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(this->get_logger(), "Board 3 read error: %s", strerror(errno));
          close(board3_fd_);
          board3_fd_ = -1;
        }
      }
    } else if (result < 0) {
      RCLCPP_WARN(this->get_logger(), "select error: %s", strerror(errno));
      if (board1_fd_ >= 0) {close(board1_fd_);}
      if (board2_fd_ >= 0) {close(board2_fd_);}
      if (board3_fd_ >= 0) {close(board3_fd_);}
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

void TeensyBridge::HandleBatteryMessage(const MessageData & data, rclcpp::Time timestamp)
{
  auto battery_data = message_parser_->ParseBatteryData(data);
  if (battery_data.valid) {
    auto msg = message_parser_->ToBatteryStateMsg(battery_data, timestamp);
    battery_pub_->publish(msg);
  }
}

void TeensyBridge::HandleIMUMessage(const MessageData & data, rclcpp::Time timestamp)
{
  auto imu_data = message_parser_->ParseIMUData(data);
  if (imu_data.valid) {
    auto msg = message_parser_->ToImuMsg(imu_data, timestamp);

      // Publish to the appropriate sensor topic
    if (imu_data.sensor_id == 0) {
      imu_sensor0_pub_->publish(msg);
      RCLCPP_DEBUG(this->get_logger(), "Published IMU data for sensor 0");
    } else if (imu_data.sensor_id == 1) {
      imu_sensor1_pub_->publish(msg);
      RCLCPP_DEBUG(this->get_logger(), "Published IMU data for sensor 1");
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid IMU sensor ID: %d", imu_data.sensor_id);
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Received invalid IMU data");
  }
}

void TeensyBridge::HandlePerformanceMessage(const MessageData & data, rclcpp::Time timestamp)
{
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

void TeensyBridge::HandleSafetyMessage(const MessageData & data, rclcpp::Time timestamp)
{
  (void)timestamp;   // Suppress unused parameter warning
    // Extract safety state and publish E-stop status
  auto state_it = data.find("state");
  auto conditions_it = data.find("active_conditions");

  if (state_it != data.end() && conditions_it != data.end()) {
    bool estop_active = (state_it->second == "ESTOP" || state_it->second == "SHUTDOWN") ||
      (conditions_it->second == "true");

    // Update V2 status
    {
      std::lock_guard<std::mutex> lock(fault_mutex_);
      current_estop_status_.active = estop_active;
      if (!estop_active) {
        current_estop_status_.faults.clear();
      }
      // RCLCPP_INFO(this->get_logger(), "Publishing V2 ESTOP (active=%d)", current_estop_status_.active);
      estop_status_pub_->publish(current_estop_status_);
    }
  }
}


void TeensyBridge::HandleEstopMessage(const MessageData & data, rclcpp::Time timestamp)
{
  (void)timestamp;   // Suppress unused parameter warning
    // Log E-stop events
  auto active_it = data.find("active");
  if (active_it != data.end() && active_it->second == "true") {
    RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED");
    if (log_estop_raw_lines_) {
      RCLCPP_WARN(this->get_logger(), "E-STOP RAW (board %d): %s", current_board_id,
        sanitize_frame_for_log(current_serial_frame, 512).c_str());
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "E-STOP CLEARED");
    if (log_estop_raw_lines_) {
      RCLCPP_INFO(this->get_logger(), "E-STOP RAW (board %d): %s", current_board_id,
        sanitize_frame_for_log(current_serial_frame, 512).c_str());
    }
    
    // Clear faults in V2 status
    {
      std::lock_guard<std::mutex> lock(fault_mutex_);
      current_estop_status_.active = false;
      current_estop_status_.faults.clear();
      estop_status_pub_->publish(current_estop_status_);
    }
  }
}

void TeensyBridge::HandleDiagnosticMessage(const MessageData & data, rclcpp::Time timestamp)
{
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "HandleDiagnosticMessage from board %d (map size=%zu)", current_board_id, data.size());

  auto diagnostic_data = message_parser_->ParseDiagnosticData(data);
  if (diagnostic_data.valid) {
      // Some Teensy firmware versions return SD service responses wrapped as DIAG messages.
      // Route those payloads to the SD service handlers and avoid publishing them as diagnostics
      // (directory listings and file dumps can be very large).
    if (diagnostic_data.level == "SDIR") {
      RCLCPP_DEBUG(this->get_logger(), "Routing SDIR response from DIAG (len=%zu)",
        diagnostic_data.message.size());
      HandleSdirResponse(diagnostic_data.message);
      return;
    }
    if (diagnostic_data.level == "SDLINE") {
      RCLCPP_DEBUG(this->get_logger(), "Routing SDLINE response from DIAG (len=%zu)",
        diagnostic_data.message.size());
      HandleSdlineResponse(diagnostic_data.message);
      return;
    }
    if (diagnostic_data.level == "SDEOF") {
      RCLCPP_DEBUG(this->get_logger(), "Routing SDEOF response from DIAG");
      HandleSdeofResponse(diagnostic_data.message);
      return;
    }

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
          size_t source_start = source_pos + 7;   // Length of "source:"
          size_t source_end = message.find(",", source_start);
          if (source_end != std::string::npos) {
            source = message.substr(source_start, source_end - source_start);
          } else {
            source = message.substr(source_start);
          }
        }

          // Parse reason field
        size_t reason_pos = message.find("reason:");
        if (reason_pos != std::string::npos) {
          size_t reason_start = reason_pos + 7;   // Length of "reason:"
          size_t reason_end = message.find(",", reason_start);
          if (reason_end != std::string::npos) {
            reason = message.substr(reason_start, reason_end - reason_start);
          } else {
            reason = message.substr(reason_start);
          }
        }

          // Log ESTOP event
        RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED (from diagnostic): %s - %s",
            source.c_str(), reason.c_str());

        if (log_estop_raw_lines_) {
          RCLCPP_WARN(this->get_logger(), "E-STOP RAW (board %d): %s", current_board_id,
            sanitize_frame_for_log(current_serial_frame, 512).c_str());
        }

        // Update V2 Status
        {
          std::lock_guard<std::mutex> lock(fault_mutex_);
          current_estop_status_.active = true;
          
          bool found = false;
          for (auto & fault : current_estop_status_.faults) {
            if (fault.source == source) {
              fault.reason = reason;
              // fault.latching = (message.find("manual_reset:true") != std::string::npos);
              // timestamp update?
              found = true;
              break;
            }
          }
          if (!found) {
            sigyn_interfaces::msg::SystemFault new_fault;
            new_fault.source = source;
            new_fault.reason = reason;
            new_fault.description = message; // Or parse value?
            new_fault.latching = (message.find("manual_reset:true") != std::string::npos);
            new_fault.first_occurrence = this->now(); 
            current_estop_status_.faults.push_back(new_fault);
          }
          estop_status_pub_->publish(current_estop_status_);
        }
      }
    }

    auto msg = message_parser_->ToDiagnosticArrayMsg(diagnostic_data, timestamp);

    diagnostics_pub_->publish(msg);
  } else {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "ParseDiagnosticData returned invalid data, not publishing");
  }
}

void TeensyBridge::HandleFaultMessage(const MessageData & data, rclcpp::Time timestamp)
{
    // Check for active_fault first to ensure it's at the beginning
  auto active_it = data.find("active_fault");

  // If "active_fault" is present and false, we treat this as a Safety Heartbeat
  if (active_it != data.end() && active_it->second == "false") {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    current_estop_status_.active = false;
    current_estop_status_.faults.clear();
    estop_status_pub_->publish(current_estop_status_);
  }

  auto fault_data = message_parser_->ParseFaultData(data);
  if (fault_data.valid) {
      // Convert fault to diagnostic message
    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = timestamp;

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = fault_data.source;
    status.message = fault_data.description;
    status.hardware_id = "teensy_v2_board_" + std::to_string(current_board_id);

    if (fault_data.severity == "EMERGENCY_STOP" || fault_data.severity == "CRITICAL") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

        // Also trigger ESTOP if severity is EMERGENCY_STOP
      if (fault_data.severity == "EMERGENCY_STOP") {
        RCLCPP_WARN(this->get_logger(), "E-STOP TRIGGERED (from FAULT): %s - %s",
            fault_data.source.c_str(), fault_data.description.c_str());

        if (log_estop_raw_lines_) {
          RCLCPP_WARN(this->get_logger(), "E-STOP RAW (board %d): %s", current_board_id,
            sanitize_frame_for_log(current_serial_frame, 512).c_str());
        }

        // Update V2 status
        {
          std::lock_guard<std::mutex> lock(fault_mutex_);
          current_estop_status_.active = true;

          sigyn_interfaces::msg::SystemFault new_fault;
          new_fault.source = fault_data.source;
          // severity is "EMERGENCY_STOP" here
          new_fault.reason = fault_data.severity;
          new_fault.description = fault_data.description;
          new_fault.latching = true;
          new_fault.first_occurrence = timestamp;

          current_estop_status_.faults.push_back(new_fault);
          estop_status_pub_->publish(current_estop_status_);
        }
      }
    } else if (fault_data.severity == "WARNING") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    diagnostic_msgs::msg::KeyValue kv_severity;
    kv_severity.key = "severity";
    kv_severity.value = fault_data.severity;
    status.values.push_back(kv_severity);

    msg.status.push_back(status);
    diagnostics_pub_->publish(msg);
  }
}

void TeensyBridge::HandleTemperatureMessage(const MessageData & data, rclcpp::Time timestamp)
{
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
      temp_msg.variance = 0.1;    // Conservative variance estimate

        // Route to appropriate publisher based on sensor index
      if (i == 0 && temperature_motor0_pub_) {
        temperature_motor0_pub_->publish(temp_msg);
      } else if (i == 1 && temperature_motor1_pub_) {
        temperature_motor1_pub_->publish(temp_msg);
      }
    }

    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "Processed TEMPERATURE message: %d sensors, %zu temperature values",
      temperature_data.total_sensors, temperature_data.temperatures.size());
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid temperature data or empty temperatures array");
  }
}

void TeensyBridge::HandleVL53L0XMessage(const MessageData & data, rclcpp::Time timestamp)
{
  auto vl53l0x_data = message_parser_->ParseVL53L0XData(data);

  if (vl53l0x_data.valid) {
      // Publish range data for each sensor in the distances array
    for (size_t i = 0; i < vl53l0x_data.distances_mm.size() && i < 8; ++i) {
      if (vl53l0x_data.distances_mm[i] > 0) {    // 0 indicates invalid reading
        sensor_msgs::msg::Range range_msg;

          // Correct timestamp using age_us to get actual measurement time
        rclcpp::Time corrected_timestamp = timestamp;
        if (i < vl53l0x_data.age_us.size() && vl53l0x_data.age_us[i] > 0 &&
          vl53l0x_data.age_us[i] <= vl53_age_correction_max_us_)
        {
            // Subtract the age to get the actual measurement time
          corrected_timestamp = timestamp - rclcpp::Duration::from_nanoseconds(
              static_cast<int64_t>(vl53l0x_data.age_us[i]) * 1000);
        }

        range_msg.header.stamp = corrected_timestamp;
        range_msg.header.frame_id = "vl53l0x_" + std::to_string(i);
        range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
        range_msg.field_of_view = 0.44;    // ~25 degrees in radians for VL53L0X
        range_msg.min_range = 0.03;        // 3cm minimum range
        range_msg.max_range = 2.0;         // 2m maximum range

          // Convert mm to meters
        range_msg.range = vl53l0x_data.distances_mm[i] / 1000.0;

          // Route to appropriate publisher based on sensor index
        auto publishers = std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>{
          vl53l0x_sensor0_pub_, vl53l0x_sensor1_pub_, vl53l0x_sensor2_pub_, vl53l0x_sensor3_pub_,
          vl53l0x_sensor4_pub_, vl53l0x_sensor5_pub_, vl53l0x_sensor6_pub_, vl53l0x_sensor7_pub_
        };

        if (i < publishers.size() && publishers[i]) {
          publishers[i]->publish(range_msg);
          RCLCPP_DEBUG(this->get_logger(), "Published range for sensor %zu: %.3f m", i,
              range_msg.range);
        }
      }
    }
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Received invalid VL53L0X data");
  }
}

void TeensyBridge::HandleRoboClawMessage(const MessageData & data, rclcpp::Time timestamp)
{
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
    } else {
      roboclaw_status.message = "RoboClaw Error Code: " + error_it->second;
    }
  } else {
    roboclaw_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    roboclaw_status.message = "RoboClaw operating normally";
  }

    // Add all RoboClaw data as key-value pairs
  for (const auto & kv : data) {
    auto roboclaw_kv = diagnostic_msgs::msg::KeyValue();
    roboclaw_kv.key = kv.first;
    roboclaw_kv.value = kv.second;
    roboclaw_status.values.push_back(roboclaw_kv);
  }

  diag_msg.status.push_back(roboclaw_status);
  diagnostics_pub_->publish(diag_msg);
}

void TeensyBridge::HandleOdomMessage(const MessageData & data, rclcpp::Time timestamp)
{
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
    } else {
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
    } else {
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
    } else {
      RCLCPP_WARN(this->get_logger(), "Missing velocity data in odometry message");
    }

      // Set covariance matrices (based on wheel odometry characteristics)
      // Position covariance (uncertainty grows with distance traveled)
    odom_msg.pose.covariance[0] = 0.001;    // x
    odom_msg.pose.covariance[7] = 0.001;    // y
    odom_msg.pose.covariance[35] = 0.001;   // yaw

      // Velocity covariance (encoder-based, relatively accurate)
    odom_msg.twist.covariance[0] = 0.0001;    // vx
    odom_msg.twist.covariance[7] = 0.0001;    // vy
    odom_msg.twist.covariance[35] = 0.0001;   // wz

    odom_pub_->publish(odom_msg);

  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to parse odometry data: %s", e.what());
  }
}

void TeensyBridge::EstopCommandCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
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

void TeensyBridge::ConfigCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
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

void TeensyBridge::StatusTimerCallback()
{
    // Publish connection status
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 60000,
    "Status: Board1=%s, Board2=%s, Board3=%s",
    (board1_fd_ >= 0) ? "connected" : "disconnected",
    (board2_fd_ >= 0) ? "connected" : "disconnected",
    (board3_fd_ >= 0) ? "connected" : "disconnected");
}

void TeensyBridge::DiagnosticsTimerCallback()
{
    // Only publish parsing statistics if we have received any messages
  if (message_parser_) {
      // Check if there's any meaningful data to report
    auto stats_msg = message_parser_->GetParsingStatistics();

      // Only publish if we have received at least one message
    bool has_data = false;
    for (const auto & status : stats_msg.status) {
      for (const auto & kv : status.values) {
        if (kv.key == "total_received" && std::stoi(kv.value) > 0) {
          has_data = true;
          break;
        }
      }
      if (has_data) {break;}
    }

    if (has_data) {
      diagnostics_pub_->publish(stats_msg);
    } else {
        // Log that we're not receiving data, but don't spam with empty diagnostics
      static auto last_no_data_log = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_no_data_log).count() >= 30) {
        RCLCPP_WARN(this->get_logger(),
            "No data received from Teensy devices - diagnostic publishing suspended");
        last_no_data_log = now;
      }
    }
  }
}

void TeensyBridge::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Queue the twist message - all serial communication happens in the main loop
    // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.3f, angular.z=%.3f",
    //             msg->linear.x, msg->angular.z);

  std::ostringstream oss;
  oss << "TWIST:linear_x:" << msg->linear.x << ",angular_z:" << msg->angular.z << "\n";

  std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);
  outgoing_message_queue_.push(oss.str());

    // RCLCPP_INFO(this->get_logger(), "Queued TWIST command: %s", oss.str().c_str());
}

void TeensyBridge::CmdVelGripperCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Queue the twist message for the gripper board (board3)
  RCLCPP_INFO(this->get_logger(), "Received cmd_vel_gripper: linear.x=%.3f, angular.z=%.3f",
      msg->linear.x, msg->angular.z);

  std::ostringstream oss;
  oss << "TWIST:linear_x:" << msg->linear.x << ",angular_z:" << msg->angular.z << "\n";

  std::lock_guard<std::mutex> lock(gripper_queue_mutex_);
  gripper_message_queue_.push(oss.str());

  RCLCPP_INFO(this->get_logger(), "Queued gripper TWIST command: %s", oss.str().c_str());
}

void TeensyBridge::SendQueuedMessages()
{
  std::lock_guard<std::mutex> lock(outgoing_queue_mutex_);

  while (!outgoing_message_queue_.empty()) {
    const std::string & message = outgoing_message_queue_.front();

      // RCLCPP_INFO(this->get_logger(), "Sending message to Teensy: '%s'", message.c_str());

    if (message.rfind("SDDIR:", 0) == 0) {
      RCLCPP_INFO(this->get_logger(), "Sending SDDIR command to Teensy (board1)");
    }

    ssize_t bytes_written = write(board1_fd_, message.c_str(), message.length());
    if (bytes_written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write to serial");
      break;
    } else {
        // RCLCPP_INFO(this->get_logger(), "Sent %zd bytes to Teensy", bytes_written);
        // Force immediate transmission
      fsync(board1_fd_);
    }

    outgoing_message_queue_.pop();
  }
}

void TeensyBridge::SendGripperMessages()
{
  std::lock_guard<std::mutex> lock(gripper_queue_mutex_);

  while (!gripper_message_queue_.empty()) {
    const std::string & message = gripper_message_queue_.front();

    RCLCPP_INFO(this->get_logger(), "Sending gripper message: '%s'", message.c_str());

    if (board3_fd_ >= 0) {
      ssize_t bytes_written = write(board3_fd_, message.c_str(), message.length());
      if (bytes_written < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to gripper board (board3)");
        break;
      } else {
        RCLCPP_INFO(this->get_logger(), "Sent %zd bytes to gripper board", bytes_written);
          // Force immediate transmission
        fsync(board3_fd_);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Gripper board (board3) not connected, discarding message");
    }

    gripper_message_queue_.pop();
  }
}

void TeensyBridge::HandleSdGetDirRequest(
  const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Request> request,
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetDir::Response> response)
{

  (void)request;    // Unused parameter

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
        0,   // dir_entries_expected
        0,   // dir_entries_received
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
  auto status = completion_future.wait_for(std::chrono::seconds(sd_getdir_timeout_sec_));

  if (status == std::future_status::timeout) {
    RCLCPP_ERROR(this->get_logger(), "SD GetDir request timed out after %u seconds",
      sd_getdir_timeout_sec_);
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
  } else {
      // RCLCPP_INFO(this->get_logger(), "SD GetDir request completed successfully");
  }
}

void TeensyBridge::HandleSdGetFileRequest(
  const std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Request> request,
  std::shared_ptr<sigyn_interfaces::srv::TeensySdGetFile::Response> response)
{

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
        0,   // dir_entries_expected (unused for get_file)
        0,   // dir_entries_received (unused for get_file)
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
  auto status = completion_future.wait_for(std::chrono::seconds(sd_getfile_timeout_sec_));

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

void TeensyBridge::HandleSdirResponse(const std::string & data)
{
    // RCLCPP_INFO(this->get_logger(), "Received SDIR response: '%s'", data.c_str());

  std::lock_guard<std::mutex> lock(service_mutex_);

  // Don't assume the SD directory request is at the front of the queue.
  PendingServiceRequest matched_request;
  bool found = false;
  std::queue<PendingServiceRequest> temp_queue;
  while (!pending_service_requests_.empty()) {
    auto req = pending_service_requests_.front();
    pending_service_requests_.pop();
    if (!found && req.service_type == "get_dir") {
      matched_request = std::move(req);
      found = true;
      continue;
    }
    temp_queue.push(std::move(req));
  }
  pending_service_requests_ = std::move(temp_queue);

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "Received SDIR response but no matching pending request");
    return;
  }

  // SDIR can be either:
  // - Legacy: full directory listing in one frame (tab-delimited entries)
  // - Streamed: header-only, followed by SDLINE entries and SDEOF
  const bool looks_like_legacy_listing = (data.find('\t') != std::string::npos);
  if (looks_like_legacy_listing) {
    if (matched_request.dir_response) {
      matched_request.dir_response->directory_listing = data;
      matched_request.dir_response->success = true;
      matched_request.dir_response->error_message = "";
    }
    if (matched_request.completion_promise) {
      try {
        matched_request.completion_promise->set_value(true);
      } catch (const std::future_error & e) {
        RCLCPP_WARN(this->get_logger(),
          "Directory listing completion promise already satisfied: %s", e.what());
      }
    }
    RCLCPP_INFO(this->get_logger(), "Completed legacy directory listing request (single SDIR frame)");
    return;
  }

  auto parse_dir_expected_entries = [](const std::string & header) -> uint32_t {
      // Header example: "TOTAL FILES: 21, OMITTED_FILES: 1"
      auto find_uint_after = [&](const char * needle) -> std::optional<uint32_t> {
          const std::string_view s(header);
          const std::string_view n(needle);
          size_t pos = s.find(n);
          if (pos == std::string_view::npos) {
            return std::nullopt;
          }
          pos += n.size();
          while (pos < s.size() && (s[pos] == ' ' || s[pos] == '\t')) {
            ++pos;
          }
          uint32_t value = 0;
          bool any = false;
          while (pos < s.size() && std::isdigit(static_cast<unsigned char>(s[pos]))) {
            any = true;
            value = static_cast<uint32_t>(value * 10U + static_cast<uint32_t>(s[pos] - '0'));
            ++pos;
          }
          return any ? std::optional<uint32_t>(value) : std::nullopt;
        };

      auto total_opt = find_uint_after("TOTAL FILES:");
      auto omitted_opt = find_uint_after("OMITTED_FILES:");
      if (!total_opt.has_value()) {
        return 0U;
      }
      const uint32_t total = total_opt.value();
      const uint32_t omitted = omitted_opt.value_or(0U);
      return (omitted >= total) ? 0U : (total - omitted);
    };

  matched_request.accumulated_content.clear();
  matched_request.dir_entries_received = 0;
  matched_request.dir_entries_expected = parse_dir_expected_entries(data);
  const auto now = this->get_clock()->now();
  matched_request.accumulated_content += data;
  matched_request.accumulated_content += "\n";
  matched_request.last_activity_time = now;

  // Put it back in the queue to keep collecting SDLINE until SDEOF.
  pending_service_requests_.push(std::move(matched_request));
}

void TeensyBridge::HandleSdlineResponse(const std::string & data)
{
  std::lock_guard<std::mutex> lock(service_mutex_);

  // Append to the first pending get_file or get_dir request (do not assume it's at the front).
  const auto now = this->get_clock()->now();
  bool updated = false;
  bool completed = false;
  PendingServiceRequest completed_request;
  std::queue<PendingServiceRequest> temp_queue;
  while (!pending_service_requests_.empty()) {
    auto req = pending_service_requests_.front();
    pending_service_requests_.pop();
    if (!updated && (req.service_type == "get_file" || req.service_type == "get_dir")) {
      req.accumulated_content += data + "\n";
      req.last_activity_time = now;

      if (req.service_type == "get_dir") {
        req.dir_entries_received += 1;

        // Robust completion: if the SDIR header told us how many entries to expect,
        // complete as soon as we've received them, even if SDEOF is lost.
        if (req.dir_entries_expected > 0 && req.dir_entries_received >= req.dir_entries_expected) {
          completed = true;
          completed_request = std::move(req);
          updated = true;
          continue; // Do not re-queue this request.
        }
      }
      updated = true;
    }
    temp_queue.push(std::move(req));
  }
  pending_service_requests_ = std::move(temp_queue);

  if (completed) {
    if (completed_request.dir_response) {
      completed_request.dir_response->success = true;
      completed_request.dir_response->error_message = "";
      completed_request.dir_response->directory_listing = completed_request.accumulated_content;
    }

    if (completed_request.completion_promise) {
      try {
        completed_request.completion_promise->set_value(true);
      } catch (const std::future_error & e) {
        RCLCPP_WARN(this->get_logger(), "SD service completion promise already satisfied: %s",
          e.what());
      }
    }
  }
}

void TeensyBridge::HandleSdeofResponse(const std::string & data)
{
  std::lock_guard<std::mutex> lock(service_mutex_);

  const bool is_dir_eof = (data == "DIR");
  const auto now = this->get_clock()->now();

  // Prefer completing the matching pending request:
  // - If Teensy indicates directory EOF ("DIR"), complete get_dir.
  // - Otherwise, complete get_file (historically SDEOF was primarily for file dumps).
  PendingServiceRequest matched_request;
  bool found = false;
  std::queue<PendingServiceRequest> temp_queue;
  while (!pending_service_requests_.empty()) {
    auto req = pending_service_requests_.front();
    pending_service_requests_.pop();
    if (!found && ((is_dir_eof && req.service_type == "get_dir") || (!is_dir_eof && req.service_type == "get_file"))) {
      matched_request = std::move(req);
      found = true;
      continue;
    }
    temp_queue.push(std::move(req));
  }
  pending_service_requests_ = std::move(temp_queue);

  if (!found) {
    // Fallback: complete a pending get_dir request if present.
    std::queue<PendingServiceRequest> temp_queue2;
    while (!pending_service_requests_.empty()) {
      auto req = pending_service_requests_.front();
      pending_service_requests_.pop();
      if (!found && ((is_dir_eof && req.service_type == "get_file") || (!is_dir_eof && req.service_type == "get_dir"))) {
        matched_request = std::move(req);
        found = true;
        continue;
      }
      temp_queue2.push(std::move(req));
    }
    pending_service_requests_ = std::move(temp_queue2);
  }

  if (!found) {
    return;
  }

  if (matched_request.service_type == "get_dir") {
    if (matched_request.dir_response) {
      matched_request.dir_response->success = true;
      matched_request.dir_response->error_message = "";
      matched_request.dir_response->directory_listing = matched_request.accumulated_content;
    }

  } else if (matched_request.service_type == "get_file") {
    if (matched_request.file_response) {
      matched_request.file_response->success = true;
      matched_request.file_response->error_message = "";
      matched_request.file_response->file_contents = matched_request.accumulated_content;
    }
  }

  if (matched_request.completion_promise) {
    try {
      matched_request.completion_promise->set_value(true);
    } catch (const std::future_error & e) {
      RCLCPP_WARN(this->get_logger(), "SD service completion promise already satisfied: %s",
        e.what());
    }
  }
}

void TeensyBridge::HandleResetFault(const std::shared_ptr<sigyn_interfaces::srv::ResetFault::Request> request,
                                    std::shared_ptr<sigyn_interfaces::srv::ResetFault::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Received ResetFault request via service. Source: '%s', Reason: '%s'",
              request->source.c_str(), request->reason.c_str());

  // Construct command for Teensy
  // If specific source is requested, try to send targeted reset if supported,
  // otherwise send generic reset (which clears all safety flags in current firmware).
  // Firmware Todo: Support 'cmd:reset,target:<source>'

  std::string command;
  if (request->source.empty() || request->source == "all") {
    command = "cmd:reset,value:true\n";
  } else {
    // Attempt targeted reset command (firmware update required to fully support this granularity)
    command = "cmd:reset_fault,source:" + request->source + "\n";
  }

  // Send to all boards since we don't track which board originated the fault in the map key yet (could improve this)
  if (board1_fd_ >= 0) {
    if (write(board1_fd_, command.c_str(), command.length()) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write reset command to board1");
      response->success = false;
      response->message = "Failed to write to board1";
      return;
    }
  }
  // Sending to others just in case? Usually motion control is board 1.

  response->success = true;
  response->message = "Reset command sent: " + command;
}

void TeensyBridge::UpdateAndPublishFaults()
{
  std::lock_guard<std::mutex> lock(fault_mutex_);
  estop_status_pub_->publish(current_estop_status_);
}

}  // namespace sigyn_to_sensor_v2
