// Copyright (c) 2024 Sigyn Robotics
// Licensed under the MIT License

// System includes
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstring>

#include "sigyn_hardware_interface/sigyn_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sigyn_hardware_interface
{

hardware_interface::CallbackReturn SigynSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize configuration
  config_.teensy_port = info_.hardware_parameters["port"];
  config_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  config_.wheel_diameter = std::stod(info_.hardware_parameters["wheel_diameter"]);
  config_.wheel_base = std::stod(info_.hardware_parameters["wheel_base"]);
  config_.pulses_per_revolution = std::stoi(info_.hardware_parameters["pulses_per_revolution"]);
  config_.command_timeout = std::stod(info_.hardware_parameters["command_timeout"]);
  config_.read_timeout = std::stod(info_.hardware_parameters["read_timeout"]);

  // Calculate wheel radius (remove this line since we don't have wheel_radius_ variable)

  // Initialize vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_previous_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  encoder_positions_.resize(2, 0);
  encoder_positions_previous_.resize(2, 0);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Check joint interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        getLogger(),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        getLogger(),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        getLogger(),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        getLogger(),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        getLogger(),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SigynSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SigynSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SigynSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Activating SigynSystem...");

  // Open serial connection to TeensyV2
  if (!initializeTeensyCommunication())
  {
    RCLCPP_ERROR(getLogger(), "Failed to open serial connection to %s", config_.teensy_port.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize joint positions and velocities
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
    hw_commands_previous_[i] = 0.0;
  }

  // Initialize encoder values
  encoder_positions_[0] = 0;  // left
  encoder_positions_[1] = 0;  // right
  encoder_positions_previous_[0] = 0;
  encoder_positions_previous_[1] = 0;
  encoder_initialized_ = false;

  // Initialize timestamps
  last_read_time_ = rclcpp::Clock().now();
  last_command_time_ = rclcpp::Clock().now();

  RCLCPP_INFO(getLogger(), "Successfully activated SigynSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SigynSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Deactivating SigynSystem...");
  
  // Stop motors before closing
  sendVelocityCommand(0.0, 0.0);
  
  // Close serial connection
  closeTeensyCommunication();
  
  RCLCPP_INFO(getLogger(), "Successfully deactivated SigynSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SigynSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(getLogger(), "Configuring SigynSystem...");
  
  // Initialize communication
  communication_active_ = false;
  teensy_fd_ = -1;
  
  RCLCPP_INFO(getLogger(), "Successfully configured SigynSystem");
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool SigynSystem::initializeTeensyCommunication()
{
  teensy_fd_ = open(config_.teensy_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (teensy_fd_ < 0)
  {
    RCLCPP_ERROR(getLogger(), "Failed to open serial port %s: %s", config_.teensy_port.c_str(), strerror(errno));
    return false;
  }

  struct termios tty;
  if (tcgetattr(teensy_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(getLogger(), "Error from tcgetattr: %s", strerror(errno));
    close(teensy_fd_);
    return false;
  }

  // Set baud rate
  speed_t speed;
  switch (config_.baud_rate)
  {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:
      RCLCPP_ERROR(getLogger(), "Unsupported baud rate: %d", config_.baud_rate);
      close(teensy_fd_);
      return false;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // Set 8N1
  tty.c_cflag &= ~PARENB;   // No parity
  tty.c_cflag &= ~CSTOPB;   // One stop bit
  tty.c_cflag &= ~CSIZE;    // Clear size mask
  tty.c_cflag |= CS8;       // 8 bits
  tty.c_cflag &= ~CRTSCTS;  // No flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

  // Make raw
  cfmakeraw(&tty);

  // Flush Port, then applies attributes
  tcflush(teensy_fd_, TCIFLUSH);
  if (tcsetattr(teensy_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(getLogger(), "Error from tcsetattr: %s", strerror(errno));
    close(teensy_fd_);
    return false;
  }

  communication_active_ = true;
  RCLCPP_INFO(getLogger(), "Successfully opened serial connection to %s at %d baud", 
              config_.teensy_port.c_str(), config_.baud_rate);
  return true;
}

void SigynSystem::closeTeensyCommunication()
{
  if (communication_active_ && teensy_fd_ >= 0)
  {
    close(teensy_fd_);
    communication_active_ = false;
    RCLCPP_INFO(getLogger(), "Closed serial connection");
  }
}

hardware_interface::return_type SigynSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!communication_active_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Read encoder data from TeensyV2
  if (readEncoderData())
  {
    last_read_time_ = time;
    
    // Convert encoder counts to position and velocity
    encoderPositionsToWheelPositions();
  }
  else
  {
    // Check for read timeout
    auto time_since_last_read = time - last_read_time_;
    if (time_since_last_read.seconds() > config_.read_timeout)
    {
      static auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(getLogger(), clock, 1000,
                           "Read timeout - no encoder data received for %.3f seconds",
                           time_since_last_read.seconds());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SigynSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!communication_active_)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Convert joint velocity commands to wheel velocities
  if (!std::isnan(hw_commands_[0]) && !std::isnan(hw_commands_[1]))
  {
    double left_wheel_velocity = hw_commands_[0];   // rad/s
    double right_wheel_velocity = hw_commands_[1];  // rad/s

    // Send velocity command to TeensyV2
    if (sendVelocityCommand(left_wheel_velocity, right_wheel_velocity))
    {
      last_command_time_ = time;
    }
    else
    {
      static auto clock = rclcpp::Clock();
      RCLCPP_WARN_THROTTLE(getLogger(), clock, 1000,
                           "Failed to send velocity command to Teensy");
    }
  }

  return hardware_interface::return_type::OK;
}

bool SigynSystem::readEncoderData()
{
  std::string data = readFromSerial();
  if (!data.empty())
  {
    return parseEncoderMessage(data);
  }
  return false;
}

void SigynSystem::encoderPositionsToWheelPositions()
{
  if (!encoder_initialized_)
  {
    encoder_initialized_ = true;
    return;
  }
  
  // Calculate encoder differences
  int32_t left_diff = encoder_positions_[0] - encoder_positions_previous_[0];
  int32_t right_diff = encoder_positions_[1] - encoder_positions_previous_[1];

  // Convert encoder counts to radians
  double left_delta_pos = (left_diff * 2.0 * M_PI) / config_.pulses_per_revolution;
  double right_delta_pos = (right_diff * 2.0 * M_PI) / config_.pulses_per_revolution;

  // Update positions
  hw_positions_[0] += left_delta_pos;  // Left wheel
  hw_positions_[1] += right_delta_pos; // Right wheel

  // Calculate velocities (rad/s) - simplified version
  static rclcpp::Time prev_time = rclcpp::Clock().now();
  rclcpp::Time current_time = rclcpp::Clock().now();
  double dt = (current_time - prev_time).seconds();
  if (dt > 0.0)
  {
    hw_velocities_[0] = left_delta_pos / dt;  // Left wheel
    hw_velocities_[1] = right_delta_pos / dt; // Right wheel
  }

  // Store previous values
  encoder_positions_previous_[0] = encoder_positions_[0];
  encoder_positions_previous_[1] = encoder_positions_[1];
  prev_time = current_time;
}

bool SigynSystem::sendVelocityCommand(double left_vel_rad_s, double right_vel_rad_s)
{
  // Convert rad/s to RPM for TeensyV2
  double left_rpm = (left_vel_rad_s * 60.0) / (2.0 * M_PI);
  double right_rpm = (right_vel_rad_s * 60.0) / (2.0 * M_PI);

  // Create command string in TeensyV2 format
  std::stringstream ss;
  ss << "MOTOR," << std::fixed << std::setprecision(2) << left_rpm << "," << right_rpm << "\n";
  std::string command = ss.str();

  ssize_t bytes_written = writeToSerial(command);
  if (bytes_written < 0)
  {
    static auto clock = rclcpp::Clock();
    RCLCPP_ERROR_THROTTLE(getLogger(), clock, 1000,
                          "Failed to write velocity command: %s", strerror(errno));
    return false;
  }

  return true;
}

bool SigynSystem::parseEncoderMessage(const std::string& message)
{
  // Expected format: "ENCODER,left_count,right_count,timestamp"
  std::vector<std::string> tokens;
  std::stringstream ss(message);
  std::string token;

  while (std::getline(ss, token, ','))
  {
    tokens.push_back(token);
  }

  if (tokens.size() != 4)
  {
    static auto clock = rclcpp::Clock();
    RCLCPP_WARN_THROTTLE(getLogger(), clock, 1000,
                         "Invalid encoder message format: %s", message.c_str());
    return false;
  }

  try
  {
    if (tokens[0] == "ENCODER")
    {
      encoder_positions_[0] = std::stoi(tokens[1]);  // left
      encoder_positions_[1] = std::stoi(tokens[2]);  // right
      // uint32_t timestamp = std::stoul(tokens[3]); // TeensyV2 timestamp if needed
      return true;
    }
  }
  catch (const std::exception& e)
  {
    static auto clock = rclcpp::Clock();
    RCLCPP_WARN_THROTTLE(getLogger(), clock, 1000,
                         "Failed to parse encoder values: %s", e.what());
    return false;
  }

  return false;
}

ssize_t SigynSystem::writeToSerial(const std::string& data)
{
  if (!communication_active_ || teensy_fd_ < 0)
  {
    return -1;
  }

  return ::write(teensy_fd_, data.c_str(), data.length());
}

std::string SigynSystem::readFromSerial()
{
  if (!communication_active_ || teensy_fd_ < 0)
  {
    return "";
  }

  char read_buffer[256];
  ssize_t bytes_read = ::read(teensy_fd_, read_buffer, sizeof(read_buffer) - 1);
  
  if (bytes_read > 0)
  {
    read_buffer[bytes_read] = '\0';
    return std::string(read_buffer);
  }
  
  return "";
}



}  // namespace sigyn_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  sigyn_hardware_interface::SigynSystem, hardware_interface::SystemInterface)
