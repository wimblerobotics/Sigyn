// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file sigyn_system.hpp
 * @brief ROS2 Control Hardware Interface for Sigyn robot
 * 
 * Provides hardware abstraction layer for motor control and sensor integration
 * through the TeensyV2 embedded system. This interface allows the robot to work
 * with standard ROS2 controllers while maintaining the existing safety features
 * and real-time performance of the TeensyV2 system.
 * 
 * @author Wimble Robotics
 * @date 2025
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace sigyn_hardware_interface
{

/**
 * @brief Hardware interface for Sigyn robot differential drive system
 * 
 * This class implements the ROS2 Control SystemInterface for the Sigyn robot,
 * providing communication with the TeensyV2 embedded system that controls
 * the RoboClaw motor controllers. It maintains compatibility with the existing
 * TeensyV2 architecture while providing a standard ROS2 Control interface.
 * 
 * Features:
 * - Differential drive motor control
 * - Real-time encoder feedback
 * - Safety monitoring integration
 * - Serial communication with TeensyV2
 * - Configurable motor parameters
 */
class SigynSystem : public hardware_interface::SystemInterface
{
public:
  SigynSystem();
  virtual ~SigynSystem();

  // SystemInterface implementation
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Configuration parameters
  struct Config {
    std::string teensy_port = "/dev/teensy_sensor";
    int baud_rate = 921600;
    double wheel_diameter = 0.102224144529039;  // From TeensyV2 constants
    double wheel_base = 0.3906;                 // From TeensyV2 constants
    uint32_t pulses_per_revolution = 1000;      // From TeensyV2 constants
    double command_timeout = 0.5;               // Seconds
    double read_timeout = 0.1;                  // Seconds
  } config_;

  // Joint state vectors (left_wheel, right_wheel)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_commands_previous_;

  // Communication
  int teensy_fd_;
  std::atomic<bool> communication_active_;
  mutable std::mutex communication_mutex_;
  
  // Timing
  rclcpp::Time last_command_time_;
  rclcpp::Time last_read_time_;
  
  // Encoder tracking
  std::vector<int32_t> encoder_positions_;
  std::vector<int32_t> encoder_positions_previous_;
  bool encoder_initialized_;

  // Helper methods
  bool initializeTeensyCommunication();
  void closeTeensyCommunication();
  bool sendVelocityCommand(double left_vel, double right_vel);
  bool readEncoderData();
  bool parseEncoderMessage(const std::string& message);
  void wheelVelocitiesToMotorSpeeds(double left_vel, double right_vel, 
                                   int32_t& left_qpps, int32_t& right_qpps);
  void encoderPositionsToWheelPositions();
  void encoderVelocitiesToWheelVelocities(int32_t left_qpps, int32_t right_qpps);
  
  // Serial communication helpers
  bool configureSerialPort();
  ssize_t writeToSerial(const std::string& data);
  std::string readFromSerial();
  
  // Logging
  rclcpp::Logger getLogger() const { return rclcpp::get_logger("SigynSystem"); }
};

}  // namespace sigyn_hardware_interface
