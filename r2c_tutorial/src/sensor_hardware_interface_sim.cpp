// Copyright 2024 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "r2c_tutorial/sensor_hardware_interface_sim.hpp"

#include <limits>
#include <vector>
#include <string>

namespace r2c_tutorial
{

bool SensorHardwareInterfaceSim::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, gz::sim::Entity> & /*joints*/,
  const hardware_interface::HardwareInfo & hardware_info,
  gz::sim::EntityComponentManager & ecm,
  unsigned int update_rate)
{
  node_ = model_nh;
  ecm_ = &ecm;
  update_rate_ = update_rate;

  RCLCPP_INFO(
    node_->get_logger(),
    "Initializing SensorHardwareInterfaceSim for Gazebo simulation");

  // Store hardware info for later use
  info_ = hardware_info;

  // Extract sensor names from hardware info
  for (const auto & sensor : info_.sensors) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Found sensor: %s", sensor.name.c_str());

    // Identify sensor types by name
    if (sensor.name.find("temperature") != std::string::npos) {
      temperature_sensor_name_ = sensor.name;
    } else if (sensor.name.find("battery") != std::string::npos) {
      battery_sensor_name_ = sensor.name;
    } else if (sensor.name.find("range") != std::string::npos) {
      range_sensor_name_ = sensor.name;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "SensorHardwareInterfaceSim initialized successfully");
  RCLCPP_INFO(
    node_->get_logger(),
    "  Temperature sensor: %s", temperature_sensor_name_.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  Battery sensor: %s", battery_sensor_name_.c_str());
  RCLCPP_INFO(
    node_->get_logger(),
    "  Range sensor: %s", range_sensor_name_.c_str());

  return true;
}

hardware_interface::CallbackReturn SensorHardwareInterfaceSim::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Store hardware info
  info_ = info;

  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "on_init() called");

  // Initialize sensor values to safe defaults
  temperature_ = 25.0;  // Room temperature
  voltage_ = 12.0;      // Nominal battery voltage
  current_ = 2.0;       // Moderate current draw
  range_ = 1.0;         // 1 meter distance

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorHardwareInterfaceSim::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "Configuring simulated sensor hardware");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SensorHardwareInterfaceSim::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Temperature state interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      temperature_sensor_name_.empty() ? "temperature_sensor" : temperature_sensor_name_,
      "temperature",
      &temperature_));

  // Battery voltage state interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      battery_sensor_name_.empty() ? "battery_sensor" : battery_sensor_name_,
      "voltage",
      &voltage_));

  // Battery current state interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      battery_sensor_name_.empty() ? "battery_sensor" : battery_sensor_name_,
      "current",
      &current_));

  // Range state interface
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      range_sensor_name_.empty() ? "range_sensor" : range_sensor_name_,
      "range",
      &range_));

  RCLCPP_INFO(
    node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SensorHardwareInterfaceSim::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Temperature command interface (for test value injection)
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      temperature_sensor_name_.empty() ? "temperature_sensor" : temperature_sensor_name_,
      "temperature",
      &temperature_cmd_));

  // Battery voltage command interface
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      battery_sensor_name_.empty() ? "battery_sensor" : battery_sensor_name_,
      "voltage",
      &voltage_cmd_));

  // Battery current command interface
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      battery_sensor_name_.empty() ? "battery_sensor" : battery_sensor_name_,
      "current",
      &current_cmd_));

  // Range command interface
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      range_sensor_name_.empty() ? "range_sensor" : range_sensor_name_,
      "range",
      &range_cmd_));

  RCLCPP_INFO(
    node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

hardware_interface::CallbackReturn SensorHardwareInterfaceSim::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "Activating simulated sensor hardware");

  // Record start time for simulation
  if (node_) {
    start_time_ = node_->now().seconds();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorHardwareInterfaceSim::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
    "Deactivating simulated sensor hardware");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorHardwareInterfaceSim::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  // Calculate elapsed time for simulation
  double t = time.seconds() - start_time_;

  // Check if we should use commanded values (for testing)
  // If any command is non-zero, use command mode
  use_commands_ = (temperature_cmd_ != 0.0) || 
                  (voltage_cmd_ != 0.0) || 
                  (current_cmd_ != 0.0) || 
                  (range_cmd_ != 0.0);

  if (use_commands_) {
    // Use commanded test values
    if (temperature_cmd_ != 0.0) temperature_ = temperature_cmd_;
    if (voltage_cmd_ != 0.0) voltage_ = voltage_cmd_;
    if (current_cmd_ != 0.0) current_ = current_cmd_;
    if (range_cmd_ != 0.0) range_ = range_cmd_;
  } else {
    // Generate simulated sensor values
    
    // Temperature: Room temperature with periodic variation
    // Simulates ambient temperature changes + system heating
    temperature_ = 25.0 + 5.0 * std::sin(t * 0.1) + 2.0 * std::sin(t * 0.05);
    
    // Voltage: Slowly decreasing battery
    // Starts at 12.0V, drains to ~11.0V over time with some noise
    voltage_ = 12.0 - 0.0001 * t + 0.05 * std::sin(t * 0.3);
    voltage_ = std::max(10.5, voltage_);  // Don't go below 10.5V
    
    // Current: Variable load simulation
    // Base load with periodic spikes (simulating motors, sensors)
    current_ = 2.0 + 1.0 * std::sin(t * 0.5) + 0.5 * std::sin(t * 1.2);
    current_ = std::max(0.0, current_);  // Current can't be negative
    
    // Range: Distance sensor with obstacle simulation
    // Periodic variation simulating objects at different distances
    range_ = 1.0 + 0.5 * std::sin(t * 0.2) + 0.3 * std::cos(t * 0.15);
    range_ = std::max(0.03, std::min(2.0, range_));  // Clamp to sensor range
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorHardwareInterfaceSim::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // In simulation, write() just validates that commands are within reasonable bounds
  // The actual command values are already stored in the command interface variables
  // and will be used in the next read() cycle if non-zero

  // Validate commanded values (optional safety checks)
  if (temperature_cmd_ < -40.0 || temperature_cmd_ > 125.0) {
    if (temperature_cmd_ != 0.0) {  // 0 means "use simulation"
      RCLCPP_WARN(
        node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
        "Temperature command out of range: %.1f°C", temperature_cmd_);
    }
  }

  if (voltage_cmd_ < 0.0 || voltage_cmd_ > 20.0) {
    if (voltage_cmd_ != 0.0) {
      RCLCPP_WARN(
        node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
        "Voltage command out of range: %.2fV", voltage_cmd_);
    }
  }

  if (current_cmd_ < 0.0 || current_cmd_ > 50.0) {
    if (current_cmd_ != 0.0) {
      RCLCPP_WARN(
        node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
        "Current command out of range: %.2fA", current_cmd_);
    }
  }

  if (range_cmd_ < 0.0 || range_cmd_ > 5.0) {
    if (range_cmd_ != 0.0) {
      RCLCPP_WARN(
        node_ ? node_->get_logger() : rclcpp::get_logger("SensorHardwareInterfaceSim"),
        "Range command out of range: %.3fm", range_cmd_);
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace r2c_tutorial

// Export the plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  r2c_tutorial::SensorHardwareInterfaceSim,
  gz_ros2_control::GazeboSimSystemInterface)
