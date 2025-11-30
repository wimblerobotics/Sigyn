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

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Gazebo includes
#include <gz/sim/System.hh>
#include <gz_ros2_control/gz_system_interface.hpp>

namespace r2c_tutorial
{

/**
 * @brief Gazebo simulation version of sensor hardware interface.
 * 
 * This class provides simulated sensor data for Gazebo environments.
 * It inherits from GazeboSimSystemInterface to integrate with gz_ros2_control.
 * 
 * The simulation generates synthetic sensor data:
 * - Temperature: Sinusoidal variation around room temperature
 * - Voltage: Slowly decreasing (simulating battery drain)
 * - Current: Variable load simulation
 * - Range: Distance sensor with periodic variation
 * 
 * Command interfaces allow injecting test values during simulation.
 */
class SensorHardwareInterfaceSim : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  /**
   * @brief Initialize the simulated sensor hardware with Gazebo.
   * 
   * @param model_nh ROS 2 node for the model
   * @param joints Map of joint names to Gazebo entities
   * @param hardware_info Hardware description from URDF
   * @param ecm Gazebo Entity-Component Manager
   * @param update_rate Update rate for the simulation
   * @return true if initialization successful
   */
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & ecm,
    unsigned int update_rate) override;

  /**
   * @brief Initialize the hardware interface.
   * 
   * Called during controller_manager initialization.
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware interface.
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces for sensors.
   * 
   * Creates state interfaces for:
   * - temperature_sensor/temperature
   * - battery_sensor/voltage
   * - battery_sensor/current
   * - range_sensor/range
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces for testing.
   * 
   * Command interfaces allow injecting test values:
   * - temperature_sensor/temperature
   * - battery_sensor/voltage
   * - battery_sensor/current
   * - range_sensor/range
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Activate the hardware interface.
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface.
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read simulated sensor data.
   * 
   * Generates synthetic sensor values based on simulation time.
   * If command values have been written (for testing), uses those instead.
   * 
   * @param time Current ROS time
   * @param period Time since last read
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief Process command data (for test value injection).
   * 
   * @param time Current ROS time
   * @param period Time since last write
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ROS node for logging
  rclcpp::Node::SharedPtr node_;

  // Gazebo references (for future integration with Gazebo sensors)
  gz::sim::EntityComponentManager * ecm_{nullptr};
  unsigned int update_rate_{0};

  // Sensor state values (read by controllers)
  double temperature_{25.0};        // Temperature in Celsius
  double voltage_{12.0};            // Battery voltage in V
  double current_{2.0};             // Battery current in A
  double range_{1.0};               // Distance sensor in meters

  // Command values (for injecting test values)
  double temperature_cmd_{0.0};
  double voltage_cmd_{0.0};
  double current_cmd_{0.0};
  double range_cmd_{0.0};

  // Simulation parameters
  double start_time_{0.0};          // Simulation start time for offset
  bool use_commands_{false};        // Flag to use commanded values instead of simulation

  // Configuration from URDF
  std::string temperature_sensor_name_;
  std::string battery_sensor_name_;
  std::string range_sensor_name_;
};

}  // namespace r2c_tutorial
