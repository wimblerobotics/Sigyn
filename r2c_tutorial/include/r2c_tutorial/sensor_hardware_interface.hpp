#ifndef R2C_TUTORIAL__SENSOR_HARDWARE_INTERFACE_HPP_
#define R2C_TUTORIAL__SENSOR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace r2c_tutorial
{

/**
 * @brief Hardware interface for sensor data from Teensy
 * 
 * This hardware interface subscribes to sensor topics published by
 * sigyn_to_sensor_v2 (for real hardware) or simulated topics (for Gazebo)
 * and exposes the sensor values through ros2_control state interfaces.
 * 
 * State Interfaces:
 * - temperature_sensor/temperature (°C)
 * - voltage_sensor/voltage (V) 
 * - current_sensor/current (A)
 * - vl53l0x_sensor/range (m)
 */
class SensorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SensorHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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
  // Sensor state values (read by controllers)
  double temperature_{25.0};  // °C
  double voltage_{12.0};      // V
  double current_{0.5};       // A
  double range_{0.5};         // m

  // Sensor command values (write by controllers in sim mode)
  double temperature_cmd_{25.0};
  double voltage_cmd_{12.0};
  double current_cmd_{0.5};
  double range_cmd_{0.5};

  // Simulation mode flag
  bool sim_mode_{true};

  // ROS node for subscriptions
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;

  // Callbacks
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg);
  void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
};

}  // namespace r2c_tutorial

#endif  // R2C_TUTORIAL__SENSOR_HARDWARE_INTERFACE_HPP_
