#ifndef R2C_TUTORIAL__SENSOR_COMMAND_CONTROLLER_HPP_
#define R2C_TUTORIAL__SENSOR_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace r2c_tutorial
{

/**
 * @brief Controller that writes commands to sensor command interfaces (simulation only)
 * 
 * This controller subscribes to command topics and writes the values to
 * sensor command interfaces. This allows testing safety systems in simulation
 * by injecting fault conditions.
 * 
 * Command Interfaces (Write):
 * - temperature_sensor/temperature
 * - voltage_sensor/voltage
 * - current_sensor/current
 * - vl53l0x_sensor/range
 * 
 * Subscribed Topics:
 * - ~/temperature_cmd (std_msgs/msg/Float64)
 * - ~/voltage_cmd (std_msgs/msg/Float64)
 * - ~/current_cmd (std_msgs/msg/Float64)
 * - ~/range_cmd (std_msgs/msg/Float64)
 * 
 * Note: This controller is only active in simulation mode where hardware
 * interfaces expose command interfaces. In real hardware mode, sensors are
 * read-only and this controller should not be loaded.
 */
class SensorCommandController : public controller_interface::ControllerInterface
{
public:
  SensorCommandController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Subscribers for command topics
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr temperature_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr voltage_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr range_cmd_sub_;

  // Command values to write to hardware
  double temperature_cmd_{25.0};
  double voltage_cmd_{12.0};
  double current_cmd_{0.5};
  double range_cmd_{0.5};

  // Flags to track if new commands arrived
  bool temperature_cmd_received_{false};
  bool voltage_cmd_received_{false};
  bool current_cmd_received_{false};
  bool range_cmd_received_{false};

  // Sensor interface names
  std::string temperature_sensor_name_;
  std::string battery_sensor_name_;     // Combined voltage + current
  std::string range_sensor_name_;

  // Callbacks
  void temperature_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void voltage_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void current_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void range_cmd_callback(const std_msgs::msg::Float64::SharedPtr msg);
};

}  // namespace r2c_tutorial

#endif  // R2C_TUTORIAL__SENSOR_COMMAND_CONTROLLER_HPP_
