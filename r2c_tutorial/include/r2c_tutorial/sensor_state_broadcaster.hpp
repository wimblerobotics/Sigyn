#ifndef R2C_TUTORIAL__SENSOR_STATE_BROADCASTER_HPP_
#define R2C_TUTORIAL__SENSOR_STATE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace r2c_tutorial
{

/**
 * @brief Controller that broadcasts sensor states from hardware interfaces
 * 
 * This controller reads sensor state interfaces from the hardware and publishes
 * them as ROS messages. It acts as a bridge between ros2_control and standard
 * ROS topics.
 * 
 * State Interfaces (Read):
 * - temperature_sensor/temperature
 * - voltage_sensor/voltage
 * - current_sensor/current
 * - vl53l0x_sensor/range
 * 
 * Published Topics:
 * - ~/temperature (sensor_msgs/msg/Temperature)
 * - ~/battery (sensor_msgs/msg/BatteryState) - voltage + current
 * - ~/range (sensor_msgs/msg/Range)
 */
class SensorStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  SensorStateBroadcaster();

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
  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr range_pub_;

  // Sensor interface names
  std::string temperature_sensor_name_;
  std::string battery_sensor_name_;     // Combined voltage + current
  std::string range_sensor_name_;

  // Frame IDs for sensor messages
  std::string temperature_frame_id_;
  std::string battery_frame_id_;
  std::string range_frame_id_;

  // Publishing rate
  double publish_rate_;
  rclcpp::Time last_publish_time_;
};

}  // namespace r2c_tutorial

#endif  // R2C_TUTORIAL__SENSOR_STATE_BROADCASTER_HPP_
