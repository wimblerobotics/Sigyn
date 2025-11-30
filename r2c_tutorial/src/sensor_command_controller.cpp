#include "r2c_tutorial/sensor_command_controller.hpp"

#include <memory>
#include <string>
#include <vector>

namespace r2c_tutorial
{

SensorCommandController::SensorCommandController()
: controller_interface::ControllerInterface(),
  temperature_sensor_name_("temperature_sensor"),
  battery_sensor_name_("battery_sensor"),
  range_sensor_name_("range_sensor")
{
}

controller_interface::CallbackReturn SensorCommandController::on_init()
{
  try
  {
    // Declare parameters
    auto_declare<std::string>("temperature_sensor_name", temperature_sensor_name_);
    auto_declare<std::string>("battery_sensor_name", battery_sensor_name_);
    auto_declare<std::string>("range_sensor_name", range_sensor_name_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Exception thrown during init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SensorCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  temperature_sensor_name_ = get_node()->get_parameter("temperature_sensor_name").as_string();
  battery_sensor_name_ = get_node()->get_parameter("battery_sensor_name").as_string();
  range_sensor_name_ = get_node()->get_parameter("range_sensor_name").as_string();

  // Create subscribers for command topics
  temperature_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/temperature_cmd", 10,
    std::bind(&SensorCommandController::temperature_cmd_callback, this, std::placeholders::_1));
  
  voltage_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/voltage_cmd", 10,
    std::bind(&SensorCommandController::voltage_cmd_callback, this, std::placeholders::_1));
  
  current_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/current_cmd", 10,
    std::bind(&SensorCommandController::current_cmd_callback, this, std::placeholders::_1));
  
  range_cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
    "~/range_cmd", 10,
    std::bind(&SensorCommandController::range_cmd_callback, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Sensor command controller configured");
  RCLCPP_WARN(get_node()->get_logger(), "⚠️  SIMULATION MODE ONLY - Writing sensor commands for testing");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SensorCommandController::state_interface_configuration() const
{
  // This controller doesn't read states, only writes commands
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
SensorCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request command interfaces for all sensors
  config.names.push_back(temperature_sensor_name_ + "/temperature");
  config.names.push_back(battery_sensor_name_ + "/voltage");
  config.names.push_back(battery_sensor_name_ + "/current");
  config.names.push_back(range_sensor_name_ + "/range");

  return config;
}

controller_interface::CallbackReturn SensorCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the expected command interfaces
  if (command_interfaces_.size() != 4)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 4 command interfaces, got %zu. Are you in simulation mode?",
      command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Sensor command controller activated");
  RCLCPP_INFO(get_node()->get_logger(), "Subscribe to these topics to inject test values:");
  RCLCPP_INFO(get_node()->get_logger(), "  ~/temperature_cmd (Float64) - Temperature in °C");
  RCLCPP_INFO(get_node()->get_logger(), "  ~/voltage_cmd (Float64) - Voltage in V");
  RCLCPP_INFO(get_node()->get_logger(), "  ~/current_cmd (Float64) - Current in A");
  RCLCPP_INFO(get_node()->get_logger(), "  ~/range_cmd (Float64) - Range in m");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SensorCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Sensor command controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SensorCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commanded values to command interfaces
  // Note: The order of command_interfaces_ matches the order we requested them
  
  if (temperature_cmd_received_)
  {
    command_interfaces_[0].set_value(temperature_cmd_);
    temperature_cmd_received_ = false;
  }
  
  if (voltage_cmd_received_)
  {
    command_interfaces_[1].set_value(voltage_cmd_);
    voltage_cmd_received_ = false;
  }
  
  if (current_cmd_received_)
  {
    command_interfaces_[2].set_value(current_cmd_);
    current_cmd_received_ = false;
  }
  
  if (range_cmd_received_)
  {
    command_interfaces_[3].set_value(range_cmd_);
    range_cmd_received_ = false;
  }

  return controller_interface::return_type::OK;
}

void SensorCommandController::temperature_cmd_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  temperature_cmd_ = msg->data;
  temperature_cmd_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Setting temperature to %.1f °C", msg->data);
}

void SensorCommandController::voltage_cmd_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  voltage_cmd_ = msg->data;
  voltage_cmd_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Setting voltage to %.2f V", msg->data);
}

void SensorCommandController::current_cmd_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  current_cmd_ = msg->data;
  current_cmd_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Setting current to %.3f A", msg->data);
}

void SensorCommandController::range_cmd_callback(
  const std_msgs::msg::Float64::SharedPtr msg)
{
  range_cmd_ = msg->data;
  range_cmd_received_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Setting range to %.3f m", msg->data);
}

}  // namespace r2c_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  r2c_tutorial::SensorCommandController,
  controller_interface::ControllerInterface)
