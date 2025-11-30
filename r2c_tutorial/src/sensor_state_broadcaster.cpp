#include "r2c_tutorial/sensor_state_broadcaster.hpp"

#include <memory>
#include <string>
#include <vector>

namespace r2c_tutorial
{

SensorStateBroadcaster::SensorStateBroadcaster()
: controller_interface::ControllerInterface(),
  temperature_sensor_name_("temperature_sensor"),
  battery_sensor_name_("battery_sensor"),
  range_sensor_name_("range_sensor"),
  temperature_frame_id_("temperature_sensor"),
  battery_frame_id_("base_link"),
  range_frame_id_("temperature_sensor"),
  publish_rate_(10.0)
{
}

controller_interface::CallbackReturn SensorStateBroadcaster::on_init()
{
  try
  {
    // Declare parameters
    auto_declare<std::string>("temperature_sensor_name", temperature_sensor_name_);
    auto_declare<std::string>("battery_sensor_name", battery_sensor_name_);
    auto_declare<std::string>("range_sensor_name", range_sensor_name_);
    
    auto_declare<std::string>("temperature_frame_id", temperature_frame_id_);
    auto_declare<std::string>("battery_frame_id", battery_frame_id_);
    auto_declare<std::string>("range_frame_id", range_frame_id_);
    
    auto_declare<double>("publish_rate", publish_rate_);
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

controller_interface::CallbackReturn SensorStateBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  temperature_sensor_name_ = get_node()->get_parameter("temperature_sensor_name").as_string();
  battery_sensor_name_ = get_node()->get_parameter("battery_sensor_name").as_string();
  range_sensor_name_ = get_node()->get_parameter("range_sensor_name").as_string();
  
  temperature_frame_id_ = get_node()->get_parameter("temperature_frame_id").as_string();
  battery_frame_id_ = get_node()->get_parameter("battery_frame_id").as_string();
  range_frame_id_ = get_node()->get_parameter("range_frame_id").as_string();
  
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

  // Create publishers
  temperature_pub_ = get_node()->create_publisher<sensor_msgs::msg::Temperature>(
    "~/temperature", 10);
  
  battery_pub_ = get_node()->create_publisher<sensor_msgs::msg::BatteryState>(
    "~/battery", 10);
  
  range_pub_ = get_node()->create_publisher<sensor_msgs::msg::Range>(
    "~/range", 10);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured sensor state broadcaster with publish rate: %.1f Hz", publish_rate_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
SensorStateBroadcaster::command_interface_configuration() const
{
  // This controller doesn't command anything, only reads states
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
SensorStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request state interfaces for all sensors
  config.names.push_back(temperature_sensor_name_ + "/temperature");
  config.names.push_back(battery_sensor_name_ + "/voltage");
  config.names.push_back(battery_sensor_name_ + "/current");
  config.names.push_back(range_sensor_name_ + "/range");

  return config;
}

controller_interface::CallbackReturn SensorStateBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify we have the expected state interfaces
  if (state_interfaces_.size() != 4)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected 4 state interfaces, got %zu", state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  last_publish_time_ = get_node()->now();

  RCLCPP_INFO(get_node()->get_logger(), "Sensor state broadcaster activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SensorStateBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Sensor state broadcaster deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SensorStateBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Throttle publishing to the configured rate
  if ((time - last_publish_time_).seconds() < (1.0 / publish_rate_))
  {
    return controller_interface::return_type::OK;
  }
  last_publish_time_ = time;

  // Read sensor values from state interfaces
  // Note: The order of state_interfaces_ matches the order we requested them
  double temperature = state_interfaces_[0].get_value();
  double voltage = state_interfaces_[1].get_value();
  double current = state_interfaces_[2].get_value();
  double range = state_interfaces_[3].get_value();

  // Publish temperature
  sensor_msgs::msg::Temperature temp_msg;
  temp_msg.header.stamp = time;
  temp_msg.header.frame_id = temperature_frame_id_;
  temp_msg.temperature = temperature;
  temp_msg.variance = 0.0;  // Could be configurable
  temperature_pub_->publish(temp_msg);

  // Publish battery state (voltage + current)
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = time;
  battery_msg.header.frame_id = battery_frame_id_;
  battery_msg.voltage = voltage;
  battery_msg.current = current;
  battery_msg.charge = std::numeric_limits<float>::quiet_NaN();  // Unknown
  battery_msg.capacity = std::numeric_limits<float>::quiet_NaN();  // Unknown
  battery_msg.design_capacity = std::numeric_limits<float>::quiet_NaN();  // Unknown
  battery_msg.percentage = std::numeric_limits<float>::quiet_NaN();  // Unknown
  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_msg.present = true;
  battery_pub_->publish(battery_msg);

  // Publish range
  sensor_msgs::msg::Range range_msg;
  range_msg.header.stamp = time;
  range_msg.header.frame_id = range_frame_id_;
  range_msg.radiation_type = sensor_msgs::msg::Range::INFRARED;
  range_msg.field_of_view = 0.436;  // ~25 degrees for VL53L0X
  range_msg.min_range = 0.03;  // 3cm
  range_msg.max_range = 2.0;   // 2m
  range_msg.range = range;
  range_pub_->publish(range_msg);

  return controller_interface::return_type::OK;
}

}  // namespace r2c_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  r2c_tutorial::SensorStateBroadcaster,
  controller_interface::ControllerInterface)
