#include "r2c_tutorial/sensor_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r2c_tutorial
{

hardware_interface::CallbackReturn SensorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize sensor values from parameters
  temperature_ = std::stod(info_.sensors[0].parameters.at("initial_value"));
  voltage_ = std::stod(info_.sensors[1].parameters.at("initial_value"));
  current_ = std::stod(info_.sensors[2].parameters.at("initial_value"));
  range_ = std::stod(info_.sensors[3].parameters.at("initial_value"));

  // Initialize command values to match initial states
  temperature_cmd_ = temperature_;
  voltage_cmd_ = voltage_;
  current_cmd_ = current_;
  range_cmd_ = range_;

  // Check for sim_mode parameter
  if (info_.hardware_parameters.find("sim_mode") != info_.hardware_parameters.end()) {
    sim_mode_ = (info_.hardware_parameters.at("sim_mode") == "true");
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterface"),
    "Initialized sensor hardware interface (sim_mode=%s) with:",
    sim_mode_ ? "true" : "false");
  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterface"),
    "  Temperature: %.1f °C", temperature_);
  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterface"),
    "  Voltage: %.2f V", voltage_);
  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterface"),
    "  Current: %.3f A", current_);
  RCLCPP_INFO(
    rclcpp::get_logger("SensorHardwareInterface"),
    "  Range: %.3f m", range_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> 
SensorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export sensor state interfaces
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &temperature_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &voltage_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[2].name, info_.sensors[2].state_interfaces[0].name, &current_));
  
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      info_.sensors[3].name, info_.sensors[3].state_interfaces[0].name, &range_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> 
SensorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Only export command interfaces in simulation mode
  if (sim_mode_) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.sensors[0].name, "temperature", &temperature_cmd_));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.sensors[1].name, "voltage", &voltage_cmd_));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.sensors[2].name, "current", &current_cmd_));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.sensors[3].name, "range", &range_cmd_));

    RCLCPP_INFO(
      rclcpp::get_logger("SensorHardwareInterface"),
      "Exported %zu command interfaces for simulation mode", command_interfaces.size());
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("SensorHardwareInterface"),
      "No command interfaces exported - running in real hardware mode");
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn SensorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorHardwareInterface"), "Activating sensor hardware interface");

  // Create ROS node for subscriptions
  node_ = std::make_shared<rclcpp::Node>("sensor_hardware_interface_node");
  
  // Subscribe to sensor topics
  // These topics come from sigyn_to_sensor_v2 for real hardware
  // or from simulation for Gazebo
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/sigyn/teensy_bridge/battery_state", 10,
    std::bind(&SensorHardwareInterface::battery_callback, this, std::placeholders::_1));
  
  temp_sub_ = node_->create_subscription<sensor_msgs::msg::Temperature>(
    "/sensors/temperature", 10,
    std::bind(&SensorHardwareInterface::temperature_callback, this, std::placeholders::_1));
  
  range_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
    "/sensors/range", 10,
    std::bind(&SensorHardwareInterface::range_callback, this, std::placeholders::_1));

  // Start executor in separate thread
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  RCLCPP_INFO(rclcpp::get_logger("SensorHardwareInterface"), 
    "Subscribed to sensor topics - waiting for data...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SensorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SensorHardwareInterface"), "Deactivating sensor hardware interface");

  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }

  battery_sub_.reset();
  temp_sub_.reset();
  range_sub_.reset();
  node_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SensorHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Sensor values are updated via callbacks
  // This function just needs to return SUCCESS
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SensorHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // In simulation mode, apply commanded values to sensor states
  // This allows us to inject test scenarios (high temp, low voltage, etc.)
  if (sim_mode_) {
    temperature_ = temperature_cmd_;
    voltage_ = voltage_cmd_;
    current_ = current_cmd_;
    range_ = range_cmd_;
  }
  // In real hardware mode, sensor values come from topic callbacks only
  
  return hardware_interface::return_type::OK;
}

void SensorHardwareInterface::battery_callback(
  const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  // Only update from topics in real hardware mode
  // In sim mode, values come from command interfaces
  if (!sim_mode_) {
    voltage_ = msg->voltage;
    current_ = msg->current;
    // Note: BatteryState also has temperature, but we use separate temperature sensor
  }
}

void SensorHardwareInterface::temperature_callback(
  const sensor_msgs::msg::Temperature::SharedPtr msg)
{
  if (!sim_mode_) {
    temperature_ = msg->temperature;
  }
}

void SensorHardwareInterface::range_callback(
  const sensor_msgs::msg::Range::SharedPtr msg)
{
  if (!sim_mode_) {
    range_ = msg->range;
  }
}

}  // namespace r2c_tutorial

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  r2c_tutorial::SensorHardwareInterface, hardware_interface::SystemInterface)
