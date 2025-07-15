#include "sigyn_house_patroller/behavior_tree/check_temperature.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace sigyn_house_patroller {

CheckTemperature::CheckTemperature(const std::string& xml_tag_name, 
                                   const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("CheckTemperature requires a 'node' input");
  }
  
  // Create temperature subscriber
  temp_sub_ = node_->create_subscription<sensor_msgs::msg::Temperature>(
    "/temperature", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(temp_mutex_);
      last_temp_msg_ = msg;
      last_temp_time_ = std::chrono::steady_clock::now();
    });
  
  // Get parameters
  getInput("min_temperature", min_temperature_);
  getInput("max_temperature", max_temperature_);
  getInput("timeout_seconds", timeout_seconds_);
  
  RCLCPP_INFO(node_->get_logger(), "CheckTemperature initialized - range: %.1f to %.1f°C", 
              min_temperature_, max_temperature_);
}

BT::NodeStatus CheckTemperature::tick() {
  std::lock_guard<std::mutex> lock(temp_mutex_);
  
  // Check if we have recent temperature data
  auto now = std::chrono::steady_clock::now();
  if (!last_temp_msg_ || 
      std::chrono::duration_cast<std::chrono::seconds>(now - last_temp_time_).count() > timeout_seconds_) {
    RCLCPP_WARN(node_->get_logger(), "No recent temperature data available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Get temperature in Celsius
  double temp_celsius = last_temp_msg_->temperature;
  
  // Set output values
  setOutput("current_temperature", temp_celsius);
  setOutput("temperature_ok", temp_celsius >= min_temperature_ && temp_celsius <= max_temperature_);
  setOutput("temperature_high", temp_celsius > max_temperature_);
  setOutput("temperature_low", temp_celsius < min_temperature_);
  
  // Check if temperature is within acceptable range
  if (temp_celsius >= min_temperature_ && temp_celsius <= max_temperature_) {
    RCLCPP_DEBUG(node_->get_logger(), "Temperature OK: %.1f°C", temp_celsius);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Temperature out of range: %.1f°C (range: %.1f to %.1f°C)", 
                temp_celsius, min_temperature_, max_temperature_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::CheckTemperature>("CheckTemperature");
}
