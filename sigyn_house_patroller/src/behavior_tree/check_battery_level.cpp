#include "sigyn_house_patroller/behavior_tree/check_battery_level.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace sigyn_house_patroller {

CheckBatteryLevel::CheckBatteryLevel(const std::string& xml_tag_name, 
                                     const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("CheckBatteryLevel requires a 'node' input");
  }
  
  // Create battery state subscriber
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery_state", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(battery_mutex_);
      last_battery_msg_ = msg;
      last_battery_time_ = std::chrono::steady_clock::now();
    });
  
  // Get parameters
  getInput("min_battery_level", min_battery_level_);
  getInput("timeout_seconds", timeout_seconds_);
  
  RCLCPP_INFO(node_->get_logger(), "CheckBatteryLevel initialized - min level: %.2f", 
              min_battery_level_);
}

BT::NodeStatus CheckBatteryLevel::tick() {
  std::lock_guard<std::mutex> lock(battery_mutex_);
  
  // Check if we have recent battery data
  auto now = std::chrono::steady_clock::now();
  if (!last_battery_msg_ || 
      std::chrono::duration_cast<std::chrono::seconds>(now - last_battery_time_).count() > timeout_seconds_) {
    RCLCPP_WARN(node_->get_logger(), "No recent battery data available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Check battery level
  float battery_percentage = last_battery_msg_->percentage;
  
  // Set output values
  setOutput("battery_level", battery_percentage);
  setOutput("battery_voltage", last_battery_msg_->voltage);
  setOutput("is_charging", last_battery_msg_->power_supply_status == 
            sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
  
  // Check if battery level is sufficient
  if (battery_percentage >= min_battery_level_) {
    RCLCPP_DEBUG(node_->get_logger(), "Battery level OK: %.1f%%", battery_percentage);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Battery level low: %.1f%% (min: %.1f%%)", 
                battery_percentage, min_battery_level_);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::CheckBatteryLevel>("CheckBatteryLevel");
}
