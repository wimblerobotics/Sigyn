#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to check battery level
 */
class CheckBatteryLevel : public BT::SyncActionNode {
public:
  CheckBatteryLevel(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  
  sensor_msgs::msg::BatteryState::SharedPtr last_battery_msg_;
  std::chrono::steady_clock::time_point last_battery_time_;
  
  double min_battery_level_;
  double timeout_seconds_;
  
  std::mutex battery_mutex_;
};

}  // namespace sigyn_house_patroller
