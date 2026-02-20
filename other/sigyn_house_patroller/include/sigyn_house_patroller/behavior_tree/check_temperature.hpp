#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to check temperature levels
 */
class CheckTemperature : public BT::SyncActionNode {
public:
  CheckTemperature(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node for subscriptions"),
      BT::InputPort<double>("min_temperature", 18.0, "Minimum acceptable temperature (°C)"),
      BT::InputPort<double>("max_temperature", 28.0, "Maximum acceptable temperature (°C)"),
      BT::InputPort<double>("timeout_seconds", 5.0, "Timeout for temperature data"),
      BT::OutputPort<double>("current_temperature", "Current temperature in Celsius"),
      BT::OutputPort<bool>("temperature_ok", "True if temperature is within range"),
      BT::OutputPort<bool>("temperature_high", "True if temperature is too high"),
      BT::OutputPort<bool>("temperature_low", "True if temperature is too low")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  
  sensor_msgs::msg::Temperature::SharedPtr last_temp_msg_;
  std::chrono::steady_clock::time_point last_temp_time_;
  
  double min_temperature_;
  double max_temperature_;
  double timeout_seconds_;
  
  std::mutex temp_mutex_;
};

}  // namespace sigyn_house_patroller
