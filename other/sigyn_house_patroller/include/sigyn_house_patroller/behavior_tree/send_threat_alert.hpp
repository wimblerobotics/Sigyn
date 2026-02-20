#pragma once

#include <memory>
#include <string>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include "sigyn_house_patroller/msg/threat_alert.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to send threat alerts
 */
class SendThreatAlert : public BT::SyncActionNode {
public:
  SendThreatAlert(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node for publishing"),
      BT::InputPort<std::string>("threat_type", "Type of threat"),
      BT::InputPort<std::string>("threat_description", "Description of the threat"),
      BT::InputPort<std::string>("threat_severity", "Severity: INFO, WARNING, HIGH, CRITICAL"),
      BT::InputPort<double>("threat_confidence", 0.8, "Confidence in threat detection"),
      BT::InputPort<geometry_msgs::msg::Point>("threat_location", "Location of threat"),
      BT::InputPort<std::string>("sensor_data", "{}", "JSON sensor data"),
      BT::OutputPort<bool>("alert_sent", "True if alert was sent successfully"),
      BT::OutputPort<std::string>("alert_id", "ID of the sent alert"),
      BT::OutputPort<builtin_interfaces::msg::Time>("alert_timestamp", "Timestamp of alert")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_pub_;
  
  std::string threat_type_;
  std::string threat_description_;
  std::string threat_severity_;
  double threat_confidence_;
};

}  // namespace sigyn_house_patroller
