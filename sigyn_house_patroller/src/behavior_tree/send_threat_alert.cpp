#include "sigyn_house_patroller/behavior_tree/send_threat_alert.hpp"
#include <rclcpp/rclcpp.hpp>
#include "sigyn_house_patroller/msg/threat_alert.hpp"

namespace sigyn_house_patroller {

SendThreatAlert::SendThreatAlert(const std::string& xml_tag_name, 
                                 const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("SendThreatAlert requires a 'node' input");
  }
  
  // Create threat alert publisher
  threat_pub_ = node_->create_publisher<msg::ThreatAlert>(
    "/sigyn_house_patroller/threat_alerts", rclcpp::QoS(10).reliable());
  
  // Get parameters
  getInput("threat_type", threat_type_);
  getInput("threat_description", threat_description_);
  getInput("threat_severity", threat_severity_);
  getInput("threat_confidence", threat_confidence_);
  
  RCLCPP_INFO(node_->get_logger(), "SendThreatAlert initialized");
}

BT::NodeStatus SendThreatAlert::tick() {
  // Create threat alert message
  msg::ThreatAlert alert;
  alert.header.stamp = node_->now();
  alert.header.frame_id = "base_link";
  
  // Generate unique threat ID
  static int threat_counter = 0;
  alert.threat_id = "bt_alert_" + std::to_string(++threat_counter);
  
  alert.threat_type = threat_type_;
  alert.description = threat_description_;
  alert.confidence = threat_confidence_;
  alert.timestamp = node_->now();
  
  // Map severity
  if (threat_severity_ == "CRITICAL") {
    alert.severity = msg::ThreatAlert::SEVERITY_CRITICAL;
  } else if (threat_severity_ == "HIGH") {
    alert.severity = msg::ThreatAlert::SEVERITY_HIGH;
  } else if (threat_severity_ == "WARNING") {
    alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
  } else {
    alert.severity = msg::ThreatAlert::SEVERITY_INFO;
  }
  
  // Get optional location information
  geometry_msgs::msg::Point location;
  if (getInput("threat_location", location)) {
    alert.location = location;
  }
  
  // Get optional sensor data
  std::string sensor_data = "{}";
  getInput("sensor_data", sensor_data);
  alert.sensor_data = sensor_data;
  
  // Publish the alert
  threat_pub_->publish(alert);
  
  // Set output values
  setOutput("alert_sent", true);
  setOutput("alert_id", alert.threat_id);
  setOutput("alert_timestamp", alert.timestamp);
  
  RCLCPP_INFO(node_->get_logger(), 
              "SendThreatAlert: Sent %s alert - %s (confidence: %.2f)", 
              threat_severity_.c_str(), threat_description_.c_str(), threat_confidence_);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::SendThreatAlert>("SendThreatAlert");
}
