#include "sigyn_house_patroller/behavior_tree/check_door_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sigyn_house_patroller {

CheckDoorState::CheckDoorState(const std::string& xml_tag_name, 
                               const BT::NodeConfiguration& conf)
    : BT::SyncActionNode(xml_tag_name, conf) {
  
  // Get node from blackboard
  getInput("node", node_);
  if (!node_) {
    throw BT::RuntimeError("CheckDoorState requires a 'node' input");
  }
  
  // Create laser scan subscriber
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::QoS(10).reliable(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      last_scan_msg_ = msg;
      last_scan_time_ = std::chrono::steady_clock::now();
    });
  
  // Get parameters
  getInput("door_name", door_name_);
  getInput("expected_angle", expected_angle_);
  getInput("distance_tolerance", distance_tolerance_);
  getInput("timeout_seconds", timeout_seconds_);
  
  RCLCPP_INFO(node_->get_logger(), "CheckDoorState initialized for door: %s", 
              door_name_.c_str());
}

BT::NodeStatus CheckDoorState::tick() {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  
  // Check if we have recent scan data
  auto now = std::chrono::steady_clock::now();
  if (!last_scan_msg_ || 
      std::chrono::duration_cast<std::chrono::seconds>(now - last_scan_time_).count() > timeout_seconds_) {
    RCLCPP_WARN(node_->get_logger(), "No recent laser scan data available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Get laser reading at expected door angle
  double measured_distance = GetLaserReadingAtAngle(expected_angle_);
  
  if (std::isnan(measured_distance) || std::isinf(measured_distance)) {
    RCLCPP_WARN(node_->get_logger(), "Invalid laser reading for door %s", door_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Determine door state based on distance
  bool door_is_open = measured_distance > (1.0 + distance_tolerance_);  // Simple heuristic
  
  // Set output values
  setOutput("door_distance", measured_distance);
  setOutput("door_open", door_is_open);
  setOutput("door_closed", !door_is_open);
  
  RCLCPP_DEBUG(node_->get_logger(), "Door %s: distance=%.2fm, state=%s", 
               door_name_.c_str(), measured_distance, 
               door_is_open ? "OPEN" : "CLOSED");
  
  return BT::NodeStatus::SUCCESS;
}

double CheckDoorState::GetLaserReadingAtAngle(double angle) {
  if (!last_scan_msg_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  
  // Normalize angle
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  
  // Check if angle is within scan range
  if (angle < last_scan_msg_->angle_min || angle > last_scan_msg_->angle_max) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  
  // Calculate index
  int index = static_cast<int>((angle - last_scan_msg_->angle_min) / 
                               last_scan_msg_->angle_increment);
  
  if (index < 0 || index >= static_cast<int>(last_scan_msg_->ranges.size())) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  
  float range = last_scan_msg_->ranges[index];
  
  // Validate range
  if (range < last_scan_msg_->range_min || range > last_scan_msg_->range_max) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  
  return static_cast<double>(range);
}

BT::PortsList CheckDoorState::providedPorts() {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node for subscriptions"),
      BT::InputPort<std::string>("door_name", "Name of the door to check"),
      BT::InputPort<double>("expected_angle", 0.0, "Expected angle to door in radians"),
      BT::InputPort<double>("distance_tolerance", 0.1, "Distance tolerance for door detection"),
      BT::InputPort<double>("timeout_seconds", 5.0, "Timeout for laser scan data"),
      BT::OutputPort<double>("door_distance", "Measured distance to door"),
      BT::OutputPort<bool>("door_open", "True if door is open"),
      BT::OutputPort<bool>("door_closed", "True if door is closed")
    };
}

}  // namespace sigyn_house_patroller

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<sigyn_house_patroller::CheckDoorState>("CheckDoorState");
}
