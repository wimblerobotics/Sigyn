#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <cmath>
#include <limits>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to check door state using laser scan
 */
class CheckDoorState : public BT::SyncActionNode {
public:
  CheckDoorState(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts() {
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

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
  std::chrono::steady_clock::time_point last_scan_time_;
  
  std::string door_name_;
  double expected_angle_;
  double distance_tolerance_;
  double timeout_seconds_;
  
  std::mutex scan_mutex_;
  
  double GetLaserReadingAtAngle(double angle);
};

}  // namespace sigyn_house_patroller
