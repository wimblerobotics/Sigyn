#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <chrono>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sigyn_house_patroller {

/**
 * @brief Behavior tree node to navigate to a waypoint
 */
class NavigateToWaypoint : public BT::StatefulActionNode {
public:
  NavigateToWaypoint(const std::string& xml_tag_name, const BT::NodeConfiguration& conf);
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node for action client"),
      BT::InputPort<std::string>("waypoint_name", "Name of the waypoint"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose", "Target pose to navigate to"),
      BT::InputPort<double>("timeout_seconds", 120.0, "Navigation timeout"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_pose", "Current robot pose"),
      BT::OutputPort<double>("distance_remaining", "Distance remaining to target"),
      BT::OutputPort<double>("elapsed_time", "Time elapsed since navigation started")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  
  std::string waypoint_name_;
  double timeout_seconds_;
  
  geometry_msgs::msg::PoseStamped current_pose_;
  double distance_remaining_;
  builtin_interfaces::msg::Duration estimated_time_remaining_;
  
  std::chrono::steady_clock::time_point start_time_;
  BT::NodeStatus navigation_result_;
  
  std::mutex nav_mutex_;
};

}  // namespace sigyn_house_patroller
