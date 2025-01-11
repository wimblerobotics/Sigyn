#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"

class SaySomething : public BT::SyncActionNode {
 public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

 private:
  rclcpp::Node::SharedPtr node_;
};