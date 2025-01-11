#include "say_something.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <rclcpp/rclcpp.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#undef FUTURE_WAIT_BLOCK

SaySomething::SaySomething(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
  node_ = rclcpp::Node::make_shared("SaySomething");
}

BT::PortsList SaySomething::providedPorts() {
  // Optionally, a port can have a human readable description
  return {BT::InputPort<std::string>("message", "What to say")};
}

BT::NodeStatus SaySomething::tick() {
  std::string message;
  if (!getInput<std::string>("message", message)) {
    throw BT::RuntimeError("missing required input [message]");
  }

  RCLCPP_INFO(node_->get_logger(), "Saying %s", message.c_str());

  return BT::NodeStatus::SUCCESS;
}
