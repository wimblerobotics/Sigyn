#include "sigyn_behavior_trees/say_something.hpp"
namespace sigyn_behavior_trees {

SaySomething::SaySomething(const std::string& xml_tag_name, const std::string& action_name,
                           const BT::NodeConfiguration& conf)
    : BtActionNode<sigyn_behavior_trees::action::SaySomething>(xml_tag_name, action_name, conf) {}

SaySomething::~SaySomething() {}

void SaySomething::on_tick() {
  BT::Expected<std::string> msg = getInput<std::string>("message");
  // Check if expected is valid. If not, throw its error
  if (!msg) {
    throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
  goal_.message = msg.value();
  goal_.pose = pose.value();
  increment_recovery_count();
}

// BT::NodeStatus SaySomething::on_success() { return BT::NodeStatus::SUCCESS; }

// BT::NodeStatus SaySomething::on_aborted() { return BT::NodeStatus::SUCCESS; }

// BT::NodeStatus SaySomething::on_cancelled() { return BT::NodeStatus::SUCCESS; }

// void SaySomething::halt() {
//   //  BTActionNode::halt();
// }

}  // namespace sigyn_behavior_trees

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<sigyn_behavior_trees::SaySomething>(name, "say_something", config);
  };

  factory.registerBuilder<sigyn_behavior_trees::SaySomething>("SaySomething", builder);
}