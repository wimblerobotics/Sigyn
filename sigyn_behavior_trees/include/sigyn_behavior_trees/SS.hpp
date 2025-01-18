#pragma once
#include <behaviortree_cpp/action_node.h>

namespace sigyn_behavior_trees {
class SS : public BT::SyncActionNode {
 public:
  // If your Node has ports, you must use this constructor signature
  SS(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts() {
    // This action has a single input port called "message"
    return {BT::InputPort<std::string>("message")};
  }

  // Override the virtual function tick()
  BT::NodeStatus tick() override {
    BT::Expected<std::string> msg = getInput<std::string>("message");
    // Check if expected is valid. If not, throw its error
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace BT