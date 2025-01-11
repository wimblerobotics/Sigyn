#include "nav2_behavior_tree/plugins/action/wait_action.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rcutils/logging_macros.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "pluginlib/class_loader.hpp"
#include "say_something.hpp"
// #include "wait.hpp"
// #include <nav2_behaviors/plugins/wait.hpp>

#include "nav2_core/behavior.hpp"

class WaitBTNode : public BT::SyncActionNode {
 public:
  WaitBTNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("wait_duration", 5.0, "Seconds to wait")};
  }

  BT::NodeStatus tick() override {
    double duration;
    getInput<double>("wait_duration", duration);
    nav2_behaviors::Wait wait_behavior;
    // Invoke your wait logic here...
    // wait_behavior.wait(duration);
    return BT::NodeStatus::SUCCESS;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("bt1_node");

  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);
  RCUTILS_LOG_INFO("[bt1] xml_path: %s", xml_path.c_str());

  pluginlib::ClassLoader<nav2_core::Behavior> plugin_loader_("nav2_core", "nav2_core::Behavior");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  // factory.registerNodeType<WaitBTNode>("Wait");
    factory.registerNodeType<nav2_behavior_tree::WaitAction>("Wait");


  // auto wait_behavior = plugin_loader_.createSharedInstance("nav2_behaviors::Wait");
  // factory.registerNodeType<nav2_behaviors::Wait>("Wait");

  // factory.registerNodeType<BT::ActionNode>(
  //     "SaySomething", [](const std::string& name, const BT::NodeConfiguration& config) {
  //       return std::make_unique<BT::ActionNode>(name, config);
  //     });

  auto tree = factory.createTreeFromFile(xml_path);

  BT::PublisherZMQ publisher_zmq(tree);
  BT::StdCoutLogger logger_cout(tree);
  BT::printTreeRecursively(tree.rootNode());

  {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::WallRate loopRate(15);
    while (rclcpp::ok() && (status == BT::NodeStatus::RUNNING)) {
      rclcpp::spin_some(g_node);
      status = tree.tickRoot();
      loopRate.sleep();
    }
  }

  rclcpp::shutdown();
  return 0;
}