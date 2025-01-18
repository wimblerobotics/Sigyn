#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
// #include <sigyn_behavior_trees/say_something.hpp>
#include <sigyn_behavior_trees/SS.hpp>
#include <thread>

// class SaySomething : public BT::ActionNodeBase {
//  public:
//   SaySomething(const std::string& name) : ActionNodeBase(name, {}) {}

//   BT::NodeStatus tick() override {
//     std::cout << "SaySomething::tick()" << std::endl;
//     return BT::NodeStatus::SUCCESS;
//   }
//   static BT::PortsList providedBasicPorts() {
//     return BT::PortsList({
//         BT::InputPort<std::string>("message", "Message to log"),
//     });
//   }
//   // static BT::PortsList providedPorts() { return providedBasicPorts({}); }
//   void halt() override {}
// };

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("bt_test1");
  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<sigyn_behavior_trees::SS>("SS");

  auto tree = factory.createTreeFromFile(xml_path);
  tree.tickWhileRunning();
  return 0;

  return 0;
}