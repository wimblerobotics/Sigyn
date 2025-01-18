// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

// #include <cstdio>
#include <chrono>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
// #include "move_a_short_distance_ahead.cpp"
#include <memory>
#include <string>

#include "sigyn_behavior_trees/say_something.hpp"
#include "SaySomethingActionServer.cpp"
#include <atomic>
#include <thread>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr g_node = rclcpp::Node::make_shared("bt_test1");

  std::string xml_path;
  g_node->declare_parameter<std::string>("xml_path", "foo");
  g_node->get_parameter("xml_path", xml_path);

      std::shared_ptr<BT::BehaviorTreeFactory> factory = std::make_shared<BT::BehaviorTreeFactory>();
      static BT::NodeConfiguration* config = new BT::NodeConfiguration();
      config->blackboard = BT::Blackboard::create();
      config->blackboard->set("bt_loop_duration", std::chrono::milliseconds(10));
      config->blackboard->set("node", g_node);
      config->blackboard->set<std::chrono::milliseconds>("server_timeout",
                                                          std::chrono::milliseconds(20));
      config->blackboard->set("wait_for_service_timeout", std::chrono::milliseconds(10'000));

      BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
        return std::make_unique<sigyn_behavior_trees::SaySomething>(name, "SaySomething", config);
      };
      factory->registerBuilder<sigyn_behavior_trees::SaySomething>("SaySomething", builder);

  printf("bt_test1 starting, xml_path: %s\n", xml_path.c_str());
  auto tree = factory->createTreeFromFile(xml_path, config->blackboard);
  for (auto& subtree : tree.subtrees) {
    auto& blackboard = subtree->blackboard;
    blackboard->set("node", g_node);
  }

    BT::StdCoutLogger logger(tree);

  std::atomic<bool> running(true);

  std::thread action_server([&]() {
    while (running) 
    {
      tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  std::cin.get();
  running = false;
  action_server.join();

  // // BT::PublisherZMQ publisher_zmq(tree);
  // BT::StdCoutLogger logger_cout(tree);
  // BT::printTreeRecursively(tree.rootNode());


  // // // ClosePointSubscriber::SharedPtr cps = ClosePointSubscriber::singleton();
  // // std::shared_ptr<ClosePointSubscriber> cps = ClosePointSubscriber::singleton();

  // // RCLCPP_INFO(g_node->get_logger(), "[main] Waiting for map->lidar_link transform to be
  // // available"); std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
  // // std::unique_ptr<tf2_ros::Buffer> tf_buffer =
  // //     std::make_unique<tf2_ros::Buffer>(g_node->get_clock());
  // // transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // // auto timeout_ms_ = std::chrono::milliseconds(2);
  // // while (!tf_buffer->canTransform("map", "lidar_link", tf2::TimePointZero) && rclcpp::ok()) {
  // //   std::this_thread::sleep_for(timeout_ms_);
  // // }

  // // RCLCPP_INFO(g_node->get_logger(), "[main] transform is available, starting behavior tree");

  // {
  //   auto action_server_ = std::make_shared<sigyn_behavior_trees::SaySomethingActionServer>();
  //   BT::NodeStatus status = BT::NodeStatus::RUNNING;
  //   rclcpp::WallRate loopRate(15);
  //   while (rclcpp::ok() && (status == BT::NodeStatus::RUNNING)) {
  //     rclcpp::spin_some(action_server_);
  //     // rclcpp::spin_some(cps);
  //     status = tree.tickOnce();
  //     loopRate.sleep();
  //   }
  // }

  printf("bt_test1 finished\n");
  return 0;
}