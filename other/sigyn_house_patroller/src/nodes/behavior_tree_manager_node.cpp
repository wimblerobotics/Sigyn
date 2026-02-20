#include "sigyn_house_patroller/behavior_tree/behavior_tree_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::BehaviorTreeManager>();
  
  RCLCPP_INFO(rclcpp::get_logger("behavior_tree_manager"), "Starting behavior tree manager node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
