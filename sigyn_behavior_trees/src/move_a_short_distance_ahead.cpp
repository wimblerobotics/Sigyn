#include "sigyn_behavior_trees/move_a_short_distance_ahead.hpp"

// #include <cmath>
// #include <memory>
// #include <string>

namespace sigyn_behavior_trees {

MoveAShortDistanceAhead::MoveAShortDistanceAhead(const std::string& xml_tag_name,
                                                 const std::string& action_name,
                                                 const BT::NodeConfiguration& conf)
    : BtActionNode<sigyn_behavior_trees::action::MoveAShortDistanceAhead>(xml_tag_name, action_name,
                                                                          conf) {}

MoveAShortDistanceAhead::~MoveAShortDistanceAhead() {}

void MoveAShortDistanceAhead::initialize() {
  double distance;
  getInput("distance", distance);
  RCLCPP_INFO(node_->get_logger(), "MoveAShortDistanceAhead: distance %f", distance);
  if (distance <= 0) {
    RCLCPP_WARN(node_->get_logger(),
                "MoveAShortDistanceAhead distance is negative or zero "
                "(%f). Setting to zero.",
                distance);
    distance = 0.0;
  }

  goal_.distance = distance;
}

void MoveAShortDistanceAhead::on_tick() {
  if (!BT::isStatusActive(status())) {
    initialize();
  }

  printf("MoveAShortDistanceAhead::on_tick()\n");
  increment_recovery_count();
}

}  // namespace sigyn_behavior_trees

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<sigyn_behavior_trees::MoveAShortDistanceAhead>(name, "move_a_short_distance_ahead", config);
  };

  factory.registerBuilder<sigyn_behavior_trees::MoveAShortDistanceAhead>("MoveAShortDistanceAhead", builder);
}