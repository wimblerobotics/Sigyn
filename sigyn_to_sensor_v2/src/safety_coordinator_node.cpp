/**
 * @file safety_coordinator_node.cpp
 * @brief Safety coordinator node main for TeensyV2 system
 * 
 * Standalone node for safety coordination across TeensyV2 boards.
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/safety_coordinator.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_to_sensor_v2::SafetyCoordinator>();
  
  RCLCPP_INFO(node->get_logger(), "Starting TeensyV2 Safety Coordinator Node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
