/**
 * @file performance_monitor_node.cpp
 * @brief Performance monitor node main for TeensyV2 system
 * 
 * Standalone node for performance monitoring from TeensyV2 boards.
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#include "sigyn_to_sensor_v2/performance_monitor.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_to_sensor_v2::PerformanceMonitor>();
  
  RCLCPP_INFO(node->get_logger(), "Starting TeensyV2 Performance Monitor Node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
