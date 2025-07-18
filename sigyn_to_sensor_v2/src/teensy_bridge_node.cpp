/**
 * @file teensy_bridge_node.cpp
 * @brief Main communication bridge node for TeensyV2 system
 * 
 * Provides the primary communication interface between TeensyV2 embedded
 * boards and the ROS2 ecosystem. Handles serial communication, message
 * parsing, and topic/service integration.
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sigyn_to_sensor_v2/teensy_bridge.h"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  RCLCPP_INFO(rclcpp::get_logger("teensy_bridge"), "Starting TeensyV2 Bridge Node");
  
  try {
    auto node = std::make_shared<sigyn_to_sensor_v2::TeensyBridge>();
    
    // Use multi-threaded executor for better performance
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "TeensyV2 Bridge Node initialized successfully");
    
    executor.spin();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("teensy_bridge"), 
                 "Failed to initialize TeensyV2 Bridge Node: %s", e.what());
    return 1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("teensy_bridge"), "TeensyV2 Bridge Node shutting down");
  rclcpp::shutdown();
  return 0;
}
