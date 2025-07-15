#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

#include "sigyn_house_patroller/core/patrol_manager.hpp"

static std::shared_ptr<sigyn_house_patroller::PatrolManager> patrol_manager;

void SignalHandler(int signal) {
  (void)signal;  // Unused parameter
  
  if (patrol_manager) {
    RCLCPP_INFO(rclcpp::get_logger("patrol_coordinator"), "Shutting down patrol coordinator...");
    patrol_manager->Shutdown();
  }
  
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Set up signal handler
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  
  try {
    // Create patrol manager node
    patrol_manager = std::make_shared<sigyn_house_patroller::PatrolManager>("patrol_coordinator");
    
    // Initialize the patrol manager
    if (!patrol_manager->Initialize()) {
      RCLCPP_ERROR(rclcpp::get_logger("patrol_coordinator"), "Failed to initialize patrol coordinator");
      return 1;
    }
    
    // Start the patrol manager
    patrol_manager->Start();
    
    RCLCPP_INFO(rclcpp::get_logger("patrol_coordinator"), "Patrol Coordinator started successfully");
    
    // Spin the node
    rclcpp::spin(patrol_manager);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("patrol_coordinator"), "Exception in main: %s", e.what());
    return 1;
  }
  
  // Clean shutdown
  patrol_manager->Shutdown();
  patrol_manager.reset();
  
  RCLCPP_INFO(rclcpp::get_logger("patrol_coordinator"), "Patrol Coordinator shutdown complete");
  
  return 0;
}
