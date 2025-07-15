#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

#include "sigyn_house_patroller/core/patrol_manager.hpp"
#include "sigyn_house_patroller/core/waypoint_manager.hpp"
#include "sigyn_house_patroller/core/navigation_coordinator.hpp"
#include "sigyn_house_patroller/core/threat_detection_manager.hpp"

// Global pointer to shut down the node
static std::shared_ptr<rclcpp::Node> g_node = nullptr;

void SignalHandler(int signal) {
  (void)signal;  // Unused parameter
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Shutting down patrol coordinator...");
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  signal(SIGINT, SignalHandler);
  signal(SIGTERM, SignalHandler);
  
  try {
    // Create a generic node for the managers that are not nodes themselves
    auto node = std::make_shared<rclcpp::Node>("patrol_coordinator_dependencies");
    g_node = node;

    // Instantiate managers
    auto waypoint_manager = std::make_shared<sigyn_house_patroller::WaypointManager>(node);
    auto navigation_coordinator = std::make_shared<sigyn_house_patroller::NavigationCoordinator>(node);
    auto threat_detector = std::make_shared<sigyn_house_patroller::ThreatDetectionManager>(node);

    // Create patrol manager node, injecting dependencies
    rclcpp::NodeOptions options;
    auto patrol_manager = std::make_shared<sigyn_house_patroller::PatrolManager>(
        "patrol_coordinator",
        options, 
        waypoint_manager, 
        navigation_coordinator, 
        threat_detector);
    
    RCLCPP_INFO(patrol_manager->get_logger(), "Patrol Coordinator starting up...");
    
    // Spin the patrol manager node
    rclcpp::spin(patrol_manager);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("patrol_coordinator"), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("patrol_coordinator"), "Patrol Coordinator shutdown complete");
  
  return 0;
}
