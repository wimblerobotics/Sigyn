#include "sigyn_house_patroller/core/threat_detection_manager.hpp"

namespace sigyn_house_patroller {

ThreatDetectionManager::ThreatDetectionManager(std::shared_ptr<rclcpp::Node> node) : node_(node) {}

bool ThreatDetectionManager::Initialize() {
  // Stub implementation
  return true;
}

std::vector<msg::ThreatAlert> ThreatDetectionManager::RunDetection() {
  // Stub implementation
  return {};
}

}  // namespace sigyn_house_patroller
