#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "sigyn_house_patroller/msg/threat_alert.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Manages threat detection activities
 */
class ThreatDetectionManager {
public:
  explicit ThreatDetectionManager(std::shared_ptr<rclcpp::Node> node);
  ~ThreatDetectionManager() = default;

  bool Initialize();
  std::vector<msg::ThreatAlert> RunDetection();

private:
  std::shared_ptr<rclcpp::Node> node_;
};

}  // namespace sigyn_house_patroller
