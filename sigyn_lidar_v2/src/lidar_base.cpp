#include "sigyn_lidar_v2/lidar_base.hpp"
#include "sigyn_lidar_v2/ld06_driver.hpp"

namespace sigyn_lidar_v2 {

std::unique_ptr<LidarBase> LidarDriverFactory::create_driver(const std::string& device_type) {
  if (device_type == "LD06" || device_type == "ld06") {
    return std::make_unique<LD06Driver>();
  }
  // Add more driver types here as they are implemented
  // else if (device_type == "LD09" || device_type == "ld09") {
  //   return std::make_unique<LD09Driver>();
  // }
  
  return nullptr;
}

std::vector<std::string> LidarDriverFactory::get_supported_types() {
  return {"LD06"};  // Add more types as they are implemented
}

} // namespace sigyn_lidar_v2
