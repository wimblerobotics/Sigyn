#include <rclcpp/rclcpp.hpp>
#include "sigyn_lidar_v2/multi_lidar_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sigyn_lidar_v2::MultiLidarNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
