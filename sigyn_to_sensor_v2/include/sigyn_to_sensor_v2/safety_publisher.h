/**
 * @file safety_publisher.h
 * @brief Safety message publisher for TeensyV2 system
 * 
 * Publishes safety-related diagnostics and E-stop status from TeensyV2 boards.
 * Part of the Sigyn House Patroller robot system.
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#ifndef SIGYN_TO_SENSOR_V2_SAFETY_PUBLISHER_H_
#define SIGYN_TO_SENSOR_V2_SAFETY_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace sigyn_to_sensor_v2 {

/**
 * @brief Publishes safety-related messages from TeensyV2 boards
 * 
 * Handles E-stop status, safety diagnostics, and inter-board coordination
 * messages from the TeensyV2 embedded system.
 */
class SafetyPublisher {
public:
  /**
   * @brief Constructor
   * @param node Shared pointer to the ROS2 node
   */
  explicit SafetyPublisher(rclcpp::Node::SharedPtr node);

  /**
   * @brief Destructor
   */
  ~SafetyPublisher() = default;

  /**
   * @brief Publish E-stop status
   * @param estop_active True if E-stop is active
   * @param board_id Board ID that reported the status
   */
  void PublishEstopStatus(bool estop_active, uint8_t board_id);

  /**
   * @brief Publish safety diagnostics
   * @param board_id Board ID
   * @param safety_level Safety level (0=OK, 1=WARN, 2=ERROR, 3=FATAL)
   * @param message Safety message
   */
  void PublishSafetyDiagnostics(uint8_t board_id, uint8_t safety_level, const std::string& message);

  /**
   * @brief Publish inter-board safety coordination message
   * @param source_board Source board ID
   * @param target_board Target board ID (0 = broadcast)
   * @param command Safety command
   */
  void PublishSafetyCoordination(uint8_t source_board, uint8_t target_board, const std::string& command);

private:
  rclcpp::Node::SharedPtr node_;  ///< ROS2 node handle
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_coordination_pub_;

  /**
   * @brief Convert safety level to diagnostic status level
   * @param safety_level TeensyV2 safety level
   * @return ROS2 diagnostic status level
   */
  uint8_t ConvertSafetyLevel(uint8_t safety_level);
};

}  // namespace sigyn_to_sensor_v2

#endif  // SIGYN_TO_SENSOR_V2_SAFETY_PUBLISHER_H_
