/**
 * @file safety_coordinator.h
 * @brief Safety coordination ROS2 node for TeensyV2 system
 * 
 * Coordinates safety operations across multiple TeensyV2 boards,
 * manages E-stop states, and publishes safety status.
 * 
 * @author GitHub Copilot
 * @date 2025
 */

#ifndef SIGYN_TO_SENSOR_V2_SAFETY_COORDINATOR_H_
#define SIGYN_TO_SENSOR_V2_SAFETY_COORDINATOR_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace sigyn_to_sensor_v2 {

/**
 * @brief ROS2 node for safety coordination across TeensyV2 boards
 * 
 * Manages global E-stop state, coordinates safety operations between boards,
 * and provides safety services for the overall robot system.
 */
class SafetyCoordinator : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   */
  SafetyCoordinator();

  /**
   * @brief Destructor
   */
  ~SafetyCoordinator() = default;

  /**
   * @brief Process safety status from TeensyV2 board
   * @param board_id Board ID
   * @param estop_active E-stop status from this board
   * @param safety_level Safety level (0=OK, 1=WARN, 2=ERROR, 3=FATAL)
   * @param safety_message Safety status message
   */
  void ProcessSafetyStatus(uint8_t board_id, bool estop_active, 
                          uint8_t safety_level, const std::string& safety_message);

  /**
   * @brief Process inter-board safety coordination message
   * @param source_board Source board ID
   * @param target_board Target board ID (0 = broadcast)
   * @param command Safety command
   */
  void ProcessSafetyCoordination(uint8_t source_board, uint8_t target_board, 
                                const std::string& command);

private:
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr global_estop_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr safety_diagnostics_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_status_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_estop_service_;

  // Parameters
  bool auto_reset_enabled_;           ///< Allow automatic E-stop reset
  double safety_timeout_seconds_;     ///< Timeout for safety messages
  uint8_t global_safety_level_;       ///< Global safety level threshold

  // Timer for periodic safety checks
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // Safety state tracking
  struct BoardSafetyState {
    bool estop_active = false;
    uint8_t safety_level = 0;
    std::string safety_message = "Unknown";
    rclcpp::Time last_update;
    bool data_valid = false;
    bool communication_timeout = false;
  };
  
  std::map<uint8_t, BoardSafetyState> board_safety_states_;  ///< Per-board safety states
  bool global_estop_active_;  ///< Global E-stop state
  rclcpp::Time last_global_estop_time_;

  /**
   * @brief Initialize ROS2 parameters
   */
  void InitializeParameters();

  /**
   * @brief Parameter change callback
   * @param event Parameter event
   */
  void ParameterCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  /**
   * @brief Periodic safety check and timeout detection
   */
  void SafetyCheckTimer();

  /**
   * @brief Update global E-stop state based on all boards
   */
  void UpdateGlobalEstopState();

  /**
   * @brief Publish safety diagnostics
   */
  void PublishSafetyDiagnostics();

  /**
   * @brief Emergency stop service callback
   * @param request Service request
   * @param response Service response
   */
  void EmergencyStopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Reset E-stop service callback
   * @param request Service request
   * @param response Service response
   */
  void ResetEstopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Check if board communication has timed out
   * @param state Board safety state
   * @return True if communication has timed out
   */
  bool IsCommTimeoutExpired(const BoardSafetyState& state);

  /**
   * @brief Get overall system safety level
   * @return Highest safety level across all boards
   */
  uint8_t GetSystemSafetyLevel();

  /**
   * @brief Generate safety status message
   * @return Human-readable safety status
   */
  std::string GenerateSafetyStatusMessage();
};

}  // namespace sigyn_to_sensor_v2

#endif  // SIGYN_TO_SENSOR_V2_SAFETY_COORDINATOR_H_
