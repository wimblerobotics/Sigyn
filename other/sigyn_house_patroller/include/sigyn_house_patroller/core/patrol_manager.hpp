#pragma once

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "sigyn_house_patroller/core/waypoint_manager.hpp"
#include "sigyn_house_patroller/core/threat_detection_manager.hpp"
#include "sigyn_house_patroller/core/navigation_coordinator.hpp"

// Custom message, service, and action includes
#include "sigyn_house_patroller/msg/patrol_status.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/srv/set_patrol_mode.hpp"
#include "sigyn_house_patroller/action/patrol_to_waypoint.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Defines the different operational modes for the patrol manager.
 */
enum class PatrolMode {
  IDLE,
  FULL_PATROL,
  ROOM_INSPECTION,
  EMERGENCY_RESPONSE
};

// Forward declarations
class WaypointManager;
class ThreatDetectionManager;

/**
 * @brief Manages patrol logic, modes, and coordination
 */
class PatrolManager : public rclcpp::Node {
public:
  using SetPatrolModeSrv = sigyn_house_patroller::srv::SetPatrolMode;
  using PatrolToWaypointAction = sigyn_house_patroller::action::PatrolToWaypoint;
  using GoalHandlePatrol = rclcpp_action::ServerGoalHandle<PatrolToWaypointAction>;

  explicit PatrolManager(
      const std::string& node_name,
      rclcpp::NodeOptions options,
      std::shared_ptr<WaypointManager> waypoint_manager,
      std::shared_ptr<NavigationCoordinator> navigation_coordinator,
      std::shared_ptr<ThreatDetectionManager> threat_detector);

  void StartFullPatrol();
  void StartRoomInspection(const std::string& room_name);
  void StopPatrol();

private:
  // Patrol execution
  void ExecuteFullPatrol();
  void ExecuteRoomInspection();

  // Callbacks
  void OnWaypointNavigationComplete(const NavigationResult& result);

  void SetPatrolModeCallback(const std::shared_ptr<SetPatrolModeSrv::Request> request,
                             std::shared_ptr<SetPatrolModeSrv::Response> response);
  void StatusTimerCallback();
  void HealthTimerCallback();

  rclcpp_action::GoalResponse PatrolGoalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const PatrolToWaypointAction::Goal> goal);

  rclcpp_action::CancelResponse PatrolCancelCallback(
      const std::shared_ptr<GoalHandlePatrol> goal_handle);

  void PatrolAcceptedCallback(const std::shared_ptr<GoalHandlePatrol> goal_handle);
  void ExecutePatrolAction(const std::shared_ptr<GoalHandlePatrol> goal_handle);


  // Member variables
  std::shared_ptr<WaypointManager> waypoint_manager_;
  std::shared_ptr<NavigationCoordinator> navigation_coordinator_;
  std::shared_ptr<ThreatDetectionManager> threat_detector_;

  PatrolMode current_mode_;
  std::vector<PatrolWaypoint> patrol_waypoints_;
  int patrol_index_;
  bool is_patrolling_;

  // ROS2 components
  rclcpp::Service<SetPatrolModeSrv>::SharedPtr set_patrol_mode_service_;
  rclcpp_action::Server<PatrolToWaypointAction>::SharedPtr patrol_action_server_;
  rclcpp::Publisher<sigyn_house_patroller::msg::PatrolStatus>::SharedPtr patrol_status_publisher_;
  rclcpp::Publisher<sigyn_house_patroller::msg::SystemHealth>::SharedPtr system_health_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;

  std::string PatrolModeToString(PatrolMode mode);
};

}  // namespace sigyn_house_patroller
