#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sigyn_house_patroller/msg/patrol_status.hpp"
#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/srv/set_patrol_mode.hpp"
#include "sigyn_house_patroller/action/patrol_to_waypoint.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Main coordinator for the house patrol system
 * 
 * This class orchestrates all patrol activities, manages waypoint navigation,
 * coordinates with feature recognizers, and handles threat responses.
 */
class PatrolManager : public rclcpp::Node {
public:
  /**
   * @brief Patrol modes enumeration
   */
  enum class PatrolMode {
    kIdle,
    kPatrolling,
    kInvestigating,
    kCharging,
    kEmergency
  };

  /**
   * @brief Waypoint priority levels
   */
  enum class WaypointPriority {
    kLow = 1,
    kMedium = 2,
    kHigh = 3,
    kCritical = 4
  };

  /**
   * @brief Waypoint information structure
   */
  struct WaypointInfo {
    std::string name;
    geometry_msgs::msg::PoseStamped pose;
    std::string room_name;
    WaypointPriority priority;
    std::vector<std::string> required_checks;
    std::chrono::system_clock::time_point last_visited;
    double visit_frequency_hours;
    bool is_critical_point;
  };

  explicit PatrolManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  
  ~PatrolManager() = default;

  /**
   * @brief Start the patrol system
   * @param mode Initial patrol mode
   * @param waypoint_set Name of waypoint set to use
   * @return True if successfully started
   */
  bool StartPatrol(PatrolMode mode, const std::string& waypoint_set = "default");

  /**
   * @brief Stop the patrol system
   */
  void StopPatrol();

  /**
   * @brief Get current patrol status
   * @return Current patrol status message
   */
  msg::PatrolStatus GetPatrolStatus() const;

  /**
   * @brief Handle incoming threat alert
   * @param threat_alert The threat alert to process
   */
  void HandleThreatAlert(const msg::ThreatAlert& threat_alert);

  /**
   * @brief Get current patrol mode
   * @return Current patrol mode
   */
  PatrolMode GetCurrentMode() const { return current_mode_; }

private:
  // Core functionality
  void InitializeWaypoints();
  void LoadWaypointConfiguration();
  void UpdatePatrolStatus();
  void ProcessThreatAlerts();
  void ManageWaypointQueue();
  void NavigateToNextWaypoint();
  void HandleNavigationResult();
  void CheckSystemHealth();
  void HandleEmergencyMode();
  void HandleChargingMode();
  void UpdateLocationTracking();
  
  // Callback functions
  void PatrolTimerCallback();
  void StatusTimerCallback();
  void ThreatAlertCallback(const msg::ThreatAlert::SharedPtr msg);
  void SystemHealthCallback(const msg::SystemHealth::SharedPtr msg);
  void SetPatrolModeCallback(
    const std::shared_ptr<srv::SetPatrolMode::Request> request,
    std::shared_ptr<srv::SetPatrolMode::Response> response);
  void NavigationGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle);
  void NavigationResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);
  void NavigationFeedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

  // Utility functions
  std::string PatrolModeToString(PatrolMode mode) const;
  PatrolMode StringToPatrolMode(const std::string& mode_str) const;
  WaypointInfo* GetNextWaypoint();
  bool IsWaypointDue(const WaypointInfo& waypoint) const;
  double CalculateWaypointPriority(const WaypointInfo& waypoint) const;
  void UpdateWaypointVisitTime(const std::string& waypoint_name);
  bool IsAtWaypoint(const WaypointInfo& waypoint, double tolerance = 0.5) const;
  geometry_msgs::msg::PoseStamped GetCurrentPose() const;
  std::string GetCurrentRoom() const;

  // Member variables
  PatrolMode current_mode_;
  std::string current_waypoint_set_;
  std::unordered_map<std::string, std::vector<WaypointInfo>> waypoint_sets_;
  std::vector<WaypointInfo*> waypoint_queue_;
  WaypointInfo* current_waypoint_;
  msg::PatrolStatus patrol_status_;
  msg::SystemHealth system_health_;
  std::vector<msg::ThreatAlert> recent_threats_;
  
  // Threading and synchronization
  mutable std::mutex status_mutex_;
  mutable std::mutex waypoint_mutex_;
  mutable std::mutex threat_mutex_;
  
  // ROS2 infrastructure
  rclcpp::TimerBase::SharedPtr patrol_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  
  // Publishers and subscribers
  rclcpp::Publisher<msg::PatrolStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_response_pub_;
  rclcpp::Subscription<msg::ThreatAlert>::SharedPtr threat_alert_sub_;
  rclcpp::Subscription<msg::SystemHealth>::SharedPtr system_health_sub_;
  
  // Services
  rclcpp::Service<srv::SetPatrolMode>::SharedPtr set_patrol_mode_service_;
  
  // Action clients
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav_goal_handle_;
  
  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Configuration parameters
  double patrol_frequency_hz_;
  double status_frequency_hz_;
  double waypoint_tolerance_;
  double navigation_timeout_;
  double battery_critical_level_;
  double battery_low_level_;
  std::string base_frame_;
  std::string map_frame_;
  bool use_perimeter_following_;
  
  // Statistics
  std::chrono::system_clock::time_point patrol_start_time_;
  uint32_t waypoints_completed_;
  uint32_t threats_detected_;
  uint32_t alerts_sent_;
  uint32_t navigation_failures_;
};

}  // namespace sigyn_house_patroller
