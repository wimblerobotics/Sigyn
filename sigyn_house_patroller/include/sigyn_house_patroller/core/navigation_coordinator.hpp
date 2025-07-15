#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav2_msgs/srv/clear_costmap.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sigyn_house_patroller/core/waypoint_manager.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Navigation result status
 */
enum class NavigationStatus {
  kIdle,
  kNavigating,
  kSuccess,
  kFailed,
  kCancelled,
  kTimeout
};

/**
 * @brief Navigation result structure
 */
struct NavigationResult {
  NavigationStatus status;
  std::string message;
  double distance_traveled;
  std::chrono::milliseconds duration;
  std::chrono::system_clock::time_point completion_time;
  
  NavigationResult() : status(NavigationStatus::kIdle), distance_traveled(0.0), 
                      duration(std::chrono::milliseconds::zero()) {}
};

/**
 * @brief Navigation coordinator for autonomous house patrol
 */
class NavigationCoordinator {
public:
  using NavigationCallback = std::function<void(const NavigationResult&)>;
  using FeedbackCallback = std::function<void(const geometry_msgs::msg::PoseStamped&, double)>;
  using NavigateToAction = nav2_msgs::action::NavigateToPose;
  using NavigateToGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToAction>;

  explicit NavigationCoordinator(rclcpp::Node::SharedPtr node);
  ~NavigationCoordinator() = default;

  /**
   * @brief Initialize the navigation coordinator
   * @return True if initialization successful
   */
  bool Initialize();

  /**
   * @brief Navigate to a specific waypoint
   * @param waypoint Target waypoint
   * @param callback Callback for navigation completion
   * @param feedback_callback Callback for navigation feedback
   * @return True if navigation request was sent successfully
   */
  bool NavigateToWaypoint(const PatrolWaypoint& waypoint,
                         NavigationCallback callback = nullptr,
                         FeedbackCallback feedback_callback = nullptr);

  /**
   * @brief Navigate to a specific pose
   * @param pose Target pose
   * @param callback Callback for navigation completion
   * @param feedback_callback Callback for navigation feedback
   * @return True if navigation request was sent successfully
   */
  bool NavigateToPose(const geometry_msgs::msg::PoseStamped& pose,
                     NavigationCallback callback = nullptr,
                     FeedbackCallback feedback_callback = nullptr);

  /**
   * @brief Cancel current navigation
   * @return True if cancellation was successful
   */
  bool CancelNavigation();

  /**
   * @brief Check if currently navigating
   * @return True if navigation is in progress
   */
  bool IsNavigating() const;

  /**
   * @brief Get current navigation status
   * @return Current navigation status
   */
  NavigationStatus GetNavigationStatus() const;

  /**
   * @brief Get last navigation result
   * @return Last navigation result
   */
  NavigationResult GetLastResult() const;

  /**
   * @brief Set navigation timeout
   * @param timeout Timeout duration
   */
  void SetNavigationTimeout(const std::chrono::seconds& timeout);

  /**
   * @brief Get estimated time to reach waypoint
   * @param waypoint Target waypoint
   * @return Estimated travel time
   */
  std::chrono::seconds GetEstimatedTravelTime(const PatrolWaypoint& waypoint) const;

  /**
   * @brief Check if path to waypoint is clear
   * @param waypoint Target waypoint
   * @return True if path is clear
   */
  bool IsPathClear(const PatrolWaypoint& waypoint) const;

  /**
   * @brief Get current robot pose
   * @return Current robot pose
   */
  geometry_msgs::msg::PoseStamped GetCurrentPose() const;

  /**
   * @brief Calculate distance to waypoint
   * @param waypoint Target waypoint
   * @return Distance in meters
   */
  double CalculateDistanceToWaypoint(const PatrolWaypoint& waypoint) const;

  /**
   * @brief Clear costmaps to recover from navigation issues
   * @return True if costmaps were cleared successfully
   */
  bool ClearCostmaps();

  /**
   * @brief Check if robot is stuck
   * @return True if robot appears to be stuck
   */
  bool IsRobotStuck() const;

  /**
   * @brief Execute recovery behavior
   * @return True if recovery was successful
   */
  bool ExecuteRecovery();

  /**
   * @brief Check navigation system health
   * @return True if navigation system is healthy
   */
  bool IsNavigationHealthy() const;

  /**
   * @brief Get navigation performance metrics
   * @return Map of performance metrics
   */
  std::unordered_map<std::string, double> GetPerformanceMetrics() const;

private:
  /**
   * @brief Goal response callback
   * @param goal_handle Goal handle from action server
   */
  void NavigationGoalResponseCallback(const NavigateToGoalHandle::SharedPtr& goal_handle);

  /**
   * @brief Feedback callback
   * @param goal_handle Goal handle
   * @param feedback Feedback message
   */
  void NavigationFeedbackCallback(
    NavigateToGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateToAction::Feedback> feedback);

  /**
   * @brief Result callback
   * @param result Result message
   */
  void NavigationResultCallback(const NavigateToGoalHandle::WrappedResult& result);

  /**
   * @brief Check for navigation timeout
   */
  void CheckNavigationTimeout();

  /**
   * @brief Update navigation metrics
   * @param pose Current pose
   */
  void UpdateNavigationMetrics(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief Execute navigation goal
   * @param pose Target pose
   * @return True if goal was sent successfully
   */
  bool ExecuteNavigationGoal(const geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief Transform pose to map frame
   * @param pose Pose to transform
   * @return Transformed pose
   */
  geometry_msgs::msg::PoseStamped TransformToMapFrame(
    const geometry_msgs::msg::PoseStamped& pose) const;

  /**
   * @brief Check if pose is valid
   * @param pose Pose to validate
   * @return True if pose is valid
   */
  bool IsValidPose(const geometry_msgs::msg::PoseStamped& pose) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Action client for navigation
  rclcpp_action::Client<NavigateToAction>::SharedPtr nav_action_client_;
  
  // Service clients
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr get_costmap_client_;
  rclcpp::Client<nav2_msgs::srv::ClearCostmap>::SharedPtr clear_costmap_client_;
  
  // Navigation state
  NavigationStatus current_status_;
  NavigationResult last_result_;
  NavigateToGoalHandle::SharedPtr current_goal_handle_;
  
  // Callbacks
  NavigationCallback navigation_callback_;
  FeedbackCallback feedback_callback_;
  
  // Timeout handling
  std::chrono::seconds navigation_timeout_;
  std::chrono::system_clock::time_point navigation_start_time_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  
  // Performance tracking
  std::vector<geometry_msgs::msg::PoseStamped> pose_history_;
  std::chrono::system_clock::time_point last_movement_time_;
  double total_distance_traveled_;
  size_t successful_navigations_;
  size_t failed_navigations_;
  
  mutable std::mutex status_mutex_;
  mutable std::mutex metrics_mutex_;
  
  // Configuration parameters
  std::string map_frame_;
  std::string robot_frame_;
  double stuck_threshold_;
  double min_movement_distance_;
  size_t max_pose_history_;
  std::chrono::seconds stuck_timeout_;
};

}  // namespace sigyn_house_patroller
