#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/msg/patrol_status.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Manages and switches behavior trees based on patrol state and threat level
 */
class BehaviorTreeManager : public rclcpp::Node {
public:
  explicit BehaviorTreeManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BehaviorTreeManager() = default;

  /**
   * @brief Switch to a specific behavior tree
   * @param tree_name Name of the behavior tree to switch to
   * @return true if successful, false otherwise
   */
  bool SwitchBehaviorTree(const std::string& tree_name);

  /**
   * @brief Get the current active behavior tree name
   * @return Current behavior tree name
   */
  std::string GetCurrentBehaviorTree() const;

  /**
   * @brief Check if a behavior tree switch is in progress
   * @return true if switching, false otherwise
   */
  bool IsSwitching() const;

private:
  enum class BehaviorTreeState {
    NORMAL_PATROL,
    THREAT_INVESTIGATION,
    EMERGENCY_RESPONSE,
    BATTERY_CRITICAL,
    MAINTENANCE_MODE
  };

  struct BehaviorTreeConfig {
    std::string xml_path;
    std::string description;
    int priority;  // Higher number = higher priority
    bool requires_restart;
  };

  // ROS2 interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_pub_;
  rclcpp::Subscription<msg::ThreatAlert>::SharedPtr threat_alert_sub_;
  rclcpp::Subscription<msg::SystemHealth>::SharedPtr system_health_sub_;
  rclcpp::Subscription<msg::PatrolStatus>::SharedPtr patrol_state_sub_;
  
  // Service clients for Nav2 lifecycle management
  rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr lifecycle_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;

  // Service server for manual BT switching
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;

  // Timers
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Configuration
  std::unordered_map<std::string, BehaviorTreeConfig> behavior_trees_;
  std::string config_directory_;
  std::string current_behavior_tree_;
  std::string target_behavior_tree_;
  BehaviorTreeState current_state_;
  
  // State management
  bool is_switching_;
  std::chrono::steady_clock::time_point last_switch_time_;
  double switch_cooldown_;
  mutable std::mutex state_mutex_;
  bool maintenance_mode_;
  
  // Health monitoring
  double system_health_score_;
  int threat_level_;
  bool battery_critical_;
  
  // Callback functions
  void ThreatAlertCallback(const msg::ThreatAlert::SharedPtr msg);
  void SystemHealthCallback(const msg::SystemHealth::SharedPtr msg);
  void PatrolStateCallback(const msg::PatrolStatus::SharedPtr msg);
  void MonitoringTimerCallback();
  void StatusTimerCallback();
  
  // Service callbacks
  void SwitchServiceCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Internal methods
  void LoadBehaviorTreeConfigs();
  void DetermineOptimalBehaviorTree();
  bool ExecuteBehaviorTreeSwitch(const std::string& tree_name);
  bool RestartBTNavigator();
  bool UpdateBTNavigatorParameter(const std::string& param_name, const std::string& value);
  void PublishCurrentStatus();
  
  // State evaluation methods
  BehaviorTreeState EvaluateSystemState();
  std::string GetBehaviorTreeForState(BehaviorTreeState state);
  bool ShouldSwitchBehaviorTree(const std::string& new_tree);
  
  // Utility methods
  void LogBehaviorTreeSwitch(const std::string& from, const std::string& to, const std::string& reason);
  bool ValidateBehaviorTreeExists(const std::string& tree_name);
};

}  // namespace sigyn_house_patroller
