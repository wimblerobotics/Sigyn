// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#ifndef PERIMETER_ROAMER_V3__BT_NODES_HPP_
#define PERIMETER_ROAMER_V3__BT_NODES_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <sqlite3.h>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <chrono>

namespace perimeter_roamer_v3
{

enum class SpaceType {
  UNKNOWN,
  ROOM,
  HALLWAY,
  DOORWAY,
  VERY_NARROW
};

struct Wall {
  double start_x, start_y;
  double length, width;
  double angle; // orientation in radians
};

/**
 * @brief Base class for BT nodes that need ROS 2 node access
 */
class RosNodeBT {
protected:
  std::shared_ptr<rclcpp::Node> node_;
  
public:
  void setRosNode(std::shared_ptr<rclcpp::Node> node) {
    node_ = node;
  }
};

/**
 * @brief Custom behavior tree node to check battery state
 */
class CheckBatteryState : public BT::ConditionNode, public RosNodeBT
{
public:
  CheckBatteryState(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), battery_level_(100.0f), last_battery_time_(0), startup_time_(0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("low_battery_threshold", 20.0f, "Battery percentage threshold for low battery")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  float battery_level_;
  rclcpp::Time last_battery_time_;
  rclcpp::Time startup_time_;
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
};

/**
 * @brief Check LIDAR health by analyzing scan data
 */
class CheckLidarHealth : public BT::ConditionNode, public RosNodeBT
{
public:
  CheckLidarHealth(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), last_scan_time_(0), startup_time_(0)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("max_invalid_ratio", 0.5f, "Maximum ratio of invalid readings allowed"),
      BT::InputPort<float>("scan_timeout", 2.0f, "Timeout for scan data in seconds")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time startup_time_;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

/**
 * @brief Classify the current space based on LIDAR data and wall database
 */
class ClassifySpace : public BT::SyncActionNode, public RosNodeBT
{
public:
  ClassifySpace(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config), db_(nullptr)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("space_type", "Classified space type"),
      BT::OutputPort<geometry_msgs::msg::Pose>("suggested_pose", "Suggested next pose")
    };
  }

  BT::NodeStatus tick() override;

private:
  sqlite3* db_;
  std::vector<Wall> walls_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  bool loadWallDatabase();
  SpaceType analyzeSpace(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  geometry_msgs::msg::Pose generateNextPose(SpaceType space_type);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

/**
 * @brief Navigate to a specific pose using Nav2
 */
class NavigateToPose : public BT::StatefulActionNode, public RosNodeBT
{
public:
  NavigateToPose(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("target_pose", "Target pose to navigate to"),
      BT::InputPort<std::string>("behavior_tree", "Behavior tree for navigation"),
      BT::InputPort<float>("timeout", 30.0f, "Timeout for navigation in seconds"),
      BT::OutputPort<bool>("goal_interrupted", "Whether the goal was interrupted and needs to be resumed")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  
  enum class ActionState {
    IDLE,
    SENDING_GOAL,
    GOAL_ACTIVE,
    GOAL_COMPLETED,
    GOAL_FAILED,
    GOAL_INTERRUPTED
  };
  
  rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr> goal_handle_future_;
  
  ActionState action_state_ = ActionState::IDLE;
  std::atomic<bool> result_received_{false};
  std::atomic<BT::NodeStatus> navigation_result_{BT::NodeStatus::FAILURE};
  
  // Store goal for potential resume
  geometry_msgs::msg::Pose current_target_pose_;
  std::string current_behavior_tree_;
  float current_timeout_;
  std::chrono::steady_clock::time_point goal_start_time_;
  bool was_interrupted_ = false;

  bool sendGoal();
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr& goal_handle);
  void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result);
};

/**
 * @brief Condition nodes for different space types and states
 */
class CheckGoalInterrupted : public BT::ConditionNode
{
public:
  CheckGoalInterrupted(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("was_interrupted", "Whether a navigation goal was interrupted")
    };
  }

  BT::NodeStatus tick() override
  {
    bool was_interrupted = false;
    if (getInput("was_interrupted", was_interrupted) && was_interrupted) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsRoom : public BT::ConditionNode
{
public:
  IsRoom(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "ROOM") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsHallway : public BT::ConditionNode
{
public:
  IsHallway(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "HALLWAY") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsDoorway : public BT::ConditionNode
{
public:
  IsDoorway(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "DOORWAY") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

class IsVeryNarrow : public BT::ConditionNode
{
public:
  IsVeryNarrow(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("space_type", "Current space type from ClassifySpace")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string space_type;
    if (getInput("space_type", space_type) && space_type == "VERY_NARROW") {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
};

/**
 * @brief Waypoint management structures
 */
struct Waypoint {
  int id;
  double x, y, z;
  double qx, qy, qz, qw;  // quaternion orientation
  std::string text;
  bool visited;
};

/**
 * @brief Load waypoints from SQLite database
 */
class LoadWaypoints : public BT::SyncActionNode, public RosNodeBT
{
public:
  LoadWaypoints(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("database_path", "Path to waypoints database"),
      BT::OutputPort<bool>("waypoints_loaded", "Whether waypoints were successfully loaded")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::vector<Waypoint> waypoints_;
  bool loadWaypointsFromDatabase(const std::string& db_path);
};

/**
 * @brief Check if all waypoints have been completed
 */
class CheckWaypointsComplete : public BT::ConditionNode, public RosNodeBT
{
public:
  CheckWaypointsComplete(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("current_waypoint_index", "Current waypoint index"),
      BT::InputPort<int>("total_waypoints", "Total number of waypoints"),
      BT::InputPort<bool>("loop_waypoints", "Whether to loop through waypoints"),
      BT::OutputPort<bool>("all_complete", "Whether all waypoints are complete")
    };
  }

  BT::NodeStatus tick() override;
};

/**
 * @brief Get the next waypoint to visit
 */
class GetNextWaypoint : public BT::SyncActionNode, public RosNodeBT
{
public:
  GetNextWaypoint(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<int>("current_waypoint_index", "Current waypoint index"),
      BT::OutputPort<geometry_msgs::msg::Pose>("target_pose", "Target pose for next waypoint"),
      BT::OutputPort<int>("waypoint_id", "ID of next waypoint"),
      BT::OutputPort<std::string>("waypoint_name", "Name/text of next waypoint")
    };
  }

  BT::NodeStatus tick() override;

private:
  std::vector<Waypoint> getWaypointsFromBlackboard();
};

/**
 * @brief Navigate to a specific waypoint with proper action client handling
 */
class NavigateToWaypoint : public BT::StatefulActionNode, public RosNodeBT
{
public:
  using NavigateAction = nav2_msgs::action::NavigateToPose;
  
  NavigateToWaypoint(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Pose>("target_pose", "Target pose to navigate to"),
      BT::InputPort<int>("waypoint_id", "ID of waypoint being navigated to"),
      BT::InputPort<std::string>("waypoint_name", "Name of waypoint being navigated to"),
      BT::InputPort<float>("timeout", 60.0f, "Timeout for navigation in seconds")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle_;
  
  std::atomic<bool> result_received_{false};
  std::atomic<BT::NodeStatus> navigation_result_{BT::NodeStatus::FAILURE};
  
  // Waypoint info for logging
  int current_waypoint_id_ = -1;
  std::string current_waypoint_name_;
  
  // Timeout tracking
  std::chrono::steady_clock::time_point start_time_;
  float timeout_seconds_ = 60.0f;
  
  // Progress tracking
  std::chrono::steady_clock::time_point last_feedback_time_;
  double last_distance_remaining_ = -1.0;

  void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result);
  void feedbackCallback(rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr,
                       const std::shared_ptr<const NavigateAction::Feedback> feedback);
};

/**
 * @brief Mark a waypoint as visited
 */
class MarkWaypointVisited : public BT::SyncActionNode, public RosNodeBT
{
public:
  MarkWaypointVisited(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("waypoint_id", "ID of waypoint to mark as visited")
    };
  }

  BT::NodeStatus tick() override;
};

/**
 * @brief Increment waypoint index for sequential following
 */
class IncrementWaypointIndex : public BT::SyncActionNode, public RosNodeBT
{
public:
  IncrementWaypointIndex(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<int>("current_waypoint_index", "Current waypoint index")
    };
  }

  BT::NodeStatus tick() override;
};

} // namespace perimeter_roamer_v3

#endif // PERIMETER_ROAMER_V3__BT_NODES_HPP_
