// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#ifndef CAN_DO_CHALLENGE__BT_NODES_REAL_HPP_
#define CAN_DO_CHALLENGE__BT_NODES_REAL_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "sigyn_interfaces/action/move_elevator.hpp"
#include "sigyn_interfaces/action/move_extender.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <memory>
#include <string>
#include <atomic>
#include <chrono>
#include <fstream>

namespace can_do_challenge
{

static constexpr double kDefaultWithinReachDistance = -1.0;

/**
 * @brief Structure to hold detected object information
 */
struct BoundingBox {
  int x, y, width, height;
};

struct DetectedObject {
  std::string name;
  BoundingBox bounding_box;
  double confidence;
  double distance_z; // Distance to object (if available from depth)
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

// ============================================================================
// SAFETY CONDITION NODES
// ============================================================================

class BatteryAboveChargingVoltage : public BT::ConditionNode, public RosNodeBT
{
public:
  BatteryAboveChargingVoltage(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class BatteryAboveCriticalVoltage : public BT::ConditionNode, public RosNodeBT
{
public:
  BatteryAboveCriticalVoltage(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class RobotIsEstopped : public BT::ConditionNode, public RosNodeBT
{
public:
  RobotIsEstopped(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class RobotTiltedCritically : public BT::ConditionNode, public RosNodeBT
{
public:
  RobotTiltedCritically(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class RobotTiltedWarning : public BT::ConditionNode, public RosNodeBT
{
public:
  RobotTiltedWarning(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ============================================================================
// VISION CONDITION NODES
// ============================================================================

class CanDetectedByOAKD : public BT::ConditionNode, public RosNodeBT
{
public:
  CanDetectedByOAKD(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

class CanDetectedByPiCamera : public BT::ConditionNode, public RosNodeBT
{
public:
  CanDetectedByPiCamera(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

class CanCenteredInPiCamera : public BT::ConditionNode, public RosNodeBT
{
public:
  CanCenteredInPiCamera(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

class CanWithinReach : public BT::ConditionNode, public RosNodeBT
{
public:
  CanWithinReach(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<bool>("can_detected", "Whether the can was detected"),
      BT::InputPort<geometry_msgs::msg::PointStamped>("can_location", "Can location in base_link frame"),
      BT::InputPort<double>("within_distance", kDefaultWithinReachDistance,
                            "Distance threshold (m) to consider the can within reach")
    };
  }
  BT::NodeStatus tick() override;
};

class CanIsGrasped : public BT::ConditionNode, public RosNodeBT
{
public:
  CanIsGrasped(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

class WaitForNewPiFrameProcessed : public BT::ConditionNode, public RosNodeBT
{
public:
  WaitForNewPiFrameProcessed(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config), wait_start_time_(rclcpp::Time(0)), last_frame_time_(rclcpp::Time(0)), waiting_(false) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;

private:
  rclcpp::Time wait_start_time_;
  rclcpp::Time last_frame_time_;
  bool waiting_;
};

class ElevatorAtHeight : public BT::ConditionNode, public RosNodeBT
{
public:
  ElevatorAtHeight(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { 
      BT::InputPort<double>("targetHeight"),
      BT::InputPort<double>("targetHeightPixels"),
      BT::InputPort<double>("z_tolerance_pixels")
    };
  }
  BT::NodeStatus tick() override;
};

class OAKDDetectCan : public BT::StatefulActionNode, public RosNodeBT
{
public:
  OAKDDetectCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), last_detection_stamp_(rclcpp::Time(0)) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("objectOfInterest", "Object to detect"),
      BT::OutputPort<bool>("can_detected", "Whether the can was detected"),
      BT::OutputPort<geometry_msgs::msg::PointStamped>("can_location", "Can location in base_link frame")
    };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus detectOnce();
  rclcpp::Time last_detection_stamp_;
};

// ============================================================================
// NAVIGATION ACTION NODES
// ============================================================================

class ComputePathToCanLocation : public BT::SyncActionNode, public RosNodeBT
{
public:
  ComputePathToCanLocation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("location"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<nav_msgs::msg::Path>("path")
    };
  }
  BT::NodeStatus tick() override;
};

class ComputePathToPose : public BT::SyncActionNode, public RosNodeBT
{
public:
  ComputePathToPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<std::string>("planner_id"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<int>("error_code_id")
    };
  }
  BT::NodeStatus tick() override;
};

class FollowPath : public BT::SyncActionNode, public RosNodeBT
{
public:
  FollowPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("controller_id"),
      BT::OutputPort<int>("error_code_id")
    };
  }
  BT::NodeStatus tick() override;
};

class MoveTowardsCan : public BT::SyncActionNode, public RosNodeBT
{
public:
  MoveTowardsCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<bool>("can_detected", "Whether the can was detected"),
      BT::InputPort<geometry_msgs::msg::PointStamped>("can_location", "Can location in base_link frame"),
      BT::InputPort<double>("target_distance_from_object", 0.01, "Desired distance to stop from object (meters)"),
      BT::InputPort<double>("distance_tolerance", 0.01, "Distance tolerance around target distance (meters)"),
      BT::InputPort<double>("angular_tolerance", 0.0523598776, "Angular tolerance (radians)"),
      BT::InputPort<double>("angular_velocity", 0.35, "Angular velocity (rad/sec)"),
      BT::InputPort<double>("linear_velocity", 0.15, "Linear velocity (m/sec)"),
      BT::InputPort<double>("commands_per_sec", 30.0, "Max cmd_vel commands per second")
    };
  }
  
  BT::NodeStatus tick() override;
};

class WaitForNewOAKDFrame : public BT::StatefulActionNode, public RosNodeBT
{
public:
  WaitForNewOAKDFrame(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config), last_frame_timestamp_(0, 0, RCL_ROS_TIME) {}
  
  static BT::PortsList providedPorts() { return {}; }
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};
  
private:
  rclcpp::Time last_frame_timestamp_;  // Timestamp of last processed frame
};

class SleepSeconds : public BT::StatefulActionNode, public RosNodeBT
{
public:
  SleepSeconds(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("seconds", 1.0, "Seconds to wait") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {};

private:
  rclcpp::Time start_time_;
  double wait_seconds_ = 1.0;
};

// Compute a short-horizon Nav2 goal from the current OAK-D detection.
// The goal is expressed in the global frame expected by Nav2 (typically "map").
class ComputeApproachGoalToCan : public BT::StatefulActionNode, public RosNodeBT
{
public:
  ComputeApproachGoalToCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("objectOfInterest"),
      BT::InputPort<double>("within_distance", kDefaultWithinReachDistance,
                            "Distance threshold (m) to consider the can within reach"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}
};

class RotateRobot : public BT::StatefulActionNode, public RosNodeBT
{
public:
  RotateRobot(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("degrees", 0.0, "Degrees to rotate") };
  }
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Time start_time_;
  double target_duration_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

/**
 * @brief Async NavigateToPose action using Nav2
 */
class NavigateToPoseAction : public BT::StatefulActionNode, public RosNodeBT
{
public:
  NavigateToPoseAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf)
    : BT::StatefulActionNode(xml_tag_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Target pose to navigate to"),
      BT::InputPort<std::string>("behavior_tree", "Behavior tree for navigation"),
      BT::OutputPort<int>("error_code_id", "Error code from navigation")
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
    GOAL_FAILED
  };
  
  rclcpp_action::Client<NavigateAction>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr> goal_handle_future_;
  
  ActionState action_state_ = ActionState::IDLE;
  std::atomic<bool> result_received_{false};
  std::atomic<BT::NodeStatus> navigation_result_{BT::NodeStatus::FAILURE};
  
  geometry_msgs::msg::PoseStamped current_goal_;
  std::string current_behavior_tree_;
  std::chrono::steady_clock::time_point goal_start_time_;

  bool sendGoal();
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr& goal_handle);
  void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result);
};

// ============================================================================
// GRIPPER/ELEVATOR ACTION NODES
// ============================================================================

class LowerElevator : public BT::SyncActionNode, public RosNodeBT
{
public:
  LowerElevator(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class LowerElevatorSafely : public BT::SyncActionNode, public RosNodeBT
{
public:
  LowerElevatorSafely(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class LowerElevatorToTable : public BT::SyncActionNode, public RosNodeBT
{
public:
  LowerElevatorToTable(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("targetHeight", "Target height for approach")};
  }
  BT::NodeStatus tick() override;
};

class MoveElevatorToHeight : public BT::SyncActionNode, public RosNodeBT
{
public:
  MoveElevatorToHeight(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("targetHeight") };
  }
  BT::NodeStatus tick() override;
};

class StepElevatorUp : public BT::StatefulActionNode, public RosNodeBT
{
public:
  StepElevatorUp(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("stepMeters", 0.01, "Step size in meters")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool home_position_set_ = false;
  double home_position_ = 0.0;
  double last_commanded_ = 0.0;
  int step_count_ = 0;
};

class MoveElevatorAction : public BT::StatefulActionNode, public RosNodeBT
{
public:
  MoveElevatorAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("goal_position", "Target elevator position in meters (0.0-0.8999)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using MoveElevatorActionType = sigyn_interfaces::action::MoveElevator;
  
  enum class ActionState {
    IDLE,
    SENDING_GOAL,
    GOAL_ACTIVE,
    GOAL_COMPLETED,
    GOAL_FAILED,
    GOAL_CANCELED
  };
  
  rclcpp_action::Client<MoveElevatorActionType>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr> goal_handle_future_;
  
  ActionState action_state_ = ActionState::IDLE;
  std::atomic<bool> result_received_{false};
  std::atomic<BT::NodeStatus> action_result_{BT::NodeStatus::FAILURE};
  double goal_position_ = 0.0;

  bool sendGoal();
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr& goal_handle);
  void resultCallback(const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::WrappedResult& result);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr,
    const std::shared_ptr<const MoveElevatorActionType::Feedback> feedback);
};

class StepElevatorUpAction : public BT::StatefulActionNode, public RosNodeBT
{
public:
  StepElevatorUpAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("stepMeters", 0.02, "Step size in meters")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using MoveElevatorActionType = sigyn_interfaces::action::MoveElevator;
  
  enum class ActionState {
    IDLE,
    READING_POSITION,
    SENDING_GOAL,
    GOAL_ACTIVE,
    GOAL_COMPLETED,
    GOAL_FAILED
  };
  
  rclcpp_action::Client<MoveElevatorActionType>::SharedPtr action_client_;
  rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr> goal_handle_future_;
  
  ActionState action_state_ = ActionState::IDLE;
  std::atomic<bool> result_received_{false};
  std::atomic<BT::NodeStatus> action_result_{BT::NodeStatus::FAILURE};
  double target_position_ = 0.0;
  double step_size_ = 0.02;

  bool sendGoal();
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr& goal_handle);
  void resultCallback(const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::WrappedResult& result);
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr,
    const std::shared_ptr<const MoveElevatorActionType::Feedback> feedback);
};

class BackAwayFromTable : public BT::SyncActionNode, public RosNodeBT
{
public:
  BackAwayFromTable(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("distance", 0.3, "Distance to back away in meters"),
      BT::InputPort<double>("speed", 0.1, "Backing speed (m/s)")
    };
  }

  BT::NodeStatus tick() override;
};

class ComputeElevatorHeight : public BT::SyncActionNode, public RosNodeBT
{
public:
  ComputeElevatorHeight(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("canLocation"),
      BT::OutputPort<double>("targetHeight")
    };
  }
  BT::NodeStatus tick() override;
};

class RetractExtender : public BT::SyncActionNode, public RosNodeBT
{
public:
  RetractExtender(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class RetractGripper : public BT::SyncActionNode, public RosNodeBT
{
public:
  RetractGripper(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class OpenGripper : public BT::SyncActionNode, public RosNodeBT
{
public:
  OpenGripper(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class CloseGripperAroundCan : public BT::SyncActionNode, public RosNodeBT
{
public:
  CloseGripperAroundCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("canDiameter", 0.066, "Diameter of can in meters") };
  }
  BT::NodeStatus tick() override;
};

class ExtendTowardsCan : public BT::SyncActionNode, public RosNodeBT
{
public:
  ExtendTowardsCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

class AdjustExtenderToCenterCan : public BT::SyncActionNode, public RosNodeBT
{
public:
  AdjustExtenderToCenterCan(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("objectOfInterest") };
  }
  BT::NodeStatus tick() override;
};

// ============================================================================
// SETUP/UTILITY ACTION NODES
// ============================================================================

class SaveRobotPose : public BT::SyncActionNode, public RosNodeBT
{
public:
  SaveRobotPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<geometry_msgs::msg::PoseStamped>("saveTo") };
  }
  BT::NodeStatus tick() override;
};

class LoadCanLocation : public BT::SyncActionNode, public RosNodeBT
{
public:
  LoadCanLocation(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("canName"),
      BT::OutputPort<geometry_msgs::msg::Point>("location")
    };
  }
  BT::NodeStatus tick() override;
};

class ChargeBattery : public BT::SyncActionNode, public RosNodeBT
{
public:
  ChargeBattery(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class ShutdownSystem : public BT::SyncActionNode, public RosNodeBT
{
public:
  ShutdownSystem(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class SoftwareEStop : public BT::SyncActionNode, public RosNodeBT
{
public:
  SoftwareEStop(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("reason", "Emergency stop", "Reason for E-stop") };
  }
  BT::NodeStatus tick() override;
};

class WaitForDetection : public BT::StatefulActionNode, public RosNodeBT
{
public:
  WaitForDetection(const std::string & name, const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Time start_time_;
};

class ReportGraspFailure : public BT::SyncActionNode, public RosNodeBT
{
public:
  ReportGraspFailure(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class SaySomething : public BT::SyncActionNode, public RosNodeBT
{
public:
  SaySomething(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}
  
  static BT::PortsList providedPorts() {
    return { 
      BT::InputPort<std::string>("message", "", "Message to log"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Optional pose to include in message")
    };
  }
  BT::NodeStatus tick() override;
};

// ============================================================================
// CUSTOM DECORATOR
// ============================================================================

class ReactiveRepeatUntilSuccessOrCount : public BT::DecoratorNode
{
public:
  ReactiveRepeatUntilSuccessOrCount(const std::string & name, const BT::NodeConfiguration & config)
  : BT::DecoratorNode(name, config), current_cycle_(0) {}
  
  static BT::PortsList providedPorts() {
    return { BT::InputPort<int>("num_cycles") };
  }
  
  BT::NodeStatus tick() override;
  
  void halt() override {
    current_cycle_ = 0;
    BT::DecoratorNode::halt();
  }
  
private:
  int current_cycle_;
};

class CheckBoolFlag : public BT::ConditionNode
{
public:
  CheckBoolFlag(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("flag", "Boolean flag to check"),
      BT::InputPort<bool>("expected", "Expected boolean value")
    };
  }

  BT::NodeStatus tick() override;
};

}  // namespace can_do_challenge

#endif  // CAN_DO_CHALLENGE__BT_NODES_REAL_HPP_
