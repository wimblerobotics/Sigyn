// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "can_do_challenge/bt_nodes.hpp"
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;

namespace can_do_challenge
{

// ============================================================================
// SAFETY CONDITION NODES
// ============================================================================

BT::NodeStatus BatteryAboveChargingVoltage::tick()
{
  // Placeholder: Always return SUCCESS (battery is above charging voltage)
  // TODO: Subscribe to /sigyn/teensy_bridge/battery/status and check voltage
  RCLCPP_DEBUG(node_->get_logger(), "[BatteryAboveChargingVoltage] Checking battery");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus BatteryAboveCriticalVoltage::tick()
{
  // Placeholder: Always return SUCCESS (battery is above critical voltage)
  // TODO: Subscribe to battery state and check for critical level
  RCLCPP_DEBUG(node_->get_logger(), "[BatteryAboveCriticalVoltage] Checking battery");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RobotIsEstopped::tick()
{
  // Placeholder: Always return FAILURE (not estopped)
  // TODO: Subscribe to /sigyn/teensy_bridge/estop and check state
  RCLCPP_DEBUG(node_->get_logger(), "[RobotIsEstopped] Checking E-stop");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTiltedCritically::tick()
{
  // Placeholder: Always return FAILURE (not tilted critically)
  // TODO: Subscribe to IMU and check tilt angle
  RCLCPP_DEBUG(node_->get_logger(), "[RobotTiltedCritically] Checking tilt");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTiltedWarning::tick()
{
  // Placeholder: Always return FAILURE (no tilt warning)
  // TODO: Subscribe to IMU and check tilt angle
  RCLCPP_DEBUG(node_->get_logger(), "[RobotTiltedWarning] Checking tilt");
  return BT::NodeStatus::FAILURE;
}

// ============================================================================
// VISION CONDITION NODES
// ============================================================================

BT::NodeStatus CanDetectedByOAKD::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Return SUCCESS after 2 seconds to simulate detection
  // TODO: Subscribe to OAK-D object detection topic and check for can
  static auto start_time = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::steady_clock::now() - start_time).count();
  
  if (elapsed > 2) {
    RCLCPP_INFO(node_->get_logger(), "[CanDetectedByOAKD] Can '%s' detected by OAK-D", 
                object_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByOAKD] Searching for '%s' with OAK-D...", 
               object_name.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CanDetectedByPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Return SUCCESS to simulate detection
  // TODO: Subscribe to Pi Camera object detection topic
  RCLCPP_INFO(node_->get_logger(), "[CanDetectedByPiCamera] Can '%s' detected by Pi Camera", 
              object_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanCenteredInPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Return SUCCESS to simulate centering
  // TODO: Check if bounding box center is within threshold of image center
  RCLCPP_INFO(node_->get_logger(), "[CanCenteredInPiCamera] Can '%s' is centered", 
              object_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanWithinReach::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Return SUCCESS to simulate being within reach
  // TODO: Check distance from OAK-D depth or TF transform
  RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Can '%s' is within reach (~0.3m)", 
              object_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanIsGrasped::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Return SUCCESS to simulate grasp
  // TODO: Check gripper force sensor or verify can still visible in Pi Camera
  RCLCPP_INFO(node_->get_logger(), "[CanIsGrasped] Can '%s' is grasped", 
              object_name.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ElevatorAtHeight::tick()
{
  double target_height;
  getInput("targetHeight", target_height);
  
  // Placeholder: Return SUCCESS to simulate elevator at height
  // TODO: Subscribe to elevator position topic
  RCLCPP_INFO(node_->get_logger(), "[ElevatorAtHeight] Elevator at %.2fm", target_height);
  return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// NAVIGATION ACTION NODES
// ============================================================================

BT::NodeStatus ComputePathToCanLocation::tick()
{
  // Get the can location from input
  auto location = getInput<geometry_msgs::msg::Point>("location");
  if (!location) {
    RCLCPP_ERROR(node_->get_logger(), "[ComputePathToCanLocation] Failed to get location");
    return BT::NodeStatus::FAILURE;
  }
  
  // Placeholder: Create a goal pose near the can location
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = node_->now();
  
  // Stand 0.5m in front of table
  goal.pose.position.x = location.value().x;
  goal.pose.position.y = location.value().y - 0.5;
  goal.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, M_PI/2); // Face table
  goal.pose.orientation = tf2::toMsg(q);
  
  setOutput("goal", goal);
  
  RCLCPP_INFO(node_->get_logger(), 
    "[ComputePathToCanLocation] Computed goal pose at (%.2f, %.2f) facing can at (%.2f, %.2f, %.2f)",
    goal.pose.position.x, goal.pose.position.y,
    location.value().x, location.value().y, location.value().z);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePathToPose::tick()
{
  // Placeholder: Just pass through the goal
  // TODO: Call Nav2 ComputePathToPose action
  RCLCPP_INFO(node_->get_logger(), "[ComputePathToPose] Computing path");
  setOutput("error_code_id", 0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FollowPath::tick()
{
  // Placeholder: Just return success
  // TODO: Call Nav2 FollowPath action
  RCLCPP_INFO(node_->get_logger(), "[FollowPath] Following path");
  setOutput("error_code_id", 0);
  
  // Simulate navigation time
  rclcpp::sleep_for(2s);
  
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveTowardsCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Move closer to can
  // TODO: Use OAK-D depth info to compute movement vector
  RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Moving towards '%s'", 
              object_name.c_str());
  
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RotateRobot::tick()
{
  double degrees;
  getInput("degrees", degrees);
  
  // Placeholder: Rotate the robot
  // TODO: Publish twist command or use Nav2 rotate recovery
  RCLCPP_INFO(node_->get_logger(), "[RotateRobot] Rotating %.1f degrees", degrees);
  
  rclcpp::sleep_for(500ms);
  return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GRIPPER/ELEVATOR ACTION NODES
// ============================================================================

BT::NodeStatus LowerElevator::tick()
{
  // Placeholder: Lower elevator to minimum height
  // TODO: Publish to gripper control topic
  RCLCPP_INFO(node_->get_logger(), "[LowerElevator] Lowering elevator to minimum");
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorSafely::tick()
{
  // Placeholder: Lower elevator to safe travel height with can
  // TODO: Lower to just above base, not touching top plate
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorSafely] Lowering elevator to safe height");
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorToTable::tick()
{
  // Placeholder: Lower elevator to table surface height
  // TODO: Use known table height or visual feedback
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorToTable] Lowering to table surface");
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveElevatorToHeight::tick()
{
  double target_height;
  getInput("targetHeight", target_height);
  
  // Placeholder: Move elevator to specified height
  // TODO: Publish elevator position command
  RCLCPP_INFO(node_->get_logger(), "[MoveElevatorToHeight] Moving elevator to %.2fm", 
              target_height);
  rclcpp::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeElevatorHeight::tick()
{
  geometry_msgs::msg::Point can_location;
  getInput("canLocation", can_location);
  
  // Compute target height: can bottom Z + camera offset above gripper
  double target_height = can_location.z + 0.1; // 10cm above can for Pi Camera to see
  
  setOutput("targetHeight", target_height);
  
  RCLCPP_INFO(node_->get_logger(), "[ComputeElevatorHeight] Computed height: %.2fm", 
              target_height);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractExtender::tick()
{
  // Placeholder: Retract extender fully
  // TODO: Publish extender position command
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Retracting extender");
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractGripper::tick()
{
  // Placeholder: Same as RetractExtender
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Retracting gripper assembly");
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenGripper::tick()
{
  // Placeholder: Open gripper jaws
  // TODO: Publish gripper command
  RCLCPP_INFO(node_->get_logger(), "[OpenGripper] Opening gripper");
  rclcpp::sleep_for(500ms);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseGripperAroundCan::tick()
{
  double can_diameter;
  getInput("canDiameter", can_diameter);
  
  // Placeholder: Close gripper to diameter
  // TODO: Compute gripper position based on can diameter (66mm)
  RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] Closing to %.3fm diameter", 
              can_diameter);
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExtendTowardsCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Extend towards can
  // TODO: Use Pi Camera centroid to compute extension distance
  RCLCPP_INFO(node_->get_logger(), "[ExtendTowardsCan] Extending towards '%s'", 
              object_name.c_str());
  rclcpp::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AdjustExtenderToCenterCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Adjust extender left/right and forward/back
  // TODO: Use Pi Camera bounding box center vs image center to compute adjustment
  RCLCPP_DEBUG(node_->get_logger(), "[AdjustExtenderToCenterCan] Adjusting position");
  rclcpp::sleep_for(200ms);
  return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SETUP/UTILITY ACTION NODES
// ============================================================================

BT::NodeStatus SaveRobotPose::tick()
{
  // Placeholder: Get current robot pose
  // TODO: Get pose from AMCL or TF map->base_link
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = "map";
  current_pose.header.stamp = node_->now();
  current_pose.pose.position.x = 0.0;
  current_pose.pose.position.y = 0.0;
  current_pose.pose.position.z = 0.0;
  current_pose.pose.orientation.w = 1.0;
  
  setOutput("saveTo", current_pose);
  
  RCLCPP_INFO(node_->get_logger(), "[SaveRobotPose] Saved starting pose");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LoadCanLocation::tick()
{
  std::string can_name;
  getInput("canName", can_name);
  
  // Placeholder: Hardcoded location for CokeZeroCan
  // TODO: Implement proper JSON parsing when nlohmann-json is available
  geometry_msgs::msg::Point location;
  
  if (can_name == "CokeZeroCan") {
    location.x = 0.0;
    location.y = 20.0;
    location.z = 0.75;
    
    setOutput("location", location);
    
    RCLCPP_INFO(node_->get_logger(), 
                "[LoadCanLocation] Loaded location for '%s': (%.2f, %.2f, %.2f)",
                can_name.c_str(), location.x, location.y, location.z);
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_ERROR(node_->get_logger(), "[LoadCanLocation] Can '%s' not found in database", 
               can_name.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ChargeBattery::tick()
{
  // Placeholder: Initiate charging
  // TODO: Navigate to charging dock and initiate charging
  RCLCPP_INFO(node_->get_logger(), "[ChargeBattery] Charging battery");
  return BT::NodeStatus::RUNNING; // Would normally stay RUNNING until charged
}

BT::NodeStatus ShutdownSystem::tick()
{
  // Placeholder: Shutdown the system
  RCLCPP_FATAL(node_->get_logger(), "[ShutdownSystem] SYSTEM SHUTDOWN INITIATED");
  rclcpp::shutdown();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SoftwareEStop::tick()
{
  std::string reason;
  getInput("reason", reason);
  
  // Placeholder: Trigger software E-stop
  // TODO: Publish to E-stop topic
  RCLCPP_ERROR(node_->get_logger(), "[SoftwareEStop] E-STOP: %s", reason.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus WaitForDetection::tick()
{
  // Placeholder: Wait briefly
  RCLCPP_DEBUG(node_->get_logger(), "[WaitForDetection] Waiting for detection...");
  rclcpp::sleep_for(500ms);
  return BT::NodeStatus::FAILURE; // Force retry
}

BT::NodeStatus ReportGraspFailure::tick()
{
  // Placeholder: Report that grasp failed
  RCLCPP_ERROR(node_->get_logger(), "[ReportGraspFailure] GRASP FAILED - Can not detected");
  return BT::NodeStatus::FAILURE;
}

// ============================================================================
// CUSTOM DECORATOR
// ============================================================================

BT::NodeStatus ReactiveRepeat::tick()
{
  int num_cycles;
  getInput("num_cycles", num_cycles);
  
  if (current_cycle_ >= num_cycles) {
    current_cycle_ = 0;
    return BT::NodeStatus::FAILURE; // Completed all cycles
  }
  
  BT::NodeStatus child_status = child_node_->executeTick();
  
  if (child_status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }
  
  current_cycle_++;
  
  if (current_cycle_ < num_cycles) {
    return BT::NodeStatus::RUNNING; // Continue repeating
  } else {
    current_cycle_ = 0;
    return BT::NodeStatus::SUCCESS; // All cycles completed
  }
}

}  // namespace can_do_challenge
