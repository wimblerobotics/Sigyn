// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "can_do_challenge/bt_nodes.hpp"
#include "sigyn_interfaces/msg/e_stop_status.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace can_do_challenge
{

// Global sensor state (updated by subscribers)
struct SimulatedSensorState {
  double battery_voltage = 36.0;
  double roll_angle = 0.0;  // radians
  double pitch_angle = 0.0; // radians
  bool estop_triggered = false;
  double elevator_position = 0.0;  // meters
  double extender_position = 0.0;  // meters
  bool use_sim_time = false;
  
  rclcpp::Time last_battery_update;
  rclcpp::Time last_imu_update;
  rclcpp::Time last_estop_update;
  
  // Thresholds
  static constexpr double CHARGING_VOLTAGE = 35.0;  // Volts
  static constexpr double CRITICAL_VOLTAGE = 30.0;   // Volts
  static constexpr double WARNING_TILT = 20.0 * M_PI / 180.0;  // 20 degrees
  static constexpr double CRITICAL_TILT = 30.0 * M_PI / 180.0; // 30 degrees
};

static SimulatedSensorState g_sensor_state;
static std::mutex g_sensor_mutex;

// Global object detection state
struct ObjectDetectionState {
  // OAK-D camera detection
  geometry_msgs::msg::Point oakd_detection_position;  // In map frame
  bool oakd_can_detected = false;
  rclcpp::Time oakd_last_detection_time;
  
  // Pi camera (gripper) detection
  geometry_msgs::msg::Point pi_detection_position;  // In camera frame
  bool pi_can_detected = false;
  rclcpp::Time pi_last_detection_time;
  
  // Detection parameters
  static constexpr double DETECTION_TIMEOUT_SEC = 2.0;
  static constexpr double WITHIN_REACH_DISTANCE = 0.3;  // 30cm
  static constexpr double CENTERING_TOLERANCE = 0.05;   // 5cm in camera frame
};

static ObjectDetectionState g_detection_state;
static std::mutex g_detection_mutex;

// Store subscriptions so they don't get destroyed
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_oakd_sub;
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_pi_sub;

// Initialize object detection subscribers
static void initializeObjectDetection(std::shared_ptr<rclcpp::Node> node) {
  static bool initialized = false;
  if (initialized) return;
  
  RCLCPP_INFO(node->get_logger(), "Initializing object detection subscribers");
  
  // Subscribe to OAK-D detection
  g_oakd_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/oakd_top/can_detection", 10,
    [node](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      g_detection_state.oakd_detection_position = msg->point;
      g_detection_state.oakd_can_detected = true;
      g_detection_state.oakd_last_detection_time = node->now();
      RCLCPP_DEBUG(node->get_logger(), "OAK-D detection received: (%.2f, %.2f, %.2f)",
                   msg->point.x, msg->point.y, msg->point.z);
    });
  
  // Subscribe to Pi camera detection
  g_pi_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/gripper/can_detection", 10,
    [node](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      g_detection_state.pi_detection_position = msg->point;
      g_detection_state.pi_can_detected = true;
      g_detection_state.pi_last_detection_time = node->now();
      RCLCPP_DEBUG(node->get_logger(), "Pi camera detection received: (%.2f, %.2f, %.2f)",
                   msg->point.x, msg->point.y, msg->point.z);
    });
  
  initialized = true;
}

// Initialize subscribers for simulated sensors
static void initializeSimulatedSensors(std::shared_ptr<rclcpp::Node> node) {
  static bool initialized = false;
  if (initialized) return;
  
  // Check if we're in simulation (use_sim_time is already declared by rclcpp::Node)
  g_sensor_state.use_sim_time = node->get_parameter("use_sim_time").as_bool();
  
  if (!g_sensor_state.use_sim_time) {
    RCLCPP_INFO(node->get_logger(), "Real hardware mode - subscribing to real sensor topics");
    // TODO: Subscribe to real hardware topics
    initialized = true;
    return;
  }
  
  RCLCPP_INFO(node->get_logger(), "Simulation mode - subscribing to simulated sensor topics");
  
  // Subscribe to simulated battery
  auto battery_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/sigyn/teensy_bridge/battery/status", 10,
    [node](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      g_sensor_state.battery_voltage = msg->voltage;
      g_sensor_state.last_battery_update = node->now();
    });
  
  // Subscribe to simulated IMU
  auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/sigyn/teensy_bridge/imu/sensor_0", 10,
    [node](const sensor_msgs::msg::Imu::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      // Extract roll and pitch from quaternion
      double qx = msg->orientation.x;
      double qy = msg->orientation.y;
      double qz = msg->orientation.z;
      double qw = msg->orientation.w;
      
      // Roll (x-axis rotation)
      double sinr_cosp = 2 * (qw * qx + qy * qz);
      double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
      g_sensor_state.roll_angle = std::atan2(sinr_cosp, cosr_cosp);
      
      // Pitch (y-axis rotation)
      double sinp = 2 * (qw * qy - qz * qx);
      if (std::abs(sinp) >= 1)
        g_sensor_state.pitch_angle = std::copysign(M_PI / 2, sinp);
      else
        g_sensor_state.pitch_angle = std::asin(sinp);
      
      g_sensor_state.last_imu_update = node->now();
    });
  
  // Subscribe to simulated E-stop
  auto estop_sub = node->create_subscription<sigyn_interfaces::msg::EStopStatus>(
    "/sigyn/teensy_bridge/safety/estop_status", 10,
    [node](const sigyn_interfaces::msg::EStopStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      g_sensor_state.estop_triggered = msg->active;  // Field is 'active' not 'estop_triggered'
      g_sensor_state.last_estop_update = node->now();
    });
  
  // Subscribe to gripper commands to track simulated positions
  auto gripper_sub = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_gripper", 10,
    [](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      // Integrate velocities (very simple simulation)
      double dt = 0.1;  // Assume 10Hz updates
      g_sensor_state.elevator_position += msg->linear.x * dt;
      g_sensor_state.extender_position += msg->angular.z * dt;
      
      // Clamp to physical limits
      g_sensor_state.elevator_position = std::max(0.0, std::min(4.0, g_sensor_state.elevator_position));
      g_sensor_state.extender_position = std::max(0.0, std::min(0.5, g_sensor_state.extender_position));
    });
  
  initialized = true;
}

// ============================================================================
// SAFETY CONDITION NODES
// ============================================================================

BT::NodeStatus BatteryAboveChargingVoltage::tick()
{
  initializeSimulatedSensors(node_);
  
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  bool above_threshold = g_sensor_state.battery_voltage > SimulatedSensorState::CHARGING_VOLTAGE;
  
  RCLCPP_INFO(node_->get_logger(), "[BatteryAboveChargingVoltage] Voltage: %.2fV (threshold: %.2fV) -> %s",
    g_sensor_state.battery_voltage, SimulatedSensorState::CHARGING_VOLTAGE,
    above_threshold ? "SUCCESS" : "FAILURE");
    
  return above_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BatteryAboveCriticalVoltage::tick()
{
  initializeSimulatedSensors(node_);
  
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  bool above_threshold = g_sensor_state.battery_voltage > SimulatedSensorState::CRITICAL_VOLTAGE;
  
  RCLCPP_INFO(node_->get_logger(), "[BatteryAboveCriticalVoltage] Voltage: %.2fV (threshold: %.2fV) -> %s",
    g_sensor_state.battery_voltage, SimulatedSensorState::CRITICAL_VOLTAGE,
    above_threshold ? "SUCCESS" : "FAILURE");
  
  return above_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotIsEstopped::tick()
{
  initializeSimulatedSensors(node_);
  
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  
  if (g_sensor_state.estop_triggered) {
    RCLCPP_WARN(node_->get_logger(), "[RobotIsEstopped] E-STOP IS TRIGGERED!");
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[RobotIsEstopped] E-stop check: NOT TRIGGERED (SAFE)");
  // Returns SUCCESS if estopped, FAILURE if not (inverted logic for safety)
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTiltedCritically::tick()
{
  initializeSimulatedSensors(node_);
  
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  double max_tilt = std::max(std::abs(g_sensor_state.roll_angle), std::abs(g_sensor_state.pitch_angle));
  bool critical = max_tilt > SimulatedSensorState::CRITICAL_TILT;
  
  if (critical) {
    RCLCPP_ERROR(node_->get_logger(), "[RobotTiltedCritically] CRITICAL TILT! Roll: %.1f°, Pitch: %.1f°",
      g_sensor_state.roll_angle * 180.0 / M_PI,
      g_sensor_state.pitch_angle * 180.0 / M_PI);
  }
  
  return critical ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotTiltedWarning::tick()
{
  initializeSimulatedSensors(node_);
  
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  double max_tilt = std::max(std::abs(g_sensor_state.roll_angle), std::abs(g_sensor_state.pitch_angle));
  bool warning = max_tilt > SimulatedSensorState::WARNING_TILT;
  
  if (warning) {
    RCLCPP_WARN(node_->get_logger(), "[RobotTiltedWarning] Tilt warning! Roll: %.1f°, Pitch: %.1f°",
      g_sensor_state.roll_angle * 180.0 / M_PI,
      g_sensor_state.pitch_angle * 180.0 / M_PI);
  }
  
  return warning ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ============================================================================
// VISION CONDITION NODES
// ============================================================================

BT::NodeStatus CanDetectedByOAKD::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  // Check if we have a recent detection
  if (g_detection_state.oakd_can_detected) {
    auto time_since_detection = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
    
    if (time_since_detection < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
      RCLCPP_INFO(node_->get_logger(), "[CanDetectedByOAKD] Can '%s' detected at (%.2f, %.2f, %.2f)", 
                  object_name.c_str(),
                  g_detection_state.oakd_detection_position.x,
                  g_detection_state.oakd_detection_position.y,
                  g_detection_state.oakd_detection_position.z);
      return BT::NodeStatus::SUCCESS;
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByOAKD] Searching for '%s' with OAK-D...", 
               object_name.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CanDetectedByPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  if (g_detection_state.pi_can_detected) {
    auto time_since_detection = (node_->now() - g_detection_state.pi_last_detection_time).seconds();
    
    if (time_since_detection < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
      RCLCPP_INFO(node_->get_logger(), "[CanDetectedByPiCamera] Can '%s' detected", 
                  object_name.c_str());
      return BT::NodeStatus::SUCCESS;
    }
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByPiCamera] Waiting for Pi camera detection...");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CanCenteredInPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  if (!g_detection_state.pi_can_detected) {
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if can is centered (x and y should be near zero in camera frame)
  double offset_x = std::abs(g_detection_state.pi_detection_position.y);
  double offset_y = std::abs(g_detection_state.pi_detection_position.z);
  
  if (offset_x < ObjectDetectionState::CENTERING_TOLERANCE && 
      offset_y < ObjectDetectionState::CENTERING_TOLERANCE) {
    RCLCPP_INFO(node_->get_logger(), "[CanCenteredInPiCamera] Can '%s' is centered", 
                object_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_DEBUG(node_->get_logger(), "[CanCenteredInPiCamera] Can offset: (%.3f, %.3f)",
               offset_x, offset_y);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CanWithinReach::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  if (!g_detection_state.oakd_can_detected) {
    RCLCPP_DEBUG(node_->get_logger(), "[CanWithinReach] No detection available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Get robot's current position
  try {
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    auto transform = tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));
    
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    
    // Calculate distance from robot to can
    double dx = g_detection_state.oakd_detection_position.x - robot_x;
    double dy = g_detection_state.oakd_detection_position.y - robot_y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance < ObjectDetectionState::WITHIN_REACH_DISTANCE) {
      RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Can '%s' within reach at %.2fm", 
                  object_name.c_str(), distance);
      return BT::NodeStatus::SUCCESS;
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "[CanWithinReach] Can at %.2fm (need %.2fm)",
                 distance, ObjectDetectionState::WITHIN_REACH_DISTANCE);
    return BT::NodeStatus::FAILURE;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[CanWithinReach] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
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
  
  // Subscribe to joint_states if needed
  static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  static double current_elevator_position = 0.0;
  static std::mutex position_mutex;
  
  if (!joint_state_sub) {
    joint_state_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(position_mutex);
        // Find the elevator joint
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "gripper_elevator_plate_to_gripper_extender") {
            current_elevator_position = msg->position[i];
            break;
          }
        }
      });
  }
  
  // Check if elevator is at target height (within 5cm tolerance)
  double current_pos;
  {
    std::lock_guard<std::mutex> lock(position_mutex);
    current_pos = current_elevator_position;
  }
  
  double error = std::abs(current_pos - target_height);
  bool at_height = error < 0.05;  // 5cm tolerance
  
  if (at_height) {
    RCLCPP_INFO(node_->get_logger(), "[ElevatorAtHeight] Elevator at %.2fm (target: %.2fm)", 
                current_pos, target_height);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_DEBUG(node_->get_logger(), "[ElevatorAtHeight] Elevator at %.2fm, moving to %.2fm (error: %.3fm)",
                 current_pos, target_height, error);
    return BT::NodeStatus::FAILURE;
  }
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
  std::this_thread::sleep_for(2s);
  
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveTowardsCan::onStart()
{
  initializeObjectDetection(node_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveTowardsCan::onRunning()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Check detection first without mutex holding entire function
  bool detection_valid = false;
  geometry_msgs::msg::Point det_pos;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (g_detection_state.oakd_can_detected) {
      if ((node_->now() - g_detection_state.oakd_last_detection_time).seconds() < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
        detection_valid = true;
        det_pos = g_detection_state.oakd_detection_position;
      }
    }
  }
  
  if (!detection_valid) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Lost detection of '%s'", object_name.c_str());
    return BT::NodeStatus::FAILURE;
  }
  
  // Get robot's current position and orientation
  try {
    // DO NOT create buffer locally in loop - usage of static/member if possible, or just accept overhead for now
    static auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    static auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    // Check if within reach
    double angle_error = 0.0;
    double distance = 0.0;

    if (tf_buffer->canTransform("map", "base_link", tf2::TimePointZero)) {
      auto transform = tf_buffer->lookupTransform(
        "map", "base_link", tf2::TimePointZero);
        
      double robot_x = transform.transform.translation.x;
      double robot_y = transform.transform.translation.y;
      
      // Calculate angle to can
      double dx = det_pos.x - robot_x;
      double dy = det_pos.y - robot_y;
      
      distance = std::hypot(dx, dy);
      double target_yaw = std::atan2(dy, dx);
      
      // Get robot yaw
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      
      angle_error = target_yaw - yaw;
    } else {
        RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Waiting for TF...");
        return BT::NodeStatus::RUNNING;
    }
    
    // Check if we've reached the target
    if (distance < ObjectDetectionState::WITHIN_REACH_DISTANCE) {
      RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Reached target distance");
      return BT::NodeStatus::SUCCESS;
    }
    
    // Normalize angle to [-pi, pi]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    
    // Create twist command
    static auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
    geometry_msgs::msg::Twist cmd;
    
    // If angle error is large, rotate first
    if (std::abs(angle_error) > 0.1) {
      cmd.angular.z = std::copysign(std::min(0.5, std::abs(angle_error)), angle_error);
      RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Rotating %.2f rad towards can",
                  angle_error);
    } else {
      // Move forward slowly
      cmd.linear.x = std::min(0.1, distance * 0.5);
      RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Moving %.2fm forward towards '%s'",
                  cmd.linear.x, object_name.c_str());
    }
    
    cmd_vel_pub->publish(cmd);
    return BT::NodeStatus::RUNNING;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus RotateRobot::onStart()
{
  double degrees = 0.0;
  getInput("degrees", degrees);

  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
  }

  // Calculate duration
  double speed_rad_s = 0.5;
  double radians = std::abs(degrees * M_PI / 180.0);
  target_duration_ = radians / speed_rad_s;
  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(), "[RotateRobot] Rotating %.1f degrees (Target Duration: %.2fs)", degrees, target_duration_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateRobot::onRunning()
{
  double degrees = 0.0;
  getInput("degrees", degrees);
  double sign = (degrees >= 0) ? 1.0 : -1.0;
  double speed_rad_s = 0.5;

  auto elapsed = (node_->now() - start_time_).seconds();

  if (elapsed >= target_duration_) {
    // Stop the robot
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);
    return BT::NodeStatus::SUCCESS;
  }

  // Publish rotation command
  geometry_msgs::msg::Twist msg;
  msg.angular.z = speed_rad_s * sign;
  cmd_vel_pub_->publish(msg);

  return BT::NodeStatus::RUNNING;
}

void RotateRobot::onHalted()
{
  if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist stop_msg;
    cmd_vel_pub_->publish(stop_msg);
  }
  RCLCPP_INFO(node_->get_logger(), "[RotateRobot] Halted");
}

// ============================================================================
// ASYNC NAVIGATION ACTION
// ============================================================================

BT::NodeStatus NavigateToPoseAction::onStart()
{
  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "ROS node not set!");
    return BT::NodeStatus::FAILURE;
  }

  // Initialize action client if needed
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<NavigateAction>(
      node_->get_node_base_interface(), node_->get_node_graph_interface(),
      node_->get_node_logging_interface(), node_->get_node_waitables_interface(),
      "/navigate_to_pose");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Nav2 action server not available!");
      return BT::NodeStatus::FAILURE;
    }
  }

  // Get goal from input
  if (!getInput("goal", current_goal_)) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "No goal provided!");
    return BT::NodeStatus::FAILURE;
  }

  getInput("behavior_tree", current_behavior_tree_);
  
  goal_start_time_ = std::chrono::steady_clock::now();
  action_state_ = ActionState::IDLE;
  result_received_ = false;
  navigation_result_ = BT::NodeStatus::FAILURE;

  // Send goal
  if (sendGoal()) {
    action_state_ = ActionState::SENDING_GOAL;
    RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), 
                "Sent navigation goal to (%.2f, %.2f)",
                current_goal_.pose.position.x, current_goal_.pose.position.y);
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Failed to send navigation goal");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  // Handle different action states
  switch (action_state_) {
    case ActionState::SENDING_GOAL:
      // Check if goal was accepted/rejected (non-blocking)
      if (goal_handle_future_.valid() &&
          goal_handle_future_.wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
        goal_handle_ = goal_handle_future_.get();
        if (!goal_handle_) {
          RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Goal was rejected!");
          action_state_ = ActionState::GOAL_FAILED;
          setOutput("error_code_id", -1);
          return BT::NodeStatus::FAILURE;
        }
        action_state_ = ActionState::GOAL_ACTIVE;
        RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Goal accepted, navigation active");
      }
      return BT::NodeStatus::RUNNING;

    case ActionState::GOAL_ACTIVE:
      // Check result (non-blocking)
      if (result_received_.load()) {
        BT::NodeStatus result = navigation_result_.load();
        if (result == BT::NodeStatus::SUCCESS) {
          RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Navigation succeeded!");
          action_state_ = ActionState::GOAL_COMPLETED;
          setOutput("error_code_id", 0);
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("NavigateToPoseAction"), "Navigation failed!");
          action_state_ = ActionState::GOAL_FAILED;
          setOutput("error_code_id", -1);
          return BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::RUNNING;

    case ActionState::GOAL_COMPLETED:
      setOutput("error_code_id", 0);
      return BT::NodeStatus::SUCCESS;

    case ActionState::GOAL_FAILED:
      setOutput("error_code_id", -1);
      return BT::NodeStatus::FAILURE;

    default:
      return BT::NodeStatus::RUNNING;
  }
}

void NavigateToPoseAction::onHalted()
{
  RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Navigation halted - canceling goal");
  
  if (goal_handle_ && action_state_ == ActionState::GOAL_ACTIVE) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
  }
  
  action_state_ = ActionState::IDLE;
}

bool NavigateToPoseAction::sendGoal()
{
  if (!action_client_) {
    return false;
  }

  auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
  goal_msg.pose = current_goal_;
  goal_msg.behavior_tree = current_behavior_tree_;

  auto send_goal_options = rclcpp_action::Client<NavigateAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigateToPoseAction::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&NavigateToPoseAction::resultCallback, this, std::placeholders::_1);

  result_received_ = false;
  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  return true;
}

void NavigateToPoseAction::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<NavigateAction>::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Goal was rejected by server");
  }
}

void NavigateToPoseAction::resultCallback(
  const rclcpp_action::ClientGoalHandle<NavigateAction>::WrappedResult& result)
{
  result_received_ = true;
  
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      navigation_result_ = BT::NodeStatus::SUCCESS;
      RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Navigation goal succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      navigation_result_ = BT::NodeStatus::FAILURE;
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Navigation goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      navigation_result_ = BT::NodeStatus::FAILURE;
      RCLCPP_WARN(rclcpp::get_logger("NavigateToPoseAction"), "Navigation goal was canceled");
      break;
    default:
      navigation_result_ = BT::NodeStatus::FAILURE;
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Unknown result code");
      break;
  }
}

// ============================================================================
// GRIPPER/ELEVATOR ACTION NODES
// ============================================================================

BT::NodeStatus LowerElevator::tick()
{
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }
  
  // Lower to 0.0m (minimum)
  auto msg = std_msgs::msg::Float64();
  msg.data = 0.0;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevator] Lowering elevator to 0.0m minimum");
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorSafely::tick()
{
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }
  
  // Lower to 0.1m safe height
  auto msg = std_msgs::msg::Float64();
  msg.data = 0.1;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorSafely] Lowering elevator to 0.1m safe height");
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorToTable::tick()
{
  // Get target height from blackboard (set by ComputeElevatorHeight)
  double target_height = 0.67095;  // Default to can height
  auto height_input = getInput<double>("targetHeight");
  if (height_input) {
    target_height = height_input.value() - 0.02;  // Lower 2cm below can for approach
  }
  
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }
  
  // Publish position command
  auto msg = std_msgs::msg::Float64();
  msg.data = target_height;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorToTable] Lowering to %.5fm table surface height", target_height);
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveElevatorToHeight::tick()
{
  double target_height;
  getInput("targetHeight", target_height);
  
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }
  
  // Publish position command
  auto msg = std_msgs::msg::Float64();
  msg.data = target_height;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[MoveElevatorToHeight] Moving elevator to %.5fm", 
              target_height);
  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeElevatorHeight::tick()
{
  geometry_msgs::msg::Point can_location;
  getInput("canLocation", can_location);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  double target_height;
  
  if (g_detection_state.oakd_can_detected) {
    // Use detected can height plus gripper offset
    double can_height = g_detection_state.oakd_detection_position.z;
    double gripper_offset = 0.05; // 5cm above can
    target_height = can_height + gripper_offset;
    
    RCLCPP_INFO(node_->get_logger(), "[ComputeElevatorHeight] Can at %.3fm, setting elevator to %.3fm",
                can_height, target_height);
  } else {
    // Fallback to default height
    target_height = 0.67095;
    RCLCPP_WARN(node_->get_logger(), "[ComputeElevatorHeight] No can detected, using default height %.3fm",
                target_height);
  }
  
  setOutput("targetHeight", target_height);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractExtender::tick()
{
  // Placeholder: Retract extender fully
  // TODO: Publish extender position command
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Retracting extender");
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractGripper::tick()
{
  // Placeholder: Same as RetractExtender
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Retracting gripper assembly");
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenGripper::tick()
{
  // Placeholder: Open gripper jaws
  // TODO: Publish gripper command
  RCLCPP_INFO(node_->get_logger(), "[OpenGripper] Opening gripper");
  std::this_thread::sleep_for(500ms);
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
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExtendTowardsCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  if (!g_detection_state.pi_can_detected) {
    RCLCPP_WARN(node_->get_logger(), "[ExtendTowardsCan] No Pi camera detection available");
    return BT::NodeStatus::FAILURE;
  }
  
  // Distance forward to can is the x coordinate in camera frame
  double distance_to_can = g_detection_state.pi_detection_position.x;
  
  // Leave some margin (don't extend all the way)
  double extension_distance = std::max(0.0, distance_to_can - 0.03); // Stop 3cm before
  
  RCLCPP_INFO(node_->get_logger(), 
              "[ExtendTowardsCan] Extending gripper %.3fm toward '%s' (detected at %.3fm)",
              extension_distance, object_name.c_str(), distance_to_can);
  
  // TODO: Publish to gripper extension controller
  // For now, just simulate extension time
  std::this_thread::sleep_for(1s);
  
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AdjustExtenderToCenterCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  // Placeholder: Adjust extender left/right and forward/back
  // TODO: Use Pi Camera bounding box center vs image center to compute adjustment
  RCLCPP_DEBUG(node_->get_logger(), "[AdjustExtenderToCenterCan] Adjusting position");
  std::this_thread::sleep_for(200ms);
  return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SETUP/UTILITY ACTION NODES
// ============================================================================

BT::NodeStatus SaveRobotPose::tick()
{
  // Get current robot pose from TF (map -> base_link)
  geometry_msgs::msg::PoseStamped current_pose;
  current_pose.header.frame_id = "map";
  current_pose.header.stamp = node_->now();
  
  try {
    // Create TF buffer and listener if needed
    static std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    static std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    
    if (!tf_buffer) {
      RCLCPP_INFO(node_->get_logger(), "[SaveRobotPose] Initializing TF buffer and listener...");
      tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
      // Give TF and AMCL time to populate transforms
      RCLCPP_INFO(node_->get_logger(), "[SaveRobotPose] Waiting 2 seconds for TF to populate...");
      std::this_thread::sleep_for(2000ms);
    }
    
    // Lookup transform from map to base_link (use latest available)
    RCLCPP_INFO(node_->get_logger(), "[SaveRobotPose] Looking up transform from map to base_link...");
    geometry_msgs::msg::TransformStamped transform;
    transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(5.0));
    
    // Convert transform to pose
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.position.z = transform.transform.translation.z;
    current_pose.pose.orientation = transform.transform.rotation;
    
    setOutput("saveTo", current_pose);
    
    RCLCPP_INFO(node_->get_logger(), "[SaveRobotPose] *** SAVED STARTING POSE: (%.2f, %.2f, yaw=%.2f) ***",
                current_pose.pose.position.x, current_pose.pose.position.y, 
                atan2(2.0 * (current_pose.pose.orientation.w * current_pose.pose.orientation.z), 
                      1.0 - 2.0 * current_pose.pose.orientation.z * current_pose.pose.orientation.z));
    return BT::NodeStatus::SUCCESS;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node_->get_logger(), "[SaveRobotPose] TF lookup failed: %s", ex.what());
    RCLCPP_ERROR(node_->get_logger(), "[SaveRobotPose] This likely means AMCL hasn't published the initial pose yet!");
    
    // Fallback to origin
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.w = 1.0;
    
    setOutput("saveTo", current_pose);
    
    RCLCPP_WARN(node_->get_logger(), "[SaveRobotPose] Using fallback origin pose (0, 0, 0)");
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus LoadCanLocation::tick()
{
  std::string can_name;
  getInput("canName", can_name);
  
  // Placeholder: Hardcoded location for CokeZeroCan
  // TODO: Implement proper JSON parsing when nlohmann-json is available
  geometry_msgs::msg::Point location;
  
  if (can_name == "CokeZeroCan") {
    location.x = 8.43;
    location.y = 11.2;
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
  std::this_thread::sleep_for(500ms);
  return BT::NodeStatus::FAILURE; // Force retry
}

BT::NodeStatus ReportGraspFailure::tick()
{
  // Placeholder: Report that grasp failed
  RCLCPP_ERROR(node_->get_logger(), "[ReportGraspFailure] GRASP FAILED - Can not detected");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus SaySomething::tick()
{
  std::string message;
  getInput("message", message);
  
  // Check if pose is provided
  auto pose_input = getInput<geometry_msgs::msg::PoseStamped>("pose");
  
  if (pose_input) {
    const auto& pose = pose_input.value();
    double yaw = atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z), 
                       1.0 - 2.0 * pose.pose.orientation.z * pose.pose.orientation.z);
    RCLCPP_INFO(node_->get_logger(), "[SaySomething] %s (%.2f, %.2f, yaw=%.2f)", 
                message.c_str(), pose.pose.position.x, pose.pose.position.y, yaw);
  } else {
    RCLCPP_INFO(node_->get_logger(), "[SaySomething] %s", message.c_str());
  }
  
  return BT::NodeStatus::SUCCESS;
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

// ============================================================================
// CHECK BOOL FLAG CONDITION
// ============================================================================

BT::NodeStatus CheckBoolFlag::tick()
{
  bool flag = false;
  bool expected = false;
  
  if (!getInput("flag", flag)) {
    // If we can't read the flag (e.g. key doesn't exist), assume false
    flag = false;
  }
  
  if (!getInput("expected", expected)) {
    // If expected not specified, default to true? Or false? 
    // Let's assume user must specify it, or default to checking for true.
    expected = true; 
  }
  
  if (flag == expected) {
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

}  // namespace can_do_challenge
