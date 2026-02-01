// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "can_do_challenge/bt_nodes.hpp"
#include "sigyn_interfaces/msg/e_stop_status.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/header.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <thread>
#include <regex>
#include <condition_variable>

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
  double last_extender_command = 0.0;  // meters
  bool use_sim_time = false;
  
  rclcpp::Time last_battery_update;
  rclcpp::Time last_imu_update;
  rclcpp::Time last_estop_update;
  rclcpp::Time last_joint_state_update;
  
  // Thresholds
  static constexpr double CHARGING_VOLTAGE = 35.0;  // Volts
  static constexpr double CRITICAL_VOLTAGE = 30.0;   // Volts
  static constexpr double WARNING_TILT = 20.0 * M_PI / 180.0;  // 20 degrees
  static constexpr double CRITICAL_TILT = 30.0 * M_PI / 180.0; // 30 degrees
};

// Physical robot constants
// On the real robot, the elevator "home" (zero) position is technically at the lower optical stop,
// which is mounted ~130mm (0.13m) above the physical bottom. 
// Sim URDF treats 0.15m as the lower limit relative to the "bottom".
// We subtract this offset from the commanded joint value so that:
//   Real Height = (Command + Offset)
//   Command = DesiredWorldHeight - BaseHeight - Offset
static constexpr double kElevatorHomingOffset = 0.13;

static SimulatedSensorState g_sensor_state;
static std::mutex g_sensor_mutex;

// Global object detection state
struct ObjectDetectionState {
  // OAK-D camera detection
  // Raw point as published by the detector (typically camera optical frame)
  geometry_msgs::msg::Point oakd_detection_position_raw;
  std::string oakd_detection_frame_raw;

  // Anchored point in a fixed frame (odom) to avoid "carrot on a stick" issues when rotating
  geometry_msgs::msg::Point oakd_detection_position_odom;
  bool oakd_detection_has_odom = false;
  bool oakd_can_detected = false;
  rclcpp::Time oakd_last_detection_time;
  
  // Pi camera (gripper) detection
  geometry_msgs::msg::Point pi_detection_position;
  std::string pi_detection_frame = "map";
  rclcpp::Time pi_detection_stamp;
  bool pi_can_detected = false;
  rclcpp::Time pi_last_detection_time;
  
  // Detection parameters
  static constexpr double DETECTION_TIMEOUT_SEC = 2.0;
  static constexpr double PI_DETECTION_TIMEOUT_SEC = 6.0;
  static constexpr int PI_WAIT_TIMEOUT_MS = 1000;
  static constexpr double WITHIN_REACH_DISTANCE = 0.55;  // 55cm
  static constexpr double CENTERING_TOLERANCE = 0.02;   // Tightened to 2cm for precision
  static constexpr double PI_VERTICAL_TOLERANCE = 0.05; // 5cm for elevator height check
  static constexpr double PI_VERTICAL_TARGET_OFFSET = 0.0; // target offset in optical Y (positive down)
  static constexpr double PI_MIN_DISTANCE_M = 0.15;
  static constexpr double PI_MAX_DISTANCE_M = 0.80;
};

static ObjectDetectionState g_detection_state;
static std::mutex g_detection_mutex;
static std::condition_variable g_pi_detection_cv;

static std::mutex g_pi_frame_mutex;
static std::condition_variable g_pi_frame_cv;
static rclcpp::Time g_pi_last_frame_time;

static bool getFreshPiDetection(const std::shared_ptr<rclcpp::Node>& node,
                                geometry_msgs::msg::Point& out_point,
                                double& out_age_sec)
{
  static int missing_count = 0;
  static int stale_count = 0;
  std::unique_lock<std::mutex> lock(g_detection_mutex);
  const auto last_seen = g_detection_state.pi_last_detection_time;

  g_pi_detection_cv.wait_for(
    lock,
    std::chrono::milliseconds(ObjectDetectionState::PI_WAIT_TIMEOUT_MS),
    [&]() {
      return g_detection_state.pi_can_detected &&
             g_detection_state.pi_last_detection_time.nanoseconds() != last_seen.nanoseconds();
    });

  if (!g_detection_state.pi_can_detected) {
    out_age_sec = 999.0;
    missing_count++;
    if (missing_count % 10 == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "[PiDetection] No detection available yet (count=%d)",
                  missing_count);
    }
    return false;
  }

  out_age_sec = (node->now() - g_detection_state.pi_last_detection_time).seconds();
  if (out_age_sec >= ObjectDetectionState::PI_DETECTION_TIMEOUT_SEC) {
    stale_count++;
    if (stale_count % 10 == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "[PiDetection] Detection stale age=%.2fs (count=%d)",
                  out_age_sec, stale_count);
    }
    return false;
  }

  out_point = g_detection_state.pi_detection_position;
  return true;
}

static bool piDetectionInRange(const geometry_msgs::msg::Point& point)
{
  return point.z >= ObjectDetectionState::PI_MIN_DISTANCE_M &&
         point.z <= ObjectDetectionState::PI_MAX_DISTANCE_M;
}

// Store subscriptions so they don't get destroyed
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_oakd_sub;
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_pi_sub;
static rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr g_pi_processed_sub;
static std::shared_ptr<tf2_ros::Buffer> g_tf_buffer;
static std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;

// Debug publishers (optional, for diagnosing approach issues)
struct DebugTelemetry {
  bool enabled = true;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr approach_goal_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr can_in_base_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr can_in_map_pub;
};

static DebugTelemetry g_debug;
static std::mutex g_debug_mutex;

static void initializeDebugTelemetry(const std::shared_ptr<rclcpp::Node>& node)
{
  static bool initialized = false;
  if (initialized) {
    return;
  }

  // Keep this always on for now; user explicitly asked for instrumentation.
  // If we later want to gate it, add a parameter here.
  g_debug.enabled = true;

  rclcpp::QoS qos(10);
  qos.best_effort();

  g_debug.status_pub = node->create_publisher<std_msgs::msg::String>(
    "/can_do_challenge/debug/status", qos);
  g_debug.approach_goal_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/can_do_challenge/debug/approach_goal", qos);
  g_debug.can_in_base_pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/can_do_challenge/debug/can_in_base_link", qos);
  g_debug.can_in_map_pub = node->create_publisher<geometry_msgs::msg::PointStamped>(
    "/can_do_challenge/debug/can_in_map", qos);

  initialized = true;
}

static void publishDebugStatus(const std::shared_ptr<rclcpp::Node>& node, const std::string& s)
{
  initializeDebugTelemetry(node);
  std::lock_guard<std::mutex> lock(g_debug_mutex);
  if (!g_debug.enabled || !g_debug.status_pub) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = s;
  g_debug.status_pub->publish(msg);
}

static void publishDebugPoint(const std::shared_ptr<rclcpp::Node>& node,
                              const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr& pub,
                              const std::string& frame,
                              const geometry_msgs::msg::Point& p)
{
  initializeDebugTelemetry(node);
  std::lock_guard<std::mutex> lock(g_debug_mutex);
  if (!g_debug.enabled || !pub) {
    return;
  }
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = frame;
  msg.header.stamp = node->now();
  msg.point = p;
  pub->publish(msg);
}

static void publishDebugPose(const std::shared_ptr<rclcpp::Node>& node,
                             const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
                             const geometry_msgs::msg::PoseStamped& pose)
{
  initializeDebugTelemetry(node);
  std::lock_guard<std::mutex> lock(g_debug_mutex);
  if (!g_debug.enabled || !pub) {
    return;
  }
  pub->publish(pose);
}

// Initialize object detection subscribers
static void initializeObjectDetection(std::shared_ptr<rclcpp::Node> node) {
  static bool initialized = false;
  if (initialized) return;
  
  RCLCPP_INFO(node->get_logger(), "Initializing object detection subscribers");
  
  // Global TF listener
  g_tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  g_tf_listener = std::make_shared<tf2_ros::TransformListener>(*g_tf_buffer);

  // Subscribe to OAK-D detection
  rclcpp::QoS oakd_qos(10);
  oakd_qos.best_effort();
  g_oakd_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/oakd_top/can_detection", oakd_qos,
    [node](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      // Always store the raw detection
      {
        std::lock_guard<std::mutex> lock(g_detection_mutex);
        g_detection_state.oakd_detection_position_raw = msg->point;
        g_detection_state.oakd_detection_frame_raw = msg->header.frame_id;
        g_detection_state.oakd_can_detected = true;
        g_detection_state.oakd_last_detection_time = node->now();
      }

      // Logging reduced
      // RCLCPP_INFO(node->get_logger(),
      //             "[OAKDDetection] Received detection frame=%s stamp=%.3f x=%.3f y=%.3f z=%.3f",
      //             msg->header.frame_id.c_str(),
      //             rclcpp::Time(msg->header.stamp).seconds(),
      //             msg->point.x, msg->point.y, msg->point.z);

      // Best-effort: also transform to ODOM frame immediately
      // This anchors the observation in the world, preventing it from moving with the camera
      geometry_msgs::msg::PointStamped detection_odom;
      try {
        geometry_msgs::msg::PointStamped msg_latest = *msg;
        msg_latest.header.stamp = rclcpp::Time(0);
        detection_odom = g_tf_buffer->transform(msg_latest, "odom");

        std::lock_guard<std::mutex> lock(g_detection_mutex);
        g_detection_state.oakd_detection_position_odom = detection_odom.point;
        g_detection_state.oakd_detection_has_odom = true;
        RCLCPP_DEBUG(node->get_logger(), "OAK-D detection updated in ODOM: (%.2f, %.2f)",
                    detection_odom.point.x, detection_odom.point.y);
      } catch (const tf2::TransformException & ex) {
          std::lock_guard<std::mutex> lock(g_detection_mutex);
          g_detection_state.oakd_detection_has_odom = false;
      }
    });
  RCLCPP_INFO(node->get_logger(), "Subscribed to /oakd_top/can_detection (best_effort)");
  
  // Subscribe to Pi camera detection
  rclcpp::QoS pi_qos(10);
  pi_qos.best_effort();

  g_pi_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "/gripper/can_detection", pi_qos,
    [node](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(g_detection_mutex);
        g_detection_state.pi_detection_position = msg->point;
        g_detection_state.pi_detection_frame = msg->header.frame_id;
        g_detection_state.pi_detection_stamp = rclcpp::Time(msg->header.stamp);
        g_detection_state.pi_can_detected = true;
        g_detection_state.pi_last_detection_time = node->now();
      }
      g_pi_detection_cv.notify_all();
      // Logging reduced to reduce noise
      // RCLCPP_INFO(node->get_logger(), "[PiDetection] Received detection frame=%s stamp=%.3f x=%.3f y=%.3f z=%.3f",
      //             msg->header.frame_id.c_str(),
      //             rclcpp::Time(msg->header.stamp).seconds(),
      //             msg->point.x, msg->point.y, msg->point.z);
    });
  RCLCPP_INFO(node->get_logger(), "Subscribed to /gripper/can_detection (best_effort)");

  // Subscribe to Pi detector processed-frame heartbeat
  g_pi_processed_sub = node->create_subscription<std_msgs::msg::Header>(
    "/gripper/can_detection/processed", pi_qos,
    [node](const std_msgs::msg::Header::SharedPtr msg) {
      {
        std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
        g_pi_last_frame_time = rclcpp::Time(msg->stamp);
      }
      g_pi_frame_cv.notify_all();
      // Logging reduced to reduce noise
      // RCLCPP_INFO(node->get_logger(), "[PiDetection] Processed heartbeat frame=%s stamp=%.3f",
      //             msg->frame_id.c_str(), rclcpp::Time(msg->stamp).seconds());
      (void)node;
    });
  RCLCPP_INFO(node->get_logger(), "Subscribed to /gripper/can_detection/processed (best_effort)");
  
  initialized = true;
}

// Initialize subscribers for simulated sensors
static void initializeSimulatedSensors(std::shared_ptr<rclcpp::Node> node) {
  static bool initialized = false;
  static rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub;
  static rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  static rclcpp::Subscription<sigyn_interfaces::msg::EStopStatus>::SharedPtr estop_sub;
  static rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gripper_sub;
  static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
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
  battery_sub = node->create_subscription<sensor_msgs::msg::BatteryState>(
    "/sigyn/teensy_bridge/battery/status", 10,
    [node](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      g_sensor_state.battery_voltage = msg->voltage;
      g_sensor_state.last_battery_update = node->now();
    });
  
  // Subscribe to simulated IMU
  imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
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
  estop_sub = node->create_subscription<sigyn_interfaces::msg::EStopStatus>(
    "/sigyn/teensy_bridge/safety/estop_status", 10,
    [node](const sigyn_interfaces::msg::EStopStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      g_sensor_state.estop_triggered = msg->active;  // Field is 'active' not 'estop_triggered'
      g_sensor_state.last_estop_update = node->now();
    });
  
  // Subscribe to gripper commands to track simulated positions
  gripper_sub = node->create_subscription<geometry_msgs::msg::Twist>(
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

  joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [node](const sensor_msgs::msg::JointState::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        const auto &name = msg->name[i];
        const double position = msg->position[i];
        if (name == "gripper_elevator_plate_to_gripper_extender") {
          g_sensor_state.extender_position = position;
        } else if (name == "elevator_pole_to_elevator_connector_plate") {
          g_sensor_state.elevator_position = position;
        }
      }
      g_sensor_state.last_joint_state_update = node->now();
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
  
  if (!g_detection_state.oakd_can_detected) {
    RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByOAKD] Searching for '%s' with OAK-D...",
                 object_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto time_since_detection = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
  if (time_since_detection >= ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
    RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByOAKD] Detection stale (%.2fs)", time_since_detection);
    publishDebugStatus(node_, "CanDetectedByOAKD: stale age=" + std::to_string(time_since_detection));
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[CanDetectedByOAKD] Can '%s' detected (raw frame='%s' raw=(%.2f,%.2f,%.2f) age=%.2fs)",
              object_name.c_str(),
              g_detection_state.oakd_detection_frame_raw.c_str(),
              g_detection_state.oakd_detection_position_raw.x,
              g_detection_state.oakd_detection_position_raw.y,
              g_detection_state.oakd_detection_position_raw.z,
              time_since_detection);

  publishDebugStatus(node_,
    "CanDetectedByOAKD: ok frame=" + g_detection_state.oakd_detection_frame_raw +
    " raw=(" + std::to_string(g_detection_state.oakd_detection_position_raw.x) + "," +
              std::to_string(g_detection_state.oakd_detection_position_raw.y) + "," +
              std::to_string(g_detection_state.oakd_detection_position_raw.z) + ")" +
    " age=" + std::to_string(time_since_detection));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanDetectedByPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);

  geometry_msgs::msg::Point det_point;
  double age_sec = 999.0;
  if (!getFreshPiDetection(node_, det_point, age_sec)) {
    RCLCPP_INFO(node_->get_logger(), "[CanDetectedByPiCamera] Waiting for Pi camera detection...");
    return BT::NodeStatus::FAILURE;
  }

  if (!piDetectionInRange(det_point)) {
    RCLCPP_INFO(node_->get_logger(),
                "[CanDetectedByPiCamera] Detection out of range z=%.2f (allowed %.2f-%.2f)",
                det_point.z, ObjectDetectionState::PI_MIN_DISTANCE_M,
                ObjectDetectionState::PI_MAX_DISTANCE_M);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "[CanDetectedByPiCamera] Can '%s' detected (age=%.2fs)",
              object_name.c_str(), age_sec);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CanCenteredInPiCamera::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);

  geometry_msgs::msg::Point det_point;
  double age_sec = 999.0;
  if (!getFreshPiDetection(node_, det_point, age_sec)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!piDetectionInRange(det_point)) {
    RCLCPP_INFO(node_->get_logger(),
                "[CanCenteredInPiCamera] Detection out of range z=%.2f (allowed %.2f-%.2f)",
                det_point.z, ObjectDetectionState::PI_MIN_DISTANCE_M,
                ObjectDetectionState::PI_MAX_DISTANCE_M);
    return BT::NodeStatus::FAILURE;
  }

  // Check if can is centered HORIZONTALLY only (X axis in camera frame).
  // Pi camera detector emits in Optical Frame (X=Right, Y=Down, Z=Forward).
  // We only care about horizontal (X) centering - vertical (Y) is handled by ElevatorAtHeight.
  const double offset_x = std::abs(det_point.x);
  
  if (offset_x < ObjectDetectionState::CENTERING_TOLERANCE) {
    RCLCPP_INFO(node_->get_logger(), "[CanCenteredInPiCamera] SUCCESS: Can '%s' check passed. x_offset=%.4fm < tolerance=%.4fm (age=%.2fs)", 
                object_name.c_str(), offset_x, ObjectDetectionState::CENTERING_TOLERANCE, age_sec);
    return BT::NodeStatus::SUCCESS;
  }
  
  RCLCPP_INFO(node_->get_logger(), "[CanCenteredInPiCamera] FAILURE: Can not centered. x_offset=%.4fm >= tolerance=%.4fm",
              offset_x, ObjectDetectionState::CENTERING_TOLERANCE);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CanWithinReach::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  // Snapshot under lock
  geometry_msgs::msg::Point det_raw;
  std::string det_frame;
  bool has_raw = false;
  bool has_odom = false;
  geometry_msgs::msg::Point det_odom;
  double age_sec = 999.0;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (!g_detection_state.oakd_can_detected) {
      RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] No detection available");
      return BT::NodeStatus::FAILURE;
    }
    age_sec = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
    if (age_sec >= ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
      RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Detection stale (%.2fs)", age_sec);
      return BT::NodeStatus::FAILURE;
    }
    has_raw = !g_detection_state.oakd_detection_frame_raw.empty();
    det_raw = g_detection_state.oakd_detection_position_raw;
    det_frame = g_detection_state.oakd_detection_frame_raw;
    has_odom = g_detection_state.oakd_detection_has_odom;
    det_odom = g_detection_state.oakd_detection_position_odom;
  }

  if (!g_tf_buffer) {
    RCLCPP_WARN(node_->get_logger(), "[CanWithinReach] TF buffer not ready");
    return BT::NodeStatus::FAILURE;
  }

  // Prefer base_link distance from raw detection (tracks what the robot can actually reach).
  if (has_raw) {
    try {
      geometry_msgs::msg::PointStamped det_msg;
      det_msg.header.frame_id = det_frame;
      det_msg.header.stamp = rclcpp::Time(0);
      det_msg.point = det_raw;

      auto det_in_base = g_tf_buffer->transform(det_msg, "base_link");
      const double dx = det_in_base.point.x;
      const double dy = det_in_base.point.y;
      const double distance = std::hypot(dx, dy);

      publishDebugPoint(node_, g_debug.can_in_base_pub, "base_link", det_in_base.point);
      publishDebugStatus(node_, "CanWithinReach: base dx=" + std::to_string(dx) +
                  " dy=" + std::to_string(dy) +
                  " dist=" + std::to_string(distance) +
                  " age=" + std::to_string(age_sec));

      if (dx > 0.0 && distance < ObjectDetectionState::WITHIN_REACH_DISTANCE) {
        RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Can '%s' within reach at %.2fm (age=%.2fs)",
                    object_name.c_str(), distance, age_sec);
        return BT::NodeStatus::SUCCESS;
      }

      RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] base dx=%.2f dy=%.2f dist=%.2f (need %.2f)",
          dx, dy, distance, ObjectDetectionState::WITHIN_REACH_DISTANCE);
      return BT::NodeStatus::FAILURE;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "[CanWithinReach] TF base_link<-det_raw failed: %s", ex.what());
      // fall through to odom fallback
    }
  }

  // Fallback: anchored odom distance.
  if (!has_odom) {
    RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] No odom-anchored detection available yet");
    return BT::NodeStatus::FAILURE;
  }

  try {
    auto tf_odom_base = g_tf_buffer->lookupTransform(
      "odom", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));

    const double robot_x = tf_odom_base.transform.translation.x;
    const double robot_y = tf_odom_base.transform.translation.y;

    const double dx = det_odom.x - robot_x;
    const double dy = det_odom.y - robot_y;
    const double distance = std::hypot(dx, dy);

    publishDebugStatus(node_, "CanWithinReach: odom dist=" + std::to_string(distance) +
                  " age=" + std::to_string(age_sec));

    if (distance < ObjectDetectionState::WITHIN_REACH_DISTANCE) {
      RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] (odom) Can '%s' within reach at %.2fm", 
                  object_name.c_str(), distance);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[CanWithinReach] TF odom<-base_link failed: %s", ex.what());
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

BT::NodeStatus WaitForNewPiFrameProcessed::tick()
{
  initializeObjectDetection(node_);

  auto current_time = node_->now();
  
  // On first tick or after reset, initialize state
  if (!waiting_) {
    wait_start_time_ = current_time;
    std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
    last_frame_time_ = g_pi_last_frame_time;
    waiting_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] Waiting for next processed frame (last=%.3f)",
                last_frame_time_.seconds());
  }
  
  // Check if we've exceeded timeout
  auto elapsed = (current_time - wait_start_time_).seconds();
  if (elapsed > (ObjectDetectionState::PI_WAIT_TIMEOUT_MS / 1000.0)) {
    RCLCPP_INFO(node_->get_logger(), "[WaitForNewPiFrameProcessed] Timeout after %.1fs waiting for processed frame",
                elapsed);
    waiting_ = false;  // Reset for next attempt
    return BT::NodeStatus::FAILURE;
  }
  
  // Check if new frame has been processed (non-blocking)
  // The heartbeat is published after object detection runs, regardless of whether can was found
  rclcpp::Time current_frame_time;
  {
    std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
    current_frame_time = g_pi_last_frame_time;
  }
  
  if (current_frame_time.nanoseconds() != last_frame_time_.nanoseconds() && current_frame_time.nanoseconds() > 0) {
    // New frame processed! Bounding box data (if any) is now available. Reset state and return success.
    waiting_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] New processed frame at %.3f (prev %.3f)",
                current_frame_time.seconds(), last_frame_time_.seconds());
    return BT::NodeStatus::SUCCESS;
  }
  
  // Still waiting for frame to be processed
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ElevatorAtHeight::tick()
{
  initializeObjectDetection(node_);

  static int tick_count = 0;
  tick_count++;

  geometry_msgs::msg::Point det_point;
  double age_sec = 999.0;
  if (!getFreshPiDetection(node_, det_point, age_sec)) {
    if (tick_count % 10 == 0) {
      RCLCPP_INFO(node_->get_logger(),
                  "[ElevatorAtHeight] No fresh Pi camera detection (age=%.2fs)", age_sec);
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!piDetectionInRange(det_point)) {
    RCLCPP_INFO(node_->get_logger(),
                "[ElevatorAtHeight] Detection out of range x=%.3f y=%.3f z=%.3f age=%.2fs (allowed z %.2f-%.2f)",
                det_point.x, det_point.y, det_point.z, age_sec,
                ObjectDetectionState::PI_MIN_DISTANCE_M,
                ObjectDetectionState::PI_MAX_DISTANCE_M);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
              "[ElevatorAtHeight] Pi detection raw x=%.3f y=%.3f z=%.3f age=%.2fs",
              det_point.x, det_point.y, det_point.z, age_sec);

  // Transform detection to base_link to get real 3D Z-height
  if (!g_tf_buffer) {
    RCLCPP_WARN(node_->get_logger(), "[ElevatorAtHeight] TF buffer not ready");
    return BT::NodeStatus::FAILURE;
  }

  try {
    // Get can position in base_link frame
    geometry_msgs::msg::PointStamped det_stamped;
    std::string detection_frame;
    rclcpp::Time detection_stamp;
    {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      detection_frame = g_detection_state.pi_detection_frame;
      detection_stamp = g_detection_state.pi_detection_stamp;
      det_stamped.header.frame_id = detection_frame;
      det_stamped.header.stamp = detection_stamp;
      det_stamped.point = det_point;
    }

    RCLCPP_INFO(node_->get_logger(),
          "[ElevatorAtHeight] Using detection frame '%s' stamp=%.3f for transform",
          detection_frame.c_str(), detection_stamp.seconds());

    if (detection_frame.empty() || detection_frame == "map") {
      RCLCPP_INFO(node_->get_logger(),
                  "[ElevatorAtHeight] Warning: detection frame is '%s' (expected camera optical frame)",
                  detection_frame.c_str());
    }
    
    geometry_msgs::msg::PointStamped can_in_base;
    try {
      can_in_base = g_tf_buffer->transform(det_stamped, "base_link", tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "[ElevatorAtHeight] TF transform at stamp=%.3f failed: %s. Falling back to latest TF.",
                  detection_stamp.seconds(), ex.what());
      det_stamped.header.stamp = rclcpp::Time(0);
      can_in_base = g_tf_buffer->transform(det_stamped, "base_link", tf2::durationFromSec(0.1));
    }
    RCLCPP_INFO(node_->get_logger(),
          "[ElevatorAtHeight] Can in base_link x=%.3f y=%.3f z=%.3f",
          can_in_base.point.x, can_in_base.point.y, can_in_base.point.z);
    
    // Get gripper plate position in base_link frame
    auto gripper_tf = g_tf_buffer->lookupTransform("base_link", "parallel_gripper_base_plate", 
                            rclcpp::Time(0), tf2::durationFromSec(0.1));
    double gripper_z = gripper_tf.transform.translation.z;
    double can_z = can_in_base.point.z;
    
    // Check if Z-heights match (tolerance of 0.015m since we step 0.02m at a time)
    // Tighter tolerance forces the elevator to climb to the closest step.
    const double z_error = std::abs(can_z - gripper_z);
    // Use 0.015m tolerance (1.5cm). Since step is 2cm, this ensures we align within <1 step.
    const bool at_height = z_error <= 0.018; 

    RCLCPP_INFO(node_->get_logger(),
          "[ElevatorAtHeight] Gripper Z=%.3f Can Z=%.3f | z_error=%.3f (tol=0.018)",
          gripper_z, can_z, z_error);

    if (at_height) {
      RCLCPP_INFO(node_->get_logger(),
            "[ElevatorAtHeight] Can Z-height matched! can_z=%.3fm gripper_z=%.3fm error=%.3fm (age=%.2fs)",
            can_z, gripper_z, z_error, age_sec);
      return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[ElevatorAtHeight] Can Z-height mismatch: can_z=%.3fm gripper_z=%.3fm error=%.3fm",
                can_z, gripper_z, z_error);
    return BT::NodeStatus::FAILURE;
    
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[ElevatorAtHeight] TF transform failed: %s", ex.what());
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
  
  const double standoff = ObjectDetectionState::WITHIN_REACH_DISTANCE;
  // Stand in front of table by standoff distance
  goal.pose.position.x = location.value().x;
  goal.pose.position.y = location.value().y - standoff;
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

static BT::NodeStatus computeApproachGoalToCanOnce(
  const std::shared_ptr<rclcpp::Node>& node,
  geometry_msgs::msg::PoseStamped& out_goal_in_map)
{
  // Use the node instance passed by the BT wrapper.
  initializeObjectDetection(node);
  initializeDebugTelemetry(node);

  // Snapshot detection
  geometry_msgs::msg::Point det_raw;
  std::string det_frame;
  bool has_raw = false;
  bool has_odom = false;
  geometry_msgs::msg::Point det_odom;
  double age_sec = 999.0;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (g_detection_state.oakd_can_detected) {
      age_sec = (node->now() - g_detection_state.oakd_last_detection_time).seconds();
      if (age_sec < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
        has_raw = !g_detection_state.oakd_detection_frame_raw.empty();
        det_raw = g_detection_state.oakd_detection_position_raw;
        det_frame = g_detection_state.oakd_detection_frame_raw;
        has_odom = g_detection_state.oakd_detection_has_odom;
        det_odom = g_detection_state.oakd_detection_position_odom;
      }
    }
  }

  if (!has_raw && !has_odom) {
    publishDebugStatus(node, "ComputeApproachGoalToCan: no detection (waiting)");
    return BT::NodeStatus::RUNNING;
  }

  // Require "fresh" detections for forward progress. If we don't have a current view, don't send Nav2 forward.
  constexpr double kFreshDetectionForGoalSec = 0.35;
  if (age_sec > kFreshDetectionForGoalSec) {
    publishDebugStatus(node, "ComputeApproachGoalToCan: stale age=" + std::to_string(age_sec) + " (waiting)");
    return BT::NodeStatus::RUNNING;
  }

  if (!g_tf_buffer) {
    publishDebugStatus(node, "ComputeApproachGoalToCan: TF buffer not ready (waiting)");
    return BT::NodeStatus::RUNNING;
  }

  // 1) Transform detection into base_link to get a reliable bearing.
  //    This is more robust than projecting a noisy range estimate into map.
  geometry_msgs::msg::PointStamped det_msg;
  if (has_raw && !det_frame.empty()) {
    det_msg.header.frame_id = det_frame;
    det_msg.header.stamp = rclcpp::Time(0);
    det_msg.point = det_raw;
  } else if (has_odom) {
    det_msg.header.frame_id = "odom";
    det_msg.header.stamp = rclcpp::Time(0);
    det_msg.point = det_odom;
  } else {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PointStamped det_in_base;
  try {
    const auto tf_base_from_det = g_tf_buffer->lookupTransform(
      "base_link", det_msg.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(det_msg, det_in_base, tf_base_from_det);
  } catch (const tf2::TransformException& ex) {
    publishDebugStatus(node, std::string("ComputeApproachGoalToCan: TF base_link<-det failed (waiting): ") + ex.what());
    return BT::NodeStatus::RUNNING;
  }

  // base_link: x forward, y left
  double dx_base = det_in_base.point.x;
  double dy_base = det_in_base.point.y;

  // If we have the raw optical x (right-positive) and it disagrees with base_link y sign, flip y.
  // This is a defensive guard against sim camera frame axis wiring issues.
  if (has_raw) {
    const bool raw_says_right = det_raw.x > 0.0;
    const bool base_says_left = dy_base > 0.0;
    if (raw_says_right == base_says_left) {
      dy_base = -dy_base;
    }
  }

  const double bearing = std::atan2(dy_base, dx_base);

  publishDebugPoint(node, g_debug.can_in_base_pub, "base_link", det_in_base.point);
  publishDebugStatus(node,
    "ComputeApproachGoalToCan: age=" + std::to_string(age_sec) +
    " base(dx=" + std::to_string(dx_base) + " dy=" + std::to_string(dy_base) +
    " bearing=" + std::to_string(bearing) + ")");

  // 2) Build a goal in base_link: rotate first; translate only when roughly facing the can.
  const double standoff = ObjectDetectionState::WITHIN_REACH_DISTANCE;
  constexpr double kMaxStep = 0.50;             // meters
  constexpr double kTranslateWhenAbsBearing = 0.35;  // rad (~20 deg)

  // Forward component (if range estimate is bad, at least direction stays correct).
  double forward_advance = 0.0;
  if (std::abs(bearing) <= kTranslateWhenAbsBearing && dx_base > 0.0) {
    forward_advance = std::clamp(dx_base - standoff, 0.0, kMaxStep);
  }

  geometry_msgs::msg::PoseStamped goal_in_base;
  goal_in_base.header.frame_id = "base_link";
  // Stamp is irrelevant here; we use TimePointZero when transforming.
  goal_in_base.header.stamp = rclcpp::Time(0);
  goal_in_base.pose.position.x = forward_advance;
  goal_in_base.pose.position.y = 0.0;
  goal_in_base.pose.position.z = 0.0;

  tf2::Quaternion q_goal;
  q_goal.setRPY(0.0, 0.0, bearing);
  goal_in_base.pose.orientation = tf2::toMsg(q_goal);

  // 3) Transform that goal into map for Nav2.
  geometry_msgs::msg::PoseStamped goal_in_map;
  try {
    const auto tf_map_from_base = g_tf_buffer->lookupTransform(
      "map", "base_link", tf2::TimePointZero);
    tf2::doTransform(goal_in_base, goal_in_map, tf_map_from_base);
    goal_in_map.header.frame_id = "map";
    goal_in_map.header.stamp = node->now();
  } catch (const tf2::TransformException& ex) {
    publishDebugStatus(node, std::string("ComputeApproachGoalToCan: TF map<-base_link failed (waiting): ") + ex.what());
    return BT::NodeStatus::RUNNING;
  }

  // Also publish a best-effort can point in map for visualization.
  try {
    const auto tf_map_from_det = g_tf_buffer->lookupTransform(
      "map", det_msg.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped det_in_map;
    tf2::doTransform(det_msg, det_in_map, tf_map_from_det);
    publishDebugPoint(node, g_debug.can_in_map_pub, "map", det_in_map.point);
  } catch (const tf2::TransformException&) {
    // ignore
  }

  publishDebugPose(node, g_debug.approach_goal_pub, goal_in_map);

  out_goal_in_map = goal_in_map;
  RCLCPP_INFO(node->get_logger(),
              "[ComputeApproachGoalToCan] base dx=%.2f dy=%.2f bearing=%.2f -> step=%.2f (map goal %.2f,%.2f)",
              dx_base, dy_base, bearing, forward_advance,
              goal_in_map.pose.position.x, goal_in_map.pose.position.y);

  publishDebugStatus(node,
    "ComputeApproachGoalToCan: publish goal map(" + std::to_string(goal_in_map.pose.position.x) + "," +
                                                std::to_string(goal_in_map.pose.position.y) + ") step=" +
    std::to_string(forward_advance));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeApproachGoalToCan::onStart()
{
  geometry_msgs::msg::PoseStamped goal;
  const auto status = computeApproachGoalToCanOnce(node_, goal);
  if (status == BT::NodeStatus::SUCCESS) {
    setOutput("goal", goal);
  }
  return status;
}

BT::NodeStatus ComputeApproachGoalToCan::onRunning()
{
  geometry_msgs::msg::PoseStamped goal;
  const auto status = computeApproachGoalToCanOnce(node_, goal);
  if (status == BT::NodeStatus::SUCCESS) {
    setOutput("goal", goal);
  }
  return status;
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

  // Pull the most recent raw detection snapshot
  bool detection_valid = false;
  geometry_msgs::msg::Point det_pos_raw;
  std::string det_frame_raw;
  double detection_age_sec = 999.0;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (g_detection_state.oakd_can_detected) {
      detection_age_sec = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
      if (detection_age_sec < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
        detection_valid = true;
        det_pos_raw = g_detection_state.oakd_detection_position_raw;
        det_frame_raw = g_detection_state.oakd_detection_frame_raw;
      }
    }
  }

  if (!detection_valid) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Lost detection of '%s'", object_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Safety: never drive forward on stale detections.
  constexpr double kFreshDetectionForForwardSec = 0.25;
  const bool detection_fresh_for_forward = detection_age_sec <= kFreshDetectionForForwardSec;

  try {
    geometry_msgs::msg::PointStamped det_point_raw;
    det_point_raw.point = det_pos_raw;
    det_point_raw.header.frame_id = det_frame_raw;
    det_point_raw.header.stamp = rclcpp::Time(0);

    if (!g_tf_buffer || !g_tf_buffer->canTransform("base_link", det_frame_raw, rclcpp::Time(0))) {
      RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Waiting for TF from '%s' to base_link", det_frame_raw.c_str());
      return BT::NodeStatus::RUNNING;
    }

    auto det_in_base = g_tf_buffer->transform(det_point_raw, "base_link");

    // base_link: x forward, y left
    double dx = det_in_base.point.x;
    double dy = det_in_base.point.y;

    // Heuristic sanity: if detector says "right of center" (raw x > 0 in optical), but TF says dy>0 (left), flip dy.
    // This guards against a swapped axis in the sim camera frame definition.
    if (det_pos_raw.x > 0.0 && dy > 0.0) {
      dy = -dy;
    } else if (det_pos_raw.x < 0.0 && dy < 0.0) {
      dy = -dy;
    }

    const double distance = std::hypot(dx, dy);
    const double angle_error = std::atan2(dy, dx);

    // Safety: don't rotate in-place too close to the table/can. Back up first to create clearance.
    constexpr double kRotateClearanceDistance = 0.45;  // meters
    constexpr double kBackUpSpeed = -0.08;             // m/s

    if (distance < ObjectDetectionState::WITHIN_REACH_DISTANCE) {
      RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Reached target distance");
      return BT::NodeStatus::SUCCESS;
    }

    static auto cmd_vel_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
    geometry_msgs::msg::Twist cmd;

    // If target isn't in front, rotate only.
    if (dx <= 0.05) {
      if (distance < kRotateClearanceDistance) {
        if (!detection_fresh_for_forward) {
          geometry_msgs::msg::Twist stop;
          cmd_vel_pub->publish(stop);
          RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Detection stale (%.2fs); refusing to move", detection_age_sec);
          return BT::NodeStatus::FAILURE;
        }
        cmd.linear.x = kBackUpSpeed;
        cmd.angular.z = 0.0;
        cmd_vel_pub->publish(cmd);
        return BT::NodeStatus::RUNNING;
      }

      cmd.angular.z = std::copysign(0.5, angle_error);
      cmd.linear.x = 0.0;
      cmd_vel_pub->publish(cmd);
      return BT::NodeStatus::RUNNING;
    }

    // Rotate towards target until roughly centered.
    if (std::abs(angle_error) > 0.1) {
      if (distance < kRotateClearanceDistance) {
        if (!detection_fresh_for_forward) {
          geometry_msgs::msg::Twist stop;
          cmd_vel_pub->publish(stop);
          RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Detection stale (%.2fs); refusing to move", detection_age_sec);
          return BT::NodeStatus::FAILURE;
        }
        cmd.linear.x = kBackUpSpeed;
        cmd.angular.z = 0.0;
        cmd_vel_pub->publish(cmd);
        return BT::NodeStatus::RUNNING;
      }

      cmd.angular.z = std::copysign(std::min(0.5, std::abs(angle_error)), angle_error);
      cmd.linear.x = 0.0;
      cmd_vel_pub->publish(cmd);
      return BT::NodeStatus::RUNNING;
    }

    // Only move forward if the can is still being detected (fresh).
    if (!detection_fresh_for_forward) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub->publish(stop);
      RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Detection stale (%.2fs); refusing to drive forward", detection_age_sec);
      return BT::NodeStatus::FAILURE;
    }

    cmd.linear.x = std::min(0.15, distance * 0.5);
    cmd.angular.z = angle_error;
    cmd_vel_pub->publish(cmd);
    return BT::NodeStatus::RUNNING;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] TF transform failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

// ----------------------------------------------------------------------------
// WaitForNewOAKDFrame (Simulation Version)
// ----------------------------------------------------------------------------
BT::NodeStatus WaitForNewOAKDFrame::onStart()
{
  start_wait_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForNewOAKDFrame::onRunning()
{
  // In simulation, we assume a fixed delay is sufficient to "wait for a new frame"
  // since tracking frame ID/timestamp is harder without consistent heartbeats.
  const double elapsed = (node_->now() - start_wait_time_).seconds();
  if (elapsed > 0.5) {
      return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SleepSeconds::onStart()
{
  double seconds = 1.0;
  getInput("seconds", seconds);
  wait_seconds_ = std::max(0.0, seconds);
  start_time_ = node_->now();
  RCLCPP_INFO(node_->get_logger(), "[SleepSeconds] Waiting %.2f seconds", wait_seconds_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SleepSeconds::onRunning()
{
  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed >= wait_seconds_) {
    return BT::NodeStatus::SUCCESS;
  }
  std::this_thread::sleep_for(50ms);
  return BT::NodeStatus::RUNNING;
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

  initializeDebugTelemetry(node_);

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
    publishDebugStatus(node_, "NavigateToPoseAction: FAIL no goal provided");
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
    publishDebugStatus(node_,
      "NavigateToPoseAction: sent goal frame=" + current_goal_.header.frame_id +
      " (" + std::to_string(current_goal_.pose.position.x) + "," +
             std::to_string(current_goal_.pose.position.y) + ")");
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("NavigateToPoseAction"), "Failed to send navigation goal");
    publishDebugStatus(node_, "NavigateToPoseAction: FAIL sendGoal");
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
          publishDebugStatus(node_, "NavigateToPoseAction: rejected");
          action_state_ = ActionState::GOAL_FAILED;
          setOutput("error_code_id", -1);
          return BT::NodeStatus::FAILURE;
        }
        action_state_ = ActionState::GOAL_ACTIVE;
        RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Goal accepted, navigation active");
        publishDebugStatus(node_, "NavigateToPoseAction: accepted/active");
      }
      return BT::NodeStatus::RUNNING;

    case ActionState::GOAL_ACTIVE:
      // Check result (non-blocking)
      if (result_received_.load()) {
        BT::NodeStatus result = navigation_result_.load();
        if (result == BT::NodeStatus::SUCCESS) {
          RCLCPP_INFO(rclcpp::get_logger("NavigateToPoseAction"), "Navigation succeeded!");
          publishDebugStatus(node_, "NavigateToPoseAction: SUCCEEDED");
          action_state_ = ActionState::GOAL_COMPLETED;
          setOutput("error_code_id", 0);
          return BT::NodeStatus::SUCCESS;
        } else {
          RCLCPP_WARN(rclcpp::get_logger("NavigateToPoseAction"), "Navigation failed!");
          publishDebugStatus(node_, "NavigateToPoseAction: FAILED");
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
  if (node_) {
    publishDebugStatus(node_, "NavigateToPoseAction: HALTED cancel request");
  }
  
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

  // Note: this callback isn't on the BT thread; keep it minimal.
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
      "/elevator_connector_plate/position", 10);
  }
  
  // Lower to 0.0m (minimum)
  auto msg = std_msgs::msg::Float64();
  // Respect the URDF lower limit (0.15m).
  msg.data = 0.15;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevator] Lowering elevator to 0.15m minimum");
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorSafely::tick()
{
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/elevator_connector_plate/position", 10);
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
  initializeSimulatedSensors(node_);

  // Get target height from blackboard (set by ComputeElevatorHeight)
  double target_height = 0.67095;  // Default to can height
  auto height_input = getInput<double>("targetHeight");
  if (height_input) {
    target_height = height_input.value() - 0.02;  // Lower 2cm below can for approach
  }

  // NOTE: target_height is in world Z (odom). The elevator controller topic expects
  // a joint position relative to the robot, and base_link is not on the floor.
  // Convert world Z -> joint position by adding the base_link height in odom.
  double base_link_z = 0.0;
  if (g_tf_buffer) {
    try {
      const auto tf_odom_from_base = g_tf_buffer->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);
      base_link_z = tf_odom_from_base.transform.translation.z;
    } catch (const tf2::TransformException&) {
      // best-effort; leave base_link_z as 0
    }
  }
  // Subtract homing offset to match real robot kinematics
  // BUT: In simulation, the URDF might not model this offset (zero is zero).
  double homing_offset = kElevatorHomingOffset;
  {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      if (g_sensor_state.use_sim_time) {
          homing_offset = 0.0;
      }
  }

  const double commanded_joint = target_height + base_link_z - homing_offset;
  
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/elevator_connector_plate/position", 10);
  }
  
  // Publish position command
  auto msg = std_msgs::msg::Float64();
  msg.data = commanded_joint;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(),
              "[LowerElevatorToTable] Lowering to world_z=%.5fm (base_link_z=%.3fm -> joint=%.5fm)",
              target_height, base_link_z, commanded_joint);
  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveElevatorToHeight::tick()
{
  initializeSimulatedSensors(node_);

  double target_height;
  getInput("targetHeight", target_height);

    // When raising for centering/approach, add extra clearance.
    // In simulation, use a slightly smaller clearance to reduce overshoot.
    double kExtraTableClearance = 0.0;  // ###meters
    {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      if (g_sensor_state.use_sim_time) {
        kExtraTableClearance = 0.18;
      }
    }

  // Convert world Z -> joint position by adding base_link height in odom.
  double base_link_z = 0.0;
  if (g_tf_buffer) {
    try {
      const auto tf_odom_from_base = g_tf_buffer->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);
      base_link_z = tf_odom_from_base.transform.translation.z;
    } catch (const tf2::TransformException&) {
      // best-effort; leave base_link_z as 0
    }
  }
  // Subtract homing offset to match real robot kinematics
  // BUT: In simulation, the URDF might not model this offset (zero is zero).
  // If we subtract it in sim, we end up 13cm too low and crash into the table.
  double homing_offset = kElevatorHomingOffset;
  {
      std::lock_guard<std::mutex> lock(g_sensor_mutex);
      if (g_sensor_state.use_sim_time) {
          homing_offset = 0.0;
      }
  }
  
  const double commanded_joint = target_height + base_link_z + kExtraTableClearance - homing_offset;
  
  // Create publisher if needed
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/elevator_connector_plate/position", 10);
  }
  
  // Publish position command
  auto msg = std_msgs::msg::Float64();
  msg.data = commanded_joint;
  elevator_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(),
              "[MoveElevatorToHeight] Moving elevator to world_z=%.5fm (base_link_z=%.3fm + clearance=%.2fm -> joint=%.5fm)",
              target_height, base_link_z, kExtraTableClearance, commanded_joint);
  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StepElevatorUp::onStart()
{
  initializeSimulatedSensors(node_);

  double step_m = 0.01;
  getInput("stepMeters", step_m);
  const double max_travel_m = 0.91;
  const int max_steps = 400;

  static rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
  static double current_elevator_position = 0.0;
  static std::mutex position_mutex;
  static bool joint_state_seen = false;

  if (!joint_state_sub) {
    joint_state_sub = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(position_mutex);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "elevator_pole_to_elevator_connector_plate") {
            current_elevator_position = msg->position[i];
            joint_state_seen = true;
            break;
          }
        }
      });
  }

  double current_pos = 0.0;
  {
    std::lock_guard<std::mutex> lock(position_mutex);
    if (joint_state_seen) {
      current_pos = current_elevator_position;
    } else {
      current_pos = 0.15;
    }
  }

  if (!home_position_set_) {
    home_position_ = std::max(current_pos, 0.15);
    home_position_set_ = true;
    step_count_ = 0;
    last_commanded_ = home_position_;
  }

  if (step_count_ >= max_steps) {
    RCLCPP_WARN(node_->get_logger(), "[StepElevatorUp] Max steps reached (%d)", max_steps);
    return BT::NodeStatus::FAILURE;
  }

  const double max_pos = home_position_ + max_travel_m;
  const double base_pos = std::max(current_pos, last_commanded_);
  const double next_pos = std::min(base_pos + step_m, max_pos);

  if (next_pos <= last_commanded_ + 1e-4) {
    RCLCPP_WARN(node_->get_logger(), "[StepElevatorUp] Reached max travel (%.3f)", max_pos);
    return BT::NodeStatus::FAILURE;
  }

  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr elevator_pub;
  if (!elevator_pub) {
    elevator_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/elevator_connector_plate/position", 10);
  }

  std_msgs::msg::Float64 msg;
  msg.data = next_pos;
  elevator_pub->publish(msg);

  last_commanded_ = next_pos;
  step_count_++;
  RCLCPP_INFO(node_->get_logger(),
              "[StepElevatorUp] Step %d: joint %.3f -> %.3f (max %.3f)",
              step_count_, base_pos, next_pos, max_pos);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StepElevatorUp::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void StepElevatorUp::onHalted()
{
  home_position_set_ = false;
  step_count_ = 0;
  last_commanded_ = 0.0;
}

BT::NodeStatus BackAwayFromTable::tick()
{
  double distance = 0.3;
  double speed = 0.1;
  getInput("distance", distance);
  getInput("speed", speed);

  if (speed <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "[BackAwayFromTable] Invalid speed %.3f", speed);
    return BT::NodeStatus::FAILURE;
  }

  const double duration_sec = std::abs(distance / speed);

  auto cmd_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -std::abs(speed);
  cmd_pub->publish(cmd);

  RCLCPP_INFO(node_->get_logger(), "[BackAwayFromTable] Backing away %.2fm at %.2fm/s",
              distance, speed);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));

  geometry_msgs::msg::Twist stop;
  cmd_pub->publish(stop);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputeElevatorHeight::tick()
{
  geometry_msgs::msg::Point can_location;
  getInput("canLocation", can_location);
  
  initializeSimulatedSensors(node_);
  initializeObjectDetection(node_);
  
  std::lock_guard<std::mutex> lock(g_detection_mutex);
  
  static bool latched_height_valid = false;
  static double latched_height = 0.0;
  static rclcpp::Time latched_time;

  const auto now = node_->now();
  if (latched_height_valid && (now - latched_time).seconds() < 5.0) {
    setOutput("targetHeight", latched_height);
    RCLCPP_INFO(node_->get_logger(),
                "[ComputeElevatorHeight] Using latched height %.3fm",
                latched_height);
    return BT::NodeStatus::SUCCESS;
  }

  double target_height;
  const double oakd_age = (now - g_detection_state.oakd_last_detection_time).seconds();
  const bool oakd_fresh = g_detection_state.oakd_can_detected &&
                          g_detection_state.oakd_detection_has_odom &&
                          oakd_age < ObjectDetectionState::DETECTION_TIMEOUT_SEC;
  
  if (oakd_fresh) {
    // Use detected can height plus gripper offset
    // In odom frame, Z is height.
    double can_height = g_detection_state.oakd_detection_position_odom.z;
    
    // Gripper offset: 0.0 means we target the can't detected height (e.g. centroid).
    // Previous value (-0.18) was causing the elevator to go too low (into the table).
    double gripper_offset = 0.0; 
    target_height = can_height + gripper_offset;
    
    latched_height = target_height;
    latched_time = now;
    latched_height_valid = true;

    RCLCPP_INFO(node_->get_logger(),
                "[ComputeElevatorHeight] Can at %.3fm, setting elevator to %.3fm",
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
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Retracting extender");

  initializeSimulatedSensors(node_);

  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr extender_pub;
  if (!extender_pub) {
    extender_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }

  std_msgs::msg::Float64 msg;
  double current_position = g_sensor_state.extender_position;
  if (current_position < 0.001 && g_sensor_state.last_extender_command > 0.01) {
    RCLCPP_INFO(node_->get_logger(),
                "[RetractExtender] Using last command %.3f as current position", 
                g_sensor_state.last_extender_command);
    current_position = g_sensor_state.last_extender_command;
  }
  const double tug_target = std::max(0.0, current_position - 0.02);

  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Tug step: current=%.3f -> target=%.3f", current_position, tug_target);
  msg.data = tug_target;
  extender_pub->publish(msg);
  std::this_thread::sleep_for(500ms);

  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Full retract to 0.0");
  msg.data = 0.0;
  extender_pub->publish(msg);

  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractGripper::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Retracting gripper assembly");

  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr extender_pub;
  if (!extender_pub) {
    extender_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }

  std_msgs::msg::Float64 msg;
  msg.data = 0.0;
  extender_pub->publish(msg);

  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenGripper::tick()
{
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_finger_pub;
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_finger_pub;
  
  if (!left_finger_pub) {
    RCLCPP_INFO(node_->get_logger(), "[OpenGripper] Creating publishers...");
    left_finger_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/parallel_gripper_base_plate_to_left_finger/position", 10);
    right_finger_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/parallel_gripper_base_plate_to_right_finger/position", 10);
    std::this_thread::sleep_for(100ms);
  }

  std_msgs::msg::Float64 msg;
  msg.data = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), "[OpenGripper] Opening fingers (0.0)");
  left_finger_pub->publish(msg);
  right_finger_pub->publish(msg);
  
  std::this_thread::sleep_for(500ms);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseGripperAroundCan::tick()
{
  double can_diameter;
  getInput("canDiameter", can_diameter);
  
  RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] START: can_diameter=%.4f", can_diameter);
  
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_finger_pub;
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_finger_pub;
  
  if (!left_finger_pub) {
    RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] Creating publishers...");
    left_finger_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/parallel_gripper_base_plate_to_left_finger/position", 10);
    right_finger_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/parallel_gripper_base_plate_to_right_finger/position", 10);
    std::this_thread::sleep_for(100ms);
  }
  
  // Calculate squeeze position
  // Finger Origin is at approx 0.047m from center.
  // We want finger surface at (diameter/2).
  // Target Joint = (diameter/2) - 0.047.
  double gripper_half_width_approx = 0.047;
  double target_pos_left = 0.010 - gripper_half_width_approx;
  
  // Clamp to limits just in case
  if (target_pos_left < -0.044) {
    RCLCPP_WARN(node_->get_logger(), "[CloseGripperAroundCan] Clamping left from %.4f to -0.044", target_pos_left);
    target_pos_left = -0.044;
  }
  if (target_pos_left > 0.0) {
    RCLCPP_WARN(node_->get_logger(), "[CloseGripperAroundCan] Clamping left from %.4f to 0.0", target_pos_left);
    target_pos_left = 0.0;
  }
  
  double target_pos_right = -target_pos_left; // Right is symmetric positive

  std_msgs::msg::Float64 msg_left, msg_right;
  msg_left.data = target_pos_left;
  msg_right.data = target_pos_right;

  RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] Closing to Left=%.4f, Right=%.4f", 
              target_pos_left, target_pos_right);
  left_finger_pub->publish(msg_left);
  right_finger_pub->publish(msg_right);

  std::this_thread::sleep_for(1500ms);
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

  const double pi_age = (node_->now() - g_detection_state.pi_last_detection_time).seconds();
  if (pi_age >= ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
    RCLCPP_WARN(node_->get_logger(), "[ExtendTowardsCan] Pi camera detection stale (%.2fs)", pi_age);
    return BT::NodeStatus::FAILURE;
  }

  const double offset_x = std::abs(g_detection_state.pi_detection_position.x);
  const double offset_y = std::abs(g_detection_state.pi_detection_position.y);
  
  // Use loose tolerances for this final sanity check. 
  // X: Check horizontal alignment angle (Yaw). 
  //    The user clarified: "The test should only be about the theta deviation"
  //    atan2(x, z) gives us the horizontal angle. Z is forward, X is right.
  double angle_x_rad = std::atan2(offset_x, g_detection_state.pi_detection_position.z);
  
  // Tolerance: 5 degrees (approx 0.087 rad)
  if (std::abs(angle_x_rad) > 0.1) {
    RCLCPP_WARN(node_->get_logger(),
                "[ExtendTowardsCan] Alignment Check Failed: Horizontal Angle %.3f rad too high (limit 0.1)",
                angle_x_rad);
    // return BT::NodeStatus::FAILURE; // Allow proceed for now per user request? 
    // Actually, user said "The test should only be about the theta deviation". So if theta is bad, fail.
    return BT::NodeStatus::FAILURE;
  }

  // Y: User explicitly said "I'm not sure what the y misalignment is about... RaiseElevatorToCanHeight fixed the z axis alighment"
  // If the user believes vertical alignment is solved, we should ignore Y error or just warn about it.
  // The log showed a 5.6cm error. If we fail on Y, we stop. If we ignore Y, we extend.
  // We will log Y error but NOT fail on it.
  if (offset_y > 0.08) {
      RCLCPP_WARN(node_->get_logger(),
                  "[ExtendTowardsCan] WARN: Vertical misalignment y=%.3f exceeds 8cm. Extending anyway per user instruction.",
                  offset_y);
  } else {
      RCLCPP_INFO(node_->get_logger(), "[ExtendTowardsCan] Vertical misalignment y=%.3f (acceptable)", offset_y);
  }
  
  RCLCPP_INFO(node_->get_logger(), "[ExtendTowardsCan] Alignment passed. Calculating extension...");
  
  // Distance forward to can is the Z coordinate in camera optical frame
  double distance_to_can = g_detection_state.pi_detection_position.z;
  
  // Target: Place the 'parallel_gripper_base_plate' (palm) ~5mm from the front of the can.
  // Previous test with 0.03m offset resulted in pushing the can (too long).
  // Can Radius is 33mm.
  // A safe offset estimate:
  // If 0.03m caused contact, we need to retract.
  // Let's increase offset to 0.045m (4.5cm). This reduces extension by 1.5cm vs the "pushing" case.
  // This aims to achieve the requested ~5mm gap without touching.
  double extension_distance = std::max(0.0, distance_to_can - 0.045); 
  
  RCLCPP_INFO(node_->get_logger(),
              "[ExtendTowardsCan] Extending gripper %.3fm toward '%s' (detected at %.3fm) [Offset 0.045]",
              extension_distance, object_name.c_str(), distance_to_can);
              
  // Log every step
  static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr extender_pub;
  if (!extender_pub) {
      RCLCPP_INFO(node_->get_logger(), "[ExtendTowardsCan] Creating publisher...");
    extender_pub = node_->create_publisher<std_msgs::msg::Float64>(
      "/gripper_elevator_plate_to_gripper_extender/position", 10);
  }

  std_msgs::msg::Float64 msg;
  msg.data = extension_distance;
  extender_pub->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "[ExtendTowardsCan] Published extension command: %.3f", extension_distance);

  {
    std::lock_guard<std::mutex> lock(g_sensor_mutex);
    g_sensor_state.last_extender_command = extension_distance;
  }

  std::this_thread::sleep_for(1s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AdjustExtenderToCenterCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  // Get fresh detection
  geometry_msgs::msg::Point det_point;
  double age_sec = 999.0;
  if (!getFreshPiDetection(node_, det_point, age_sec)) {
    RCLCPP_WARN(node_->get_logger(), "[AdjustExtenderToCenterCan] No fresh detection");
    return BT::NodeStatus::FAILURE;
  }

  if (!piDetectionInRange(det_point)) {
    RCLCPP_WARN(node_->get_logger(), "[AdjustExtenderToCenterCan] Detection out of range");
    return BT::NodeStatus::FAILURE;
  }

  // Get horizontal offset in camera frame (X axis, positive = right)
  const double x_offset = det_point.x;
  
  // If already centered, we're done
  if (std::abs(x_offset) < ObjectDetectionState::CENTERING_TOLERANCE) {
    RCLCPP_INFO(node_->get_logger(), "[AdjustExtenderToCenterCan] Already centered");
    return BT::NodeStatus::SUCCESS;
  }

  // Calculate rotation needed
  // Positive x_offset means can is to the right (Camera Frame X+).
  // Robot Base Frame: Positive Z is Left (CCW), Negative Z is Right (CW).
  // Therefore, we need a NEGATIVE angular_z for a POSITIVE x_offset.
  
  // Use distance to calculate precise rotation angle needed:
  double distance = det_point.z;
  double angle_error_rad = std::atan2(x_offset, distance);
  
  // We desire to zero this angle error.
  
  // Bse duration for the rotation maneuver
  double duration_sec = 0.5;

  // Calculate required velocity to complete correction in 'duration_sec'
  // angular_z * duration = -angle_error
  double required_omega = -angle_error_rad / duration_sec;
  
  RCLCPP_INFO(node_->get_logger(), 
            "[AdjustExtenderToCenterCan] Plan: err=%.4fm, dist=%.3fm -> angle_err=%.4f rad. Need omega=%.3f rad/s for %.1fs", 
             x_offset, distance, angle_error_rad, required_omega, duration_sec);
  
  const double min_angular = 0.15; // Minimum robust rotation speed
  double angular_z = 0.0;
  
  if (std::abs(required_omega) < min_angular) {
      // If required velocity is too low for the robot to move reliably,
      // we set velocity to min_angular and reduce duration proportionally
      // to achieve the same total rotation angle.
      double direction = (required_omega >= 0) ? 1.0 : -1.0;
      angular_z = direction * min_angular;
      
      // New duration: time = angle / velocity
      duration_sec = std::abs(angle_error_rad / min_angular);
      
      RCLCPP_INFO(node_->get_logger(), 
            "[AdjustExtenderToCenterCan] Micro-adjust: Required %.3f < min %.3f. Boosting to %.3f, reducing time to %.3fs", 
             required_omega, min_angular, angular_z, duration_sec);

      // Ensure we don't have excessively short pulses
      if (duration_sec < 0.1) {
          duration_sec = 0.1;
      }
  } else {
      angular_z = required_omega;
  }

  // Cap at maximum angular velocity
  const double max_angular = 1.0; 
  if (std::abs(angular_z) > max_angular) {
      angular_z = (angular_z > 0) ? max_angular : -max_angular;
      // Note: If we cap velocity, we won't complete rotation in original duration,
      // but we iterate so next frame will catch remaining error.
      RCLCPP_WARN(node_->get_logger(), 
            "[AdjustExtenderToCenterCan] Saturated: %.3f > max %.3f. Capped at %.3f", 
             std::abs(required_omega), max_angular, angular_z);
  }

  // Publish twist command to rotate robot
  // Use cmd_vel_smoothed to bypass velocity smoother latency for small movements
  // and go directly to the twist multiplexer.
  static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
  if (!twist_pub) {
    twist_pub = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed", 10);
  }

  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;
  
  long duration_ms = static_cast<long>(duration_sec * 1000);
  
  twist_pub->publish(twist);
  
  RCLCPP_INFO(node_->get_logger(), 
              "[AdjustExtenderToCenterCan] EXECUTING: z=%.3f rad/s for %ldms", 
              angular_z, duration_ms);

  // Sleep for the calculated duration to allow movement
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  
  // Stop rotation
  twist.angular.z = 0.0;
  twist_pub->publish(twist);
  
  // Wait a bit for settling
  std::this_thread::sleep_for(500ms);

  // Return FAILURE to indicate we had to move (not stable yet), prompting a retry
  return BT::NodeStatus::FAILURE;
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
  RCLCPP_INFO(node_->get_logger(), "### EXECUTING FROM SIMULATION (bt_nodes.cpp) ###");
  std::string can_name;
  getInput("canName", can_name);

  geometry_msgs::msg::Point location;
  try {
    const std::string pkg_share = ament_index_cpp::get_package_share_directory("can_do_challenge");
    const std::string db_path = pkg_share + "/config/can_locations.json";

    std::ifstream in(db_path);
    if (!in.is_open()) {
      RCLCPP_ERROR(node_->get_logger(), "[LoadCanLocation] Failed to open %s", db_path.c_str());
      return BT::NodeStatus::FAILURE;
    }

    std::ostringstream ss;
    ss << in.rdbuf();
    const std::string content = ss.str();

    // Minimal parser for our known config format.
    // Matches the entry with the requested name and captures x/y/z.
    const std::string pattern =
      std::string(R"(\"name\"\s*:\s*\")") + can_name +
      R"(\"[\s\S]*?\"location\"\s*:\s*\{[\s\S]*?\"x\"\s*:\s*([-+0-9.eE]+)[\s\S]*?\"y\"\s*:\s*([-+0-9.eE]+)[\s\S]*?\"z\"\s*:\s*([-+0-9.eE]+))";

    std::regex re(pattern);
    std::smatch m;
    if (!std::regex_search(content, m, re) || m.size() != 4) {
      RCLCPP_ERROR(node_->get_logger(), "[LoadCanLocation] Can '%s' not found in %s", can_name.c_str(), db_path.c_str());
      return BT::NodeStatus::FAILURE;
    }

    location.x = std::stod(m[1].str());
    location.y = std::stod(m[2].str());
    location.z = std::stod(m[3].str());

    setOutput("location", location);
    RCLCPP_INFO(node_->get_logger(), "[LoadCanLocation] Loaded '%s' from %s: (%.3f, %.3f, %.3f)",
                can_name.c_str(), db_path.c_str(), location.x, location.y, location.z);
    return BT::NodeStatus::SUCCESS;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "[LoadCanLocation] Failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
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

BT::NodeStatus WaitForDetection::onStart()
{
  start_time_ = node_->now();
  RCLCPP_DEBUG(node_->get_logger(), "[WaitForDetection] Waiting for detection...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDetection::onRunning()
{
  // Keep the tree alive while we wait for the camera condition to become true.
  // Sleep a little to avoid busy-spinning if the BT is ticked fast.
  std::this_thread::sleep_for(100ms);

  // Optional safety timeout: if nothing arrives for a long time, fail so higher-level
  // logic can decide what to do. (But do not fail quickly and abort the whole mission.)
  constexpr double kTimeoutSec = 15.0;
  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > kTimeoutSec) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForDetection] Timed out after %.1fs", elapsed);
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForDetection::onHalted()
{
  // Nothing to clean up.
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

BT::NodeStatus ReactiveRepeatUntilSuccessOrCount::tick()
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

  // If the child succeeds (e.g. "CenterCan" confirms we are centered), we are done.
  if (child_status == BT::NodeStatus::SUCCESS) {
    current_cycle_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  // If child failed (e.g. "CenterCan" had to move and thus wasn't centered yet),
  // we count this as a cycle used and retry.
  current_cycle_++;

  if (current_cycle_ >= num_cycles) {
    current_cycle_ = 0;
    return BT::NodeStatus::FAILURE; // Completed all cycles without Success
  }

  return BT::NodeStatus::RUNNING;
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
