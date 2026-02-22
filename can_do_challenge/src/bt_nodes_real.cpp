// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "can_do_challenge/bt_nodes_real.hpp"
#include "sigyn_interfaces/msg/e_stop_status.hpp"
#include "sigyn_interfaces/msg/gripper_status.hpp"
#include "sigyn_interfaces/msg/gripper_position_command.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sigyn_interfaces/msg/detection_array.hpp"
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
#include <unordered_set>
#include <algorithm>

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
// Real robot calibration: when elevator_position == 0.0 (homed),
// gripper center is 0.502 m above base_link in +Z.
static constexpr double kGripperCenterAtHomeAboveBaseLink = 0.502;

static SimulatedSensorState g_sensor_state;
static std::mutex g_sensor_mutex;

// Centralized Publisher/Subscriber Manager
struct SharedResources {
  // Publishers
  rclcpp::Publisher<sigyn_interfaces::msg::GripperPositionCommand>::SharedPtr gripper_position_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripper_home_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_nav_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_smoothed_pub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twister_pub;
  
  // Subscribers
  rclcpp::Subscription<sigyn_interfaces::msg::GripperStatus>::SharedPtr gripper_status_sub;
  
  // Gripper status state
  double elevator_position = 0.0;
  bool is_moving = false;
  std::mutex gripper_mutex;
  bool gripper_status_received = false;
  
  bool initialized = false;
};

static SharedResources g_resources;
static std::mutex g_resources_mutex;

// Global object detection state
struct ObjectDetectionState {
  // OAK-D camera detection
  // Raw point as published by the detector (typically camera optical frame)
  geometry_msgs::msg::Point oakd_detection_position_raw;
  std::string oakd_detection_frame_raw;
  
  // 2D info for logging (if 3D is not available)
  float oakd_2d_center_x = 0.0f;
  float oakd_2d_center_y = 0.0f;

  // Anchored point in a fixed frame (odom) to avoid "carrot on a stick" issues when rotating
  geometry_msgs::msg::Point oakd_detection_position_odom;
  bool oakd_detection_has_odom = false;
  bool oakd_can_detected = false;
  std::string oakd_detection_class;
  double oakd_detection_score = 0.0;
  rclcpp::Time oakd_last_detection_time;
  
  // Pi camera (gripper) detection
  geometry_msgs::msg::Point pi_detection_position;
  double pi_detection_pixel_y = 0.0;  // Original pixel Y coordinate from camera
  std::string pi_detection_frame = "map";
  rclcpp::Time pi_detection_stamp;
  bool pi_can_detected = false;
  rclcpp::Time pi_last_detection_time;
  
  // Detection parameters
  static constexpr double DETECTION_TIMEOUT_SEC = 2.0;
  // OAKD_MAX_AGE_SEC increased to tolerate system latency
  static constexpr double OAKD_MAX_AGE_SEC = 2.0;
  static constexpr double PI_DETECTION_TIMEOUT_SEC = 6.0;
  static constexpr int PI_WAIT_TIMEOUT_MS = 10000;  // 10s to allow camera initialization
  static constexpr double WITHIN_REACH_DISTANCE = -1.0;  // MUST be provided by BT
  static constexpr double CENTERING_TOLERANCE = 0.02;   // Tightened to 2cm for precision
  static constexpr double PI_HORIZONTAL_TARGET_OFFSET = -0.020; // Offset to compensate for camera/gripper alignment (negative = target left of optical center)
  static constexpr double PI_VERTICAL_TOLERANCE = 0.05; // 5cm for elevator height check
  static constexpr double PI_VERTICAL_TARGET_OFFSET = 0.0; // target offset in optical Y (positive down)
  static constexpr double PI_MIN_DISTANCE_M = 0.15;
  static constexpr double PI_MAX_DISTANCE_M = 0.80;
  
  // System Health
  rclcpp::Time oakd_last_heartbeat_time;
  bool oakd_alive = false;
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
  
  // Non-blocking check for detection
  std::lock_guard<std::mutex> lock(g_detection_mutex);

  if (!g_detection_state.pi_can_detected) {
    out_age_sec = 999.0;
    missing_count++;
    // Log less frequently to avoid flooding
    if (missing_count % 50 == 0) {
      RCLCPP_INFO(node->get_logger(),
                  "[PiDetection] No detection available yet (count=%d)",
                  missing_count);
    }
    return false;
  }

  out_age_sec = (node->now() - g_detection_state.pi_last_detection_time).seconds();
  if (out_age_sec >= ObjectDetectionState::PI_DETECTION_TIMEOUT_SEC) {
    stale_count++;
    if (stale_count % 50 == 0) {
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
  // Accept 2D-only detections (Z approx 0)
  if (std::abs(point.z) < 0.001) return true; 
  
  // Accept 0.30 as the default "unknown depth" from 2D-only detections
  // Or check if in valid depth range for 3D detections  
  if (std::abs(point.z - 0.30) < 0.01) return true;  // Default gripper camera distance
  return point.z >= ObjectDetectionState::PI_MIN_DISTANCE_M &&
         point.z <= ObjectDetectionState::PI_MAX_DISTANCE_M;
}

// Store subscriptions so they don't get destroyed
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_oakd_sub;
static rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr g_oakd_real_sub;
static rclcpp::Subscription<sigyn_interfaces::msg::DetectionArray>::SharedPtr g_pi_sub;
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
  
  RCLCPP_INFO(node->get_logger(), "Initializing object detection subscribers (Real & Sim)");
  
  // Create QoS
  rclcpp::QoS oakd_qos(10);
  oakd_qos.best_effort();

  // Global TF listener
  g_tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  g_tf_listener = std::make_shared<tf2_ros::TransformListener>(*g_tf_buffer);

  RCLCPP_INFO(node->get_logger(), "Subscribing to /oakd/object_detector_heartbeat with BEST_EFFORT reliability");

  // Subscribe to OAK-D detection (Real Robot)
  g_oakd_real_sub = node->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/oakd/object_detector_heartbeat", oakd_qos,
    [node](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(g_detection_mutex);
            g_detection_state.oakd_last_heartbeat_time = node->now();
            g_detection_state.oakd_alive = true;
        }

        if (!msg->detections.empty()) {
            std::lock_guard<std::mutex> lock(g_detection_mutex);
            // Just grab the first detection for Step 1 logging
            const auto& det = msg->detections[0]; 
            
            // Store 2D info for logging
            g_detection_state.oakd_2d_center_x = det.bbox.center.position.x;
            g_detection_state.oakd_2d_center_y = det.bbox.center.position.y;
            
            // Log for Step 1
            // if (!det.results.empty()) {
            //    float conf = det.results[0].hypothesis.score;
            //    std::string cls = det.results[0].hypothesis.class_id;
            //    float cx = det.bbox.center.position.x;
            //    float cy = det.bbox.center.position.y;
            //    RCLCPP_INFO(node->get_logger(), 
            //        "[OAKD-Real] Class: %s, Conf: %.2f, Box: [x=%.1f, y=%.1f]", 
            //        cls.c_str(), conf, cx, cy);
            // }
        }
    });

  auto oakd_point_cb = [node](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      g_detection_state.oakd_detection_position_raw = msg->point;
      g_detection_state.oakd_detection_frame_raw = msg->header.frame_id;
      g_detection_state.oakd_can_detected = true;
      g_detection_state.oakd_alive = true;
      g_detection_state.oakd_last_detection_time = node->now();
      g_detection_state.oakd_last_heartbeat_time = node->now();
    }

    geometry_msgs::msg::PointStamped detection_odom;
    try {
      geometry_msgs::msg::PointStamped msg_latest = *msg;
      msg_latest.header.stamp = rclcpp::Time(0);
      detection_odom = g_tf_buffer->transform(msg_latest, "odom");

      std::lock_guard<std::mutex> lock(g_detection_mutex);
      g_detection_state.oakd_detection_position_odom = detection_odom.point;
      g_detection_state.oakd_detection_has_odom = true;
    } catch (const tf2::TransformException & ex) {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      g_detection_state.oakd_detection_has_odom = false;
    }
  };

  if (node->get_parameter("use_sim_time").as_bool()) {
    g_oakd_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "/oakd_top/can_detection", oakd_qos, oakd_point_cb);
    RCLCPP_INFO(node->get_logger(), "Subscribed to /oakd_top/can_detection (best_effort, sim-only)");
  } else {
    g_oakd_sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
      "/oakd/can_detection", oakd_qos, oakd_point_cb);
    RCLCPP_INFO(node->get_logger(), "Subscribed to /oakd/can_detection (best_effort)");
  }
  
  // Subscribe to Pi camera detection (sigyn_interfaces package)
  rclcpp::QoS pi_qos(10);
  // Use RELIABLE to match publisher QoS (detection data is low-rate and critical)
  pi_qos.reliable();

  g_pi_sub = node->create_subscription<sigyn_interfaces::msg::DetectionArray>(
    "/gripper/camera/detections", pi_qos,
    [node](const sigyn_interfaces::msg::DetectionArray::SharedPtr msg) {
      // Update frame timestamp for WaitForNewPiFrameProcessed
      {
        std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
        g_pi_last_frame_time = rclcpp::Time(msg->header.stamp);
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
          "[Pi-Subscription] Callback invoked! Frame time=%.3f detections=%zu",
          g_pi_last_frame_time.seconds(), msg->detections.size());
      }
      g_pi_frame_cv.notify_all();
      
      // Process first detection if available
      if (!msg->detections.empty()) {
        const auto& det = msg->detections[0];
        
        // Convert pixel coordinates to normalized camera frame
        // Gripper camera (Pi AI HAT) uses 640x640 resolution
        // Camera center at (320, 320) and focal length ~500px
        const double image_width = 640.0;
        const double image_height = 640.0;
        const double fx = 500.0;  // Approximate focal length in pixels
        const double fy = 500.0;
        const double cx = image_width / 2.0;  // 320
        const double cy = image_height / 2.0; // 320
        
        // Assume a nominal distance for projection (will be refined with actual distance if available)
        double nominal_distance = 0.3;  // 30cm typical gripper reach
        
        // Store original pixel Y coordinate BEFORE conversion
        double original_pixel_y = det.center.y;
        
        // Convert pixel coordinates to 3D camera frame (optical conventions: X=right, Y=down, Z=forward)
        double x_3d = (det.center.x - cx) * nominal_distance / fx;
        double y_3d = (det.center.y - cy) * nominal_distance / fy;
        double z_3d = nominal_distance;
        
        // Store detection position in meters
        geometry_msgs::msg::Point pos;
        pos.x = x_3d;
        pos.y = y_3d;
        pos.z = z_3d;
        
        {
          std::lock_guard<std::mutex> lock(g_detection_mutex);
          g_detection_state.pi_detection_position = pos;
          g_detection_state.pi_detection_pixel_y = original_pixel_y;
          g_detection_state.pi_detection_frame = msg->header.frame_id;
          g_detection_state.pi_detection_stamp = rclcpp::Time(msg->header.stamp);
          g_detection_state.pi_can_detected = true;
          g_detection_state.pi_last_detection_time = node->now();
        }
        g_pi_detection_cv.notify_all();
        
        RCLCPP_INFO(node->get_logger(), "[Pi-Camera] Can detected: center=(%.1f, %.1f, %.1f) conf=%.2f frame='%s'",
                pos.x, pos.y, pos.z, det.confidence, msg->header.frame_id.c_str());
      }
    });
  RCLCPP_INFO(node->get_logger(), "Subscribed to /gripper/camera/detections (best_effort, sigyn_interfaces::msg::DetectionArray)");
  
  initialized = true;
}

// Initialize shared publishers and subscribers once
static void initializeSharedResources(std::shared_ptr<rclcpp::Node> node) {
  std::lock_guard<std::mutex> lock(g_resources_mutex);
  if (g_resources.initialized) return;
  
  // Create all publishers once
  g_resources.gripper_position_pub = node->create_publisher<sigyn_interfaces::msg::GripperPositionCommand>(
    "/gripper/position/command", 10);
  g_resources.gripper_home_pub = node->create_publisher<std_msgs::msg::Empty>(
    "/gripper/home", 10);
  g_resources.cmd_vel_nav_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_nav", 10);
  g_resources.cmd_vel_smoothed_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_smoothed", 10);
  g_resources.twister_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_testicle_twister", 10);
  
  // Create gripper status subscriber once
  g_resources.gripper_status_sub = node->create_subscription<sigyn_interfaces::msg::GripperStatus>(
    "/gripper/status", 10,
    [](const sigyn_interfaces::msg::GripperStatus::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
      g_resources.elevator_position = msg->elevator_position;
      g_resources.is_moving = msg->is_moving;
      g_resources.gripper_status_received = true;
    });
  
  g_resources.initialized = true;
  RCLCPP_INFO(node->get_logger(), "Shared resources initialized");
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
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  bool above_threshold = g_sensor_state.battery_voltage > SimulatedSensorState::CHARGING_VOLTAGE;
  
  RCLCPP_INFO(node_->get_logger(), "[BatteryAboveChargingVoltage] Voltage: %.2fV (threshold: %.2fV) -> %s",
    g_sensor_state.battery_voltage, SimulatedSensorState::CHARGING_VOLTAGE,
    above_threshold ? "SUCCESS" : "FAILURE");
    
  return above_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus BatteryAboveCriticalVoltage::tick()
{
  std::lock_guard<std::mutex> lock(g_sensor_mutex);
  bool above_threshold = g_sensor_state.battery_voltage > SimulatedSensorState::CRITICAL_VOLTAGE;
  
  RCLCPP_INFO(node_->get_logger(), "[BatteryAboveCriticalVoltage] Voltage: %.2fV (threshold: %.2fV) -> %s",
    g_sensor_state.battery_voltage, SimulatedSensorState::CRITICAL_VOLTAGE,
    above_threshold ? "SUCCESS" : "FAILURE");
  
  return above_threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus RobotIsEstopped::tick()
{
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
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                 "[CanDetectedByOAKD] Searching for '%s' with OAK-D... (No detections yet)",
                 object_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto time_since_detection = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
  if (time_since_detection >= ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
    RCLCPP_DEBUG(node_->get_logger(), "[CanDetectedByOAKD] Detection stale (%.2fs)", time_since_detection);
    publishDebugStatus(node_, "CanDetectedByOAKD: stale age=" + std::to_string(time_since_detection));
    return BT::NodeStatus::FAILURE;
  }

  float x_2d = g_detection_state.oakd_2d_center_x;
  float y_2d = g_detection_state.oakd_2d_center_y;
  double x_3d = g_detection_state.oakd_detection_position_raw.x;
  double y_3d = g_detection_state.oakd_detection_position_raw.y;
  double z_3d = g_detection_state.oakd_detection_position_raw.z;

  // Optional: non-blocking transform to base_link for trace output
  if (g_tf_buffer && !g_detection_state.oakd_detection_frame_raw.empty()) {
    try {
      if (g_tf_buffer->canTransform(
            "base_link",
            g_detection_state.oakd_detection_frame_raw,
            tf2::TimePointZero,
            tf2::durationFromSec(0.0))) {
        geometry_msgs::msg::PointStamped det_msg;
        det_msg.header.frame_id = g_detection_state.oakd_detection_frame_raw;
        det_msg.header.stamp = g_detection_state.oakd_last_detection_time;
        det_msg.point = g_detection_state.oakd_detection_position_raw;
        auto det_in_base = g_tf_buffer->transform(det_msg, "base_link");
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
          "[CanDetectedByOAKD] base_link=(%.3f, %.3f, %.3f) frame='%s' age=%.2fs",
          det_in_base.point.x, det_in_base.point.y, det_in_base.point.z,
          g_detection_state.oakd_detection_frame_raw.c_str(),
          time_since_detection);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
        "[CanDetectedByOAKD] TF skipped: %s", ex.what());
    }
  }

  if (std::abs(z_3d) < 0.001 && std::abs(x_2d) > 0.01f) {
      RCLCPP_INFO(node_->get_logger(),
                  "[CanDetectedByOAKD] Can '%s' detected (2D Box Center=[%.1f, %.1f] age=%.2fs)",
                  object_name.c_str(),
                  x_2d,
                  y_2d,
                  time_since_detection);
  } else {
      RCLCPP_INFO(node_->get_logger(),
                  "[CanDetectedByOAKD] Can '%s' detected (raw frame='%s' raw=(%.2f,%.2f,%.2f) 2D=(%.1f,%.1f) age=%.2fs)",
                  object_name.c_str(),
                  g_detection_state.oakd_detection_frame_raw.c_str(),
                  x_3d, y_3d, z_3d, x_2d, y_2d,
                  time_since_detection);
  }

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
  // Apply target offset to compensate for camera/gripper physical alignment
  const double offset_x = std::abs(det_point.x - ObjectDetectionState::PI_HORIZONTAL_TARGET_OFFSET);
  
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
  initializeSharedResources(node_);
  bool can_detected = false;
  geometry_msgs::msg::PointStamped can_location;
  double within_distance = kDefaultWithinReachDistance;

  if (!getInput("can_detected", can_detected)) {
    RCLCPP_ERROR(node_->get_logger(), "[CanWithinReach] Missing can_detected input");
    return BT::NodeStatus::FAILURE;
  }

  if (!can_detected) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
      "[CanWithinReach] Can not detected (can_detected=false)");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("can_location", can_location)) {
    RCLCPP_ERROR(node_->get_logger(), "[CanWithinReach] Missing can_location input");
    return BT::NodeStatus::FAILURE;
  }

  getInput("within_distance", within_distance);

  if (within_distance <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
      "[CanWithinReach] Invalid within_distance=%.2f. Must be set via BT XML/blackboard.",
      within_distance);
    return BT::NodeStatus::FAILURE;
  }

  // Verify can_location is in base_link frame, transform if needed
  geometry_msgs::msg::PointStamped can_in_base;
  // Handle Pixel Mode (Z=0) transformation for CanWithinReach
  if (std::abs(can_location.point.z) < 0.001) {
     RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
       "[CanWithinReach] Input is raw pixel data (Z=0). frame='%s'",
       can_location.header.frame_id.c_str());
     // Return FAILURE so MoveTowardsCan can continue visually servoing
     // We cannot possibly be "Within Reach" if we only have pixels and no depth.
     return BT::NodeStatus::FAILURE; 
  }

  if (can_location.header.frame_id == "base_link") {
    can_in_base = can_location;
    RCLCPP_DEBUG(node_->get_logger(), "[CanWithinReach] can_location already in base_link");
  } else {
    // Need to transform
    if (!g_tf_buffer) {
      RCLCPP_ERROR(node_->get_logger(), "[CanWithinReach] TF buffer not available");
      return BT::NodeStatus::FAILURE;
    }
    
    try {
      if (!g_tf_buffer->canTransform(
            "base_link",
            can_location.header.frame_id,
            tf2::TimePointZero,
            tf2::durationFromSec(0.0))) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
          "[CanWithinReach] TF not ready for '%s' -> base_link",
          can_location.header.frame_id.c_str());
        return BT::NodeStatus::FAILURE;
      }

      can_in_base = g_tf_buffer->transform(can_location, "base_link");
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
        "[CanWithinReach] base_link=(%.3f,%.3f,%.3f) frame='%s'",
        can_in_base.point.x, can_in_base.point.y, can_in_base.point.z,
        can_location.header.frame_id.c_str());
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), 
        "[CanWithinReach] TF transform failed from '%s' to base_link: %s", 
        can_location.header.frame_id.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  const double dx = can_in_base.point.x;
  const double dy = can_in_base.point.y;
  const double distance = std::hypot(dx, dy);

  RCLCPP_INFO(node_->get_logger(), 
    "[CanWithinReach] frame='%s' base dx=%.2f dy=%.2f dist=%.2f (need %.2f)",
    can_in_base.header.frame_id.c_str(), dx, dy, distance, within_distance);

  if (dx > 0.0 && distance < within_distance) {
    RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Can within reach at %.2fm", distance);
    
    // CRITICAL: Publish stop command immediately to prevent overshoot
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    g_resources.cmd_vel_nav_pub->publish(stop_cmd);
    RCLCPP_INFO(node_->get_logger(), "[CanWithinReach] Published STOP command to /cmd_vel_nav");
    
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
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
    rclcpp::Time initial_frame_time;
    {
      std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
      last_frame_time_ = g_pi_last_frame_time;
      initial_frame_time = g_pi_last_frame_time;
    }
    waiting_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] INIT: Starting wait. last_frame_time=%.3f (nanos=%ld) current_time=%.3f",
                last_frame_time_.seconds(), last_frame_time_.nanoseconds(), current_time.seconds());
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] INIT: g_pi_last_frame_time=%.3f (nanos=%ld)",
                initial_frame_time.seconds(), initial_frame_time.nanoseconds());
  }
  
  // Check if we've exceeded timeout
  auto elapsed = (current_time - wait_start_time_).seconds();
  
  // Check if new frame has been processed (non-blocking)
  // The heartbeat is published after object detection runs, regardless of whether can was found
  rclcpp::Time current_frame_time;
  {
    std::lock_guard<std::mutex> lock(g_pi_frame_mutex);
    current_frame_time = g_pi_last_frame_time;
  }
  
  // Log every second while waiting
  if (elapsed >= 1.0 && static_cast<int>(elapsed) != static_cast<int>(elapsed - 0.1)) {
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] TICK: elapsed=%.1fs current_frame_time=%.3f (nanos=%ld) last_frame_time=%.3f (nanos=%ld) match=%s zero=%s",
                elapsed,
                current_frame_time.seconds(), current_frame_time.nanoseconds(),
                last_frame_time_.seconds(), last_frame_time_.nanoseconds(),
                (current_frame_time.nanoseconds() == last_frame_time_.nanoseconds()) ? "YES" : "NO",
                (current_frame_time.nanoseconds() == 0) ? "YES" : "NO");
  }
  
  if (elapsed > (ObjectDetectionState::PI_WAIT_TIMEOUT_MS / 1000.0)) {
    RCLCPP_INFO(node_->get_logger(), 
                "[WaitForNewPiFrameProcessed] TIMEOUT after %.1fs. Final state: current_frame=%.3f (nanos=%ld) last_frame=%.3f (nanos=%ld)",
                elapsed,
                current_frame_time.seconds(), current_frame_time.nanoseconds(),
                last_frame_time_.seconds(), last_frame_time_.nanoseconds());
    waiting_ = false;  // Reset for next attempt
    return BT::NodeStatus::FAILURE;
  }
  
  if (current_frame_time.nanoseconds() != last_frame_time_.nanoseconds() && current_frame_time.nanoseconds() > 0) {
    // New frame processed! Bounding box data (if any) is now available. Reset state and return success.
    waiting_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "[WaitForNewPiFrameProcessed] SUCCESS: New processed frame detected! current=%.3f (nanos=%ld) prev=%.3f (nanos=%ld) elapsed=%.1fs",
                current_frame_time.seconds(), current_frame_time.nanoseconds(),
                last_frame_time_.seconds(), last_frame_time_.nanoseconds(),
                elapsed);
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

  // Get the actual pixel Y coordinate from global state
  double pixel_y = 0.0;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    pixel_y = g_detection_state.pi_detection_pixel_y;
  }
  
  // Debug log both 3D and pixel values
  RCLCPP_INFO(node_->get_logger(),
           "[ElevatorAtHeight] Detection: 3D_y=%.3f pixel_y=%.1f age=%.2fs",
           det_point.y, pixel_y, age_sec);

  // Get target height in pixels
  double target_y_pixels = 342.0; // Default
  double tol_pixels = 10.0;
  
  if (getInput("targetHeightPixels", target_y_pixels)) {
      getInput("z_tolerance_pixels", tol_pixels);
  } else {
     // Fallback to param if not provided by port
     try {
        if (!node_->has_parameter("pi_target_y")) {
            node_->declare_parameter<double>("pi_target_y", 342.0);
        }
        node_->get_parameter("pi_target_y", target_y_pixels);
     } catch (...) {}
     
     try {
          if (!node_->has_parameter("pi_target_tolerance")) {
               node_->declare_parameter<double>("pi_target_tolerance", 10.0);
          }
           node_->get_parameter("pi_target_tolerance", tol_pixels);
      } catch (...) {}
  }

  // Compare pixel Y to target pixel Y
  double diff = std::abs(pixel_y - target_y_pixels);
  RCLCPP_INFO(node_->get_logger(),
          "[ElevatorAtHeight] Pixel Check: pixel_y=%.1f target=%.1f diff=%.1f tol=%.1f",
          pixel_y, target_y_pixels, diff, tol_pixels);

  if (diff <= tol_pixels) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}
// Removed lines 967-1082 (Legacy 3D logic that relied on non-zero Z)

BT::NodeStatus OAKDDetectCan::detectOnce()
{
  std::string object_name;
  if (!getInput("objectOfInterest", object_name)) {
    RCLCPP_ERROR(node_->get_logger(), "[OAKDDetectCan] Missing objectOfInterest input");
    return BT::NodeStatus::FAILURE;
  }

  initializeObjectDetection(node_);

  // Snapshot detection state under lock
  geometry_msgs::msg::Point det_raw;
  std::string det_frame;
  rclcpp::Time det_stamp;
  std::string det_class;
  double det_score = 0.0;
  double age_sec = 999.0;
  bool detected = false;
  
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (g_detection_state.oakd_can_detected) {
      age_sec = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
      if (age_sec < ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
        detected = true;
        det_raw = g_detection_state.oakd_detection_position_raw;
        det_frame = g_detection_state.oakd_detection_frame_raw;
        det_stamp = g_detection_state.oakd_last_detection_time;
        det_class = g_detection_state.oakd_detection_class;
        det_score = g_detection_state.oakd_detection_score;
      }
    }
  }

  if (!detected) {
    RCLCPP_INFO(node_->get_logger(), "[OAKDDetectCan] No fresh detection for '%s'", object_name.c_str());
    setOutput("can_detected", false);
    return BT::NodeStatus::FAILURE;
  }

  // Require a newer detection timestamp than the one previously consumed by this BT node.
  // This prevents reusing the same sample across many control ticks.
  if (det_stamp.nanoseconds() <= last_detection_stamp_.nanoseconds()) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
      "[OAKDDetectCan] Waiting for newer detection sample (stamp unchanged)");
    return BT::NodeStatus::RUNNING;
  }

  // Reject stale detections to avoid overshoot
  if (age_sec > ObjectDetectionState::OAKD_MAX_AGE_SEC) {
    RCLCPP_WARN(node_->get_logger(),
      "[OAKDDetectCan] Detection stale age=%.2fs (>%.2fs). Forcing stop.",
      age_sec, ObjectDetectionState::OAKD_MAX_AGE_SEC);
    initializeSharedResources(node_);
    geometry_msgs::msg::Twist stop_cmd;
    g_resources.cmd_vel_nav_pub->publish(stop_cmd);
    setOutput("can_detected", false);
    return BT::NodeStatus::SUCCESS;
  }

  // Optional class filter for "can"-like objects (COCO: bottle/cup)
  if (!object_name.empty()) {
    std::string name_lower = object_name;
    std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
    if (name_lower.find("can") != std::string::npos || name_lower.find("coke") != std::string::npos) {
      static const std::unordered_set<std::string> kCanClasses = {
        "bottle", "cup", "wine glass", "39", "41", "40", "0"
      };
      if (!det_class.empty() && kCanClasses.find(det_class) == kCanClasses.end()) {
        RCLCPP_INFO(node_->get_logger(),
          "[OAKDDetectCan] Ignoring detection class='%s' score=%.2f for object='%s'",
          det_class.c_str(), det_score, object_name.c_str());
        setOutput("can_detected", false);
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  // Transform to base_link
  if (!g_tf_buffer) {
    RCLCPP_WARN(node_->get_logger(), "[OAKDDetectCan] TF buffer not ready");
    setOutput("can_detected", false);
    return BT::NodeStatus::FAILURE;
  }

  // Handle Pixel Mode (Z=0)
  if (std::abs(det_raw.z) < 0.001) {
    RCLCPP_INFO(node_->get_logger(),
      "[OAKDDetectCan] Detection has Z=0 (Pixels?). Skipping TF transform. Frame='%s' Pt(%.1f, %.1f)",
      det_frame.c_str(), det_raw.x, det_raw.y);
      
    // Pass raw (pixel) point directly
    geometry_msgs::msg::PointStamped det_msg;
    det_msg.header.frame_id = det_frame; // e.g. "oak_rgb_camera_optical_frame"
    det_msg.header.stamp = det_stamp;
    det_msg.point = det_raw;

    setOutput("can_detected", true);
    setOutput("can_location", det_msg);
    last_detection_stamp_ = det_stamp;
    return BT::NodeStatus::SUCCESS;
  }

  try {
    geometry_msgs::msg::PointStamped det_msg;
    det_msg.header.frame_id = det_frame;
    det_msg.header.stamp = det_stamp;
    det_msg.point = det_raw;

    auto det_in_base = g_tf_buffer->transform(det_msg, "base_link");

    RCLCPP_INFO(node_->get_logger(),
      "[OAKDDetectCan] Input frame='%s' point=(%.3f, %.3f, %.3f) class='%s' score=%.2f",
      det_frame.c_str(), det_raw.x, det_raw.y, det_raw.z, det_class.c_str(), det_score);
    RCLCPP_INFO(node_->get_logger(), 
      "[OAKDDetectCan] Output frame='%s' point=(%.3f, %.3f, %.3f) age=%.2fs",
      det_in_base.header.frame_id.c_str(), det_in_base.point.x, det_in_base.point.y, det_in_base.point.z, age_sec);

    setOutput("can_detected", true);
    setOutput("can_location", det_in_base);  // Output full PointStamped, not just Point
    last_detection_stamp_ = det_stamp;
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "[OAKDDetectCan] TF transform failed: %s", ex.what());
    setOutput("can_detected", false);
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OAKDDetectCan::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "[OAKDDetectCan] Starting detection");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OAKDDetectCan::onRunning()
{
  return detectOnce();
}

void OAKDDetectCan::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "[OAKDDetectCan] Halted");
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
  
  const double standoff = 0.0;
  // Stand in front of table by standoff distance
  goal.pose.position.x = location.value().x;
  goal.pose.position.y = location.value().y - standoff;
  goal.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, location.value().z); // Face table
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
  const double within_distance,
  geometry_msgs::msg::PoseStamped& out_goal_in_map)
{
  if (within_distance <= 0.0) {
    RCLCPP_ERROR(node->get_logger(),
      "[ComputeApproachGoalToCan] Invalid within_distance=%.2f. Must be set via BT XML/blackboard.",
      within_distance);
    return BT::NodeStatus::FAILURE;
  }
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
  const double standoff = within_distance;
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
  double within_distance = kDefaultWithinReachDistance;
  getInput("within_distance", within_distance);
  geometry_msgs::msg::PoseStamped goal;
  const auto status = computeApproachGoalToCanOnce(node_, within_distance, goal);
  if (status == BT::NodeStatus::SUCCESS) {
    setOutput("goal", goal);
  }
  return status;
}

BT::NodeStatus ComputeApproachGoalToCan::onRunning()
{
  double within_distance = kDefaultWithinReachDistance;
  getInput("within_distance", within_distance);
  geometry_msgs::msg::PoseStamped goal;
  const auto status = computeApproachGoalToCanOnce(node_, within_distance, goal);
  if (status == BT::NodeStatus::SUCCESS) {
    setOutput("goal", goal);
  }
  return status;
}

// MoveTowardsCan: Incremental reactive movement
// Takes can_detected and can_location from blackboard, moves a small fixed distance toward it
// Returns SUCCESS immediately after sending one cmd_vel, allowing ReactiveFallback to re-evaluate
BT::NodeStatus MoveTowardsCan::tick()
{
  // Get inputs from blackboard
  bool can_detected = false;
  geometry_msgs::msg::PointStamped can_location;
  double target_distance_from_object = 0.55;
  double distance_tolerance = 0.01;

  if (!getInput("can_detected", can_detected)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveTowardsCan] Missing input 'can_detected'");
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput("can_location", can_location)) {
    // Spam prevention
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, 
        "[MoveTowardsCan] Missing input 'can_location' (detection likely stale or invalid)");
    return BT::NodeStatus::FAILURE;
  }

  getInput("target_distance_from_object", target_distance_from_object);
  getInput("distance_tolerance", distance_tolerance);

  if (!can_detected) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] can_detected is false, not moving");
    return BT::NodeStatus::FAILURE;
  }

  // Handle 2D Pixel Mode (Z=0)
  if (std::abs(can_location.point.z) < 0.001) {
    // We are in 2D mode.
    // Heuristic: If X ~ 160 (320w) or ~320 (640w).
    // User noted X=194 is "Right". So Center < 194. Likely 320x320 or 300x300 input.
    // Let's assume Center X is roughly 160 (for 320 width) or 150 (300 width).
    // We'll use 320 for 640 width.
    double center_x = 320.0; 
    double x_curr = can_location.point.x;
    
    // Error = (Current - Center).
    // If x=194. center=320. diff=-126 (Target is to the LEFT).
    // We need to turn LEFT (Positive Rot).
    // So cmd.z = -diff * gain.
    // If diff is negative (-126), cmd.z should be positive.
    // -(-126) = +126. Correct.
    double error_x = x_curr - center_x;
    
    initializeSharedResources(node_);
    geometry_msgs::msg::Twist cmd;
    
    // Rotation
    cmd.angular.z = -1.0 * error_x * 0.005; 
    
    // Linear: Move forward slowly if error is small (centered)
    if (std::abs(error_x) < 30.0) {
        cmd.linear.x = 0.08;
    } else {
        cmd.linear.x = 0.0;
    }

    RCLCPP_INFO(node_->get_logger(), 
        "[MoveTowardsCan] 2D Servoing (Z=0). X=%.1f (Ref=%.1f) Err=%.1f -> Cmd (mz=%.2f, lx=%.2f)",
        x_curr, center_x, error_x, cmd.angular.z, cmd.linear.x);

    g_resources.cmd_vel_nav_pub->publish(cmd);
    return BT::NodeStatus::SUCCESS;
  }

  // Transform can_location (which is likely in odom/map) to base_link
  geometry_msgs::msg::PointStamped can_in_base;
  try {
    // Ensure we have a recent transform
    // Note: can_location.header.stamp might be old, so we use Time(0) to get latest transform if needed, 
    // or respect the stamp if we trust odometry history. 
    // Given we are doing reactive visual servoing, latest is usually better.
    geometry_msgs::msg::PointStamped can_loc_latest = can_location;
    can_loc_latest.header.stamp = rclcpp::Time(0);
    can_in_base = g_tf_buffer->transform(can_loc_latest, "base_link");
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "[MoveTowardsCan] Could not transform can location: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Compute direction and distance to can in base_link frame
  // In base_link: X is forward, Y is left.
  double rel_x = can_in_base.point.x;
  double rel_y = can_in_base.point.y;
  double distance = std::hypot(rel_x, rel_y);
  double angle_error = std::atan2(rel_y, rel_x);

  initializeSharedResources(node_);
  
  RCLCPP_INFO(node_->get_logger(), 
              "[MoveTowardsCan] Target relative to base_link: x=%.3f, y=%.3f. Dist=%.3f Target=%.3f Angle=%.2f deg",
              rel_x, rel_y, distance, target_distance_from_object, angle_error * 180.0 / M_PI);

  geometry_msgs::msg::Twist cmd;

  // Safety stop: if we're already at the requested standoff, stop immediately.
  if (rel_x > 0.0 && distance <= (target_distance_from_object + distance_tolerance)) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    g_resources.cmd_vel_nav_pub->publish(cmd);
    RCLCPP_INFO(node_->get_logger(),
                "[MoveTowardsCan] At target standoff (dist=%.3f <= %.3f). STOP",
                distance, target_distance_from_object + distance_tolerance);
    return BT::NodeStatus::SUCCESS;
  }

  // Rotate in place if target not in front (X <= 0.05) or angle is large
  // If X is small, we are "abreast" of it or it is behind us.
  if (rel_x <= 0.05) {
    // Ensure we rotate the correct way. 
    // atan2(y, x) works correctly for all quadrants.
    cmd.angular.z = std::copysign(0.3, angle_error);
    cmd.linear.x = 0.0;
    g_resources.cmd_vel_nav_pub->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Target not in front (X=%.3f). Rotating signal=%.2f", rel_x, cmd.angular.z);
    return BT::NodeStatus::SUCCESS;
  }

  // Rotate towards target if angle error > 10 degrees
  if (std::abs(angle_error) > 0.17) {  // ~10 degrees
    cmd.angular.z = std::copysign(std::min(0.5, std::abs(angle_error)), angle_error);
    cmd.linear.x = 0.0;
    g_resources.cmd_vel_nav_pub->publish(cmd);
    RCLCPP_INFO(node_->get_logger(), "[MoveTowardsCan] Rotating toward target");
    return BT::NodeStatus::SUCCESS;
  }

  // Move forward (proportional to remaining distance error, capped)
  const double distance_error = std::max(0.0, distance - target_distance_from_object);
  double move_speed = std::clamp(distance_error * 0.6, 0.04, 0.12);
  cmd.linear.x = move_speed;
  cmd.angular.z = angle_error * 0.5;  // Gentle steering correction
  g_resources.cmd_vel_nav_pub->publish(cmd);

  RCLCPP_INFO(node_->get_logger(), 
              "[MoveTowardsCan] Moving forward at %.3f m/s (angular %.2f)", 
              cmd.linear.x, cmd.angular.z);

  // Return SUCCESS immediately - ReactiveFallback will call us again next tick
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RotateRobot::onStart()
{
  double degrees = 0.0;
  getInput("degrees", degrees);

  if (!cmd_vel_pub_) {
    cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_smoothed", 10);
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
  initializeSharedResources(node_);
  RCLCPP_INFO(node_->get_logger(), "[LowerElevator] Sending home command to gripper assembly");
  
  auto msg = std_msgs::msg::Empty();
  g_resources.gripper_home_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevator] Homing command sent");
  // Give time for homing to complete
  std::this_thread::sleep_for(3s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorSafely::tick()
{
  initializeSharedResources(node_);
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorSafely] Lowering elevator to 0.1m safe height");
  
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = 0.1;  // 0.1m safe height
  msg.extender_position = -1.0; // No change to extender
  g_resources.gripper_position_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[LowerElevatorSafely] Command sent: elevator=0.1m");
  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LowerElevatorToTable::tick()
{
  initializeSharedResources(node_);
  initializeObjectDetection(node_);

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

  // With new API, we directly command the elevator position in meters from home
  // The sigyn_to_sensor_v2 handles the joint space conversion
  const double elevator_position = target_height;
  
  // Publish position command
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(elevator_position)));
  msg.extender_position = -1.0; // No change to extender
  g_resources.gripper_position_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(),
              "[LowerElevatorToTable] Commanding elevator to %.5fm (clamped to 0.0-0.8999)",
              msg.elevator_position);
  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveElevatorToHeight::tick()
{
  initializeSharedResources(node_);
  initializeObjectDetection(node_);

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
  
  // With new API, we directly command the elevator position in meters from home
  const double elevator_position = target_height + kExtraTableClearance;
  
  // Publish position command
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(elevator_position)));
  msg.extender_position = -1.0; // No change to extender

  // Breakaway assist near home: when starting from hard-stop, do a small pre-lift first.
  if (g_resources.gripper_status_received && msg.elevator_position > 0.10f) {
    float current_pos = 0.0f;
    bool moving = false;
    {
      std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
      current_pos = static_cast<float>(g_resources.elevator_position);
      moving = g_resources.is_moving;
    }

    if (!moving && current_pos < 0.03f) {
      auto pre_msg = sigyn_interfaces::msg::GripperPositionCommand();
      pre_msg.elevator_position = std::min(msg.elevator_position, 0.08f);
      pre_msg.extender_position = -1.0;
      g_resources.gripper_position_pub->publish(pre_msg);

      RCLCPP_INFO(node_->get_logger(),
                  "[MoveElevatorToHeight] Home breakaway pre-lift: current=%.3f -> %.3f",
                  current_pos, pre_msg.elevator_position);

      bool saw_motion = false;
      const auto wait_start = node_->now();
      while (rclcpp::ok() && (node_->now() - wait_start).seconds() < 2.5) {
        bool now_moving = false;
        {
          std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
          now_moving = g_resources.is_moving;
        }
        saw_motion = saw_motion || now_moving;
        if (saw_motion && !now_moving) {
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
    }
  }

  g_resources.gripper_position_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(),
              "[MoveElevatorToHeight] Commanding elevator to %.5fm (target=%.3f + clearance=%.2f, clamped to 0.0-0.8999)",
              msg.elevator_position, target_height, kExtraTableClearance);
  std::this_thread::sleep_for(3s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus MoveElevatorToObjectHeight::tick()
{
  initializeSharedResources(node_);
  initializeObjectDetection(node_);

  std::string object_name = "object";
  getInput("objectOfInterest", object_name);
  double height_offset = 0.0;
  getInput("heightOffset", height_offset);

  // If previous homing/move is still active, wait briefly to avoid command fights.
  if (g_resources.gripper_status_received) {
    auto wait_start = node_->now();
    while (rclcpp::ok()) {
      bool moving = false;
      {
        std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
        moving = g_resources.is_moving;
      }
      if (!moving) {
        break;
      }
      if ((node_->now() - wait_start).seconds() > 3.0) {
        RCLCPP_WARN(node_->get_logger(),
                    "[MoveElevatorToObjectHeight] Timed out waiting for previous elevator motion to stop");
        break;
      }
      std::this_thread::sleep_for(50ms);
    }
  }

  double can_height_in_base = 0.0;
  double age_sec = 999.0;
  geometry_msgs::msg::Point raw_point;
  std::string raw_frame;
  rclcpp::Time raw_stamp;
  bool have_raw = false;
  bool have_odom = false;
  double odom_z = 0.0;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    age_sec = (node_->now() - g_detection_state.oakd_last_detection_time).seconds();
    const bool fresh = g_detection_state.oakd_can_detected &&
                       age_sec < ObjectDetectionState::DETECTION_TIMEOUT_SEC;
    if (!fresh) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] No fresh OAK-D detection for '%s' (age=%.2fs)",
                  object_name.c_str(), age_sec);
      return BT::NodeStatus::FAILURE;
    }

    raw_point = g_detection_state.oakd_detection_position_raw;
    raw_frame = g_detection_state.oakd_detection_frame_raw;
    raw_stamp = g_detection_state.oakd_last_detection_time;
    have_raw = !raw_frame.empty();

    have_odom = g_detection_state.oakd_detection_has_odom;
    odom_z = g_detection_state.oakd_detection_position_odom.z;
  }

  bool used_base_link_height = false;
  if (have_raw && g_tf_buffer) {
    try {
      geometry_msgs::msg::PointStamped raw_msg;
      raw_msg.header.frame_id = raw_frame;
      raw_msg.header.stamp = raw_stamp;
      raw_msg.point = raw_point;
      const auto in_base = g_tf_buffer->transform(raw_msg, "base_link");
      can_height_in_base = in_base.point.z;
      used_base_link_height = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] TF to base_link failed (%s), falling back to odom z",
                  ex.what());
    }
  }

  if (!used_base_link_height) {
    if (!have_odom) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] No usable OAK-D 3D height for '%s'",
                  object_name.c_str());
      return BT::NodeStatus::FAILURE;
    }
    if (!g_tf_buffer) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] TF unavailable for odom->base_link conversion");
      return BT::NodeStatus::FAILURE;
    }

    try {
      const auto tf_odom_from_base = g_tf_buffer->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);
      can_height_in_base = odom_z - tf_odom_from_base.transform.translation.z;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] Odom fallback failed (%s)",
                  ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Convert object height in base_link to elevator command in meters from home.
  // command = object_z_in_base - (gripper_center_z_in_base at home) + user_offset
  const double requested_height =
    can_height_in_base - kGripperCenterAtHomeAboveBaseLink + height_offset;

  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(requested_height)));
  msg.extender_position = -1.0;  // no extender change

  // Breakaway assist near home: when starting from hard-stop, do a small pre-lift first.
  if (g_resources.gripper_status_received && msg.elevator_position > 0.10f) {
    float current_pos = 0.0f;
    bool moving = false;
    {
      std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
      current_pos = static_cast<float>(g_resources.elevator_position);
      moving = g_resources.is_moving;
    }

    if (!moving && current_pos < 0.03f) {
      auto pre_msg = sigyn_interfaces::msg::GripperPositionCommand();
      pre_msg.elevator_position = std::min(msg.elevator_position, 0.08f);
      pre_msg.extender_position = -1.0;
      g_resources.gripper_position_pub->publish(pre_msg);

      RCLCPP_INFO(node_->get_logger(),
                  "[MoveElevatorToObjectHeight] Home breakaway pre-lift: current=%.3f -> %.3f",
                  current_pos, pre_msg.elevator_position);

      bool saw_motion = false;
      const auto wait_start = node_->now();
      while (rclcpp::ok() && (node_->now() - wait_start).seconds() < 2.5) {
        bool now_moving = false;
        {
          std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
          now_moving = g_resources.is_moving;
        }
        saw_motion = saw_motion || now_moving;
        if (saw_motion && !now_moving) {
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
    }
  }

  g_resources.gripper_position_pub->publish(msg);

  RCLCPP_INFO(node_->get_logger(),
              "[MoveElevatorToObjectHeight] '%s' object_z_base=%.3fm home_gripper_z=%.3fm offset=%.3fm (%s) -> commanding elevator=%.3fm",
              object_name.c_str(), can_height_in_base, kGripperCenterAtHomeAboveBaseLink,
              height_offset,
              used_base_link_height ? "base_link" : "odom_fallback",
              msg.elevator_position);

  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StepElevatorUp::onStart()
{
  initializeSharedResources(node_);

  double step_m = 0.01;
  getInput("stepMeters", step_m);
  const double max_travel_m = 0.91;
  const int max_steps = 400;

  double current_pos = 0.0;
  {
    std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
    if (g_resources.gripper_status_received) {
      current_pos = g_resources.elevator_position;
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

  // Command the new position directly
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(next_pos)));
  msg.extender_position = -1.0; // No change to extender
  g_resources.gripper_position_pub->publish(msg);

  last_commanded_ = next_pos;
  step_count_++;
  RCLCPP_INFO(node_->get_logger(),
              "[StepElevatorUp] Step %d: commanding elevator to %.5fm (was %.3f, max %.3f)",
              step_count_, msg.elevator_position, base_pos, max_pos);

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

// ============================================================================
// MoveElevatorAction Implementation
// ============================================================================

BT::NodeStatus MoveElevatorAction::onStart()
{
  // Get goal position from input port
  if (!getInput("goal_position", goal_position_)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveElevatorAction] Missing required input 'goal_position'");
    return BT::NodeStatus::FAILURE;
  }

  // Validate goal position
  if (goal_position_ < 0.0 || goal_position_ > 0.8999) {
    RCLCPP_ERROR(node_->get_logger(), 
                 "[MoveElevatorAction] Invalid goal position: %.4f (must be 0.0-0.8999)", 
                 goal_position_);
    return BT::NodeStatus::FAILURE;
  }

  // Create action client if not exists
  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<MoveElevatorActionType>(
      node_, "/gripper/move_elevator");
  }

  // Wait for action server
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(node_->get_logger(), 
                "[MoveElevatorAction] Action server /gripper/move_elevator not available after waiting");
    return BT::NodeStatus::FAILURE;
  }

  // Send goal
  action_state_ = ActionState::SENDING_GOAL;
  result_received_ = false;
  
  if (!sendGoal()) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveElevatorAction] Failed to send goal");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), 
              "[MoveElevatorAction] Sent goal to move elevator to %.4f meters", 
              goal_position_);
  
  return BT::NodeStatus::RUNNING;
}

bool MoveElevatorAction::sendGoal()
{
  auto goal_msg = MoveElevatorActionType::Goal();
  goal_msg.goal_position = goal_position_;

  auto send_goal_options = rclcpp_action::Client<MoveElevatorActionType>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&MoveElevatorAction::goalResponseCallback, this, std::placeholders::_1);
  
  send_goal_options.result_callback =
    std::bind(&MoveElevatorAction::resultCallback, this, std::placeholders::_1);
  
  send_goal_options.feedback_callback =
    std::bind(&MoveElevatorAction::feedbackCallback, this, 
              std::placeholders::_1, std::placeholders::_2);

  goal_handle_future_ = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  return true;
}

void MoveElevatorAction::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveElevatorAction] Goal was rejected by server");
    action_state_ = ActionState::GOAL_FAILED;
    action_result_ = BT::NodeStatus::FAILURE;
    result_received_ = true;
  } else {
    RCLCPP_INFO(node_->get_logger(), "[MoveElevatorAction] Goal accepted by server");
    action_state_ = ActionState::GOAL_ACTIVE;
    goal_handle_ = goal_handle;
  }
}

void MoveElevatorAction::resultCallback(
  const rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::WrappedResult& result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), 
                  "[MoveElevatorAction] Goal succeeded! Final position: %.4f meters",
                  result.result->final_position);
      action_state_ = ActionState::GOAL_COMPLETED;
      action_result_ = BT::NodeStatus::SUCCESS;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "[MoveElevatorAction] Goal was aborted");
      action_state_ = ActionState::GOAL_FAILED;
      action_result_ = BT::NodeStatus::FAILURE;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "[MoveElevatorAction] Goal was canceled");
      action_state_ = ActionState::GOAL_CANCELED;
      action_result_ = BT::NodeStatus::FAILURE;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "[MoveElevatorAction] Unknown result code");
      action_state_ = ActionState::GOAL_FAILED;
      action_result_ = BT::NodeStatus::FAILURE;
      break;
  }
  result_received_ = true;
}

void MoveElevatorAction::feedbackCallback(
  rclcpp_action::ClientGoalHandle<MoveElevatorActionType>::SharedPtr,
  const std::shared_ptr<const MoveElevatorActionType::Feedback> feedback)
{
  RCLCPP_DEBUG(node_->get_logger(), 
               "[MoveElevatorAction] Current position: %.4f meters", 
               feedback->current_position);
}

BT::NodeStatus MoveElevatorAction::onRunning()
{
  // Check if result received
  if (result_received_) {
    return action_result_.load();
  }

  // Still waiting for result
  return BT::NodeStatus::RUNNING;
}

void MoveElevatorAction::onHalted()
{
  // Cancel the goal if it's active
  if (goal_handle_ && action_state_ == ActionState::GOAL_ACTIVE) {
    RCLCPP_INFO(node_->get_logger(), "[MoveElevatorAction] Canceling active goal");
    action_client_->async_cancel_goal(goal_handle_);
  }
  
  action_state_ = ActionState::IDLE;
  result_received_ = false;
  goal_handle_.reset();
}

// ============================================================================
// StepElevatorUpAction - Incremental elevator movement using action server
// ============================================================================

BT::NodeStatus StepElevatorUpAction::onStart()
{
  initializeSharedResources(node_);

  // Get step size from input port
  if (!getInput("stepMeters", step_size_)) {
    step_size_ = 0.02;  // Default to 2cm
  }

  // Wait briefly for position update
  auto start_time = std::chrono::steady_clock::now();
  while (!g_resources.gripper_status_received && 
         std::chrono::steady_clock::now() - start_time < std::chrono::milliseconds(500)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (!g_resources.gripper_status_received) {
    RCLCPP_WARN(node_->get_logger(), 
                "[StepElevatorUpAction] Failed to get current elevator position from /gripper/status");
    return BT::NodeStatus::FAILURE;
  }

  // Get current position
  double current_pos;
  {
    std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
    current_pos = g_resources.elevator_position;
  }

  // Calculate target position
  target_position_ = current_pos + step_size_;

  // Validate target position
  if (target_position_ > 0.8999) {
    RCLCPP_WARN(node_->get_logger(), 
                "[StepElevatorUpAction] Target position %.4f exceeds max travel (0.8999m). Clamping.",
                target_position_);
    target_position_ = 0.8999;
  }

  if (target_position_ <= current_pos) {
    RCLCPP_WARN(node_->get_logger(), 
                "[StepElevatorUpAction] Target position %.4f not greater than current %.4f",
                target_position_, current_pos);
    return BT::NodeStatus::SUCCESS;  // Already at or past target
  }

  // Publish position command
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(target_position_)));
  msg.extender_position = -1.0; // No change to extender
  g_resources.gripper_position_pub->publish(msg);

  command_sent_ = true;
  movement_started_ = false;
  command_retried_ = false;
  command_time_ = node_->now();
  stop_seen_ = false;
  stop_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  
  RCLCPP_INFO(node_->get_logger(), 
              "[StepElevatorUpAction] Stepping up %.4fm: %.4f -> %.4f meters", 
              step_size_, current_pos, target_position_);
  
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StepElevatorUpAction::onRunning()
{
  if (!command_sent_) {
    return BT::NodeStatus::FAILURE;
  }

  // Check status snapshot
  bool still_moving;
  double current_pos;
  {
    std::lock_guard<std::mutex> lock(g_resources.gripper_mutex);
    still_moving = g_resources.is_moving;
    current_pos = g_resources.elevator_position;
  }

  const double elapsed_sec = (node_->now() - command_time_).seconds();
  const double pos_err = std::abs(current_pos - target_position_);

  // Avoid immediate SUCCESS before controller reports motion.
  if (!movement_started_) {
    if (still_moving) {
      movement_started_ = true;
      RCLCPP_DEBUG(node_->get_logger(), "[StepElevatorUpAction] Motion started");
      return BT::NodeStatus::RUNNING;
    }

    // Let command latch and status propagate.
    if (elapsed_sec < 0.20) {
      return BT::NodeStatus::RUNNING;
    }

    // Small steps may complete without an is_moving transition.
    if (pos_err <= 0.006) {
      RCLCPP_INFO(node_->get_logger(),
                  "[StepElevatorUpAction] Completed without motion-flag transition (err=%.4f)",
                  pos_err);
      return BT::NodeStatus::SUCCESS;
    }

    // Retry once in case the position command was missed.
    if (!command_retried_ && elapsed_sec >= 0.35) {
      auto retry = sigyn_interfaces::msg::GripperPositionCommand();
      retry.elevator_position = std::max(0.0f, std::min(0.8999f, static_cast<float>(target_position_)));
      retry.extender_position = -1.0;
      g_resources.gripper_position_pub->publish(retry);
      command_retried_ = true;
      command_time_ = node_->now();
      RCLCPP_WARN(node_->get_logger(),
                  "[StepElevatorUpAction] Retrying step command once (err=%.4f)", pos_err);
      return BT::NodeStatus::RUNNING;
    }

    if (elapsed_sec > 1.5) {
      RCLCPP_WARN(node_->get_logger(),
                  "[StepElevatorUpAction] Timeout waiting for motion start (err=%.4f)", pos_err);
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  if (still_moving) {
    stop_seen_ = false;
    RCLCPP_DEBUG(node_->get_logger(), "[StepElevatorUpAction] Still moving...");
    return BT::NodeStatus::RUNNING;
  } else {
    // Debounce completion: require a brief stopped period before returning SUCCESS.
    constexpr double kStopSettleSec = 0.15;
    if (!stop_seen_) {
      stop_seen_ = true;
      stop_time_ = node_->now();
      return BT::NodeStatus::RUNNING;
    }

    const double stopped_for_sec = (node_->now() - stop_time_).seconds();
    if (stopped_for_sec < kStopSettleSec) {
      return BT::NodeStatus::RUNNING;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[StepElevatorUpAction] Movement complete (target=%.4f, current=%.4f, err=%.4f)",
                target_position_, current_pos, pos_err);
    return BT::NodeStatus::SUCCESS;
  }
}

void StepElevatorUpAction::onHalted()
{
  // Reset state
  command_sent_ = false;
  movement_started_ = false;
  command_retried_ = false;
  stop_seen_ = false;
  RCLCPP_INFO(node_->get_logger(), "[StepElevatorUpAction] Halted");
}

BT::NodeStatus BackAwayFromTable::tick()
{
  initializeSharedResources(node_);
  double distance = 0.3;
  double speed = 0.1;
  getInput("distance", distance);
  getInput("speed", speed);

  if (speed <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "[BackAwayFromTable] Invalid speed %.3f", speed);
    return BT::NodeStatus::FAILURE;
  }

  const double duration_sec = std::abs(distance / speed);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = -std::abs(speed);
  g_resources.cmd_vel_nav_pub->publish(cmd);

  RCLCPP_INFO(node_->get_logger(), "[BackAwayFromTable] Backing away %.2fm at %.2fm/s",
              distance, speed);
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));

  geometry_msgs::msg::Twist stop;
  g_resources.cmd_vel_nav_pub->publish(stop);
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
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Retracting extender to home (0.0m)");

  initializeSharedResources(node_);

  // Command extender to retract to home position (0.0m)
  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = -1.0; // No change to elevator
  msg.extender_position = 0.0;  // Fully retracted
  
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Commanding extender to 0.0m (fully retracted)");
  g_resources.gripper_position_pub->publish(msg);

  // Give time for movement to complete
  std::this_thread::sleep_for(2s);
  
  RCLCPP_INFO(node_->get_logger(), "[RetractExtender] Retraction command sent");

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus RetractGripper::tick()
{
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Homing gripper assembly (elevator and extender)");
  initializeSharedResources(node_);

  auto msg = std_msgs::msg::Empty();
  g_resources.gripper_home_pub->publish(msg);
  
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Homing command sent");

  // Give time for homing sequence to complete
  std::this_thread::sleep_for(3s);
  
  RCLCPP_INFO(node_->get_logger(), "[RetractGripper] Homing complete");

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus OpenGripper::tick()
{
  initializeSharedResources(node_);

  geometry_msgs::msg::Twist msg;
  msg.linear.x = -1000.0;  // Open gripper
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  
  RCLCPP_INFO(node_->get_logger(), "[OpenGripper] Sending cmd_vel_testicle_twister: linear.x=-1000.0 (open)");
  g_resources.twister_pub->publish(msg);
  
  std::this_thread::sleep_for(500ms);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CloseGripperAroundCan::tick()
{
  initializeSharedResources(node_);
  double can_diameter;
  getInput("canDiameter", can_diameter);
  
  RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] START: can_diameter=%.4f", can_diameter);
  
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 1000.0;  // Close gripper
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  RCLCPP_INFO(node_->get_logger(), "[CloseGripperAroundCan] Sending cmd_vel_testicle_twister: linear.x=1000.0 (close)");
  
  g_resources.twister_pub->publish(msg);

  std::this_thread::sleep_for(1500ms);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExtendTowardsCan::tick()
{
  initializeSharedResources(node_);
  std::string object_name;
  getInput("objectOfInterest", object_name);
  
  initializeObjectDetection(node_);
  
  geometry_msgs::msg::Point det_raw;
  std::string det_frame;
  rclcpp::Time det_stamp;
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    if (!g_detection_state.oakd_can_detected) {
      RCLCPP_WARN(node_->get_logger(), "[ExtendTowardsCan] No OAK-D detection available");
      return BT::NodeStatus::FAILURE;
    }
    det_raw = g_detection_state.oakd_detection_position_raw;
    det_frame = g_detection_state.oakd_detection_frame_raw;
    det_stamp = g_detection_state.oakd_last_detection_time;
  }

  const double oakd_age = (node_->now() - det_stamp).seconds();
  if (oakd_age >= ObjectDetectionState::DETECTION_TIMEOUT_SEC) {
    RCLCPP_WARN(node_->get_logger(),
                "[ExtendTowardsCan] OAK-D detection stale (%.2fs)", oakd_age);
    return BT::NodeStatus::FAILURE;
  }

  if (det_frame.empty() || !g_tf_buffer) {
    RCLCPP_WARN(node_->get_logger(),
                "[ExtendTowardsCan] OAK-D frame/TF not ready (frame='%s')",
                det_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PointStamped det_msg;
  det_msg.header.frame_id = det_frame;
  det_msg.header.stamp = rclcpp::Time(0);
  det_msg.point = det_raw;

  geometry_msgs::msg::PointStamped det_in_base;
  try {
    const auto tf_base_from_det = g_tf_buffer->lookupTransform(
      "base_link", det_msg.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(det_msg, det_in_base, tf_base_from_det);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(),
                "[ExtendTowardsCan] TF base_link<-'%s' failed: %s",
                det_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // User-calibrated geometry:
  // - Robot points straight at can.
  // - Open/retracted gripper target surface is at x=0.23m in base_link.
  // Therefore required extension is the forward gap from this surface to can front x.
  constexpr double kGripperTargetSurfaceXBase = 0.23;
  constexpr double kExtensionTrimMeters = 0.04;  // temporary calibration trim
  const double can_front_x_base = det_in_base.point.x;
  double extension_distance = can_front_x_base - kGripperTargetSurfaceXBase - kExtensionTrimMeters;
  
  RCLCPP_INFO(node_->get_logger(),
              "[ExtendTowardsCan] OAK-D base_link can_x=%.3fm, target_surface_x=%.3fm, trim=%.3fm -> requested extension=%.3fm (obj='%s')",
              can_front_x_base, kGripperTargetSurfaceXBase, kExtensionTrimMeters,
              extension_distance, object_name.c_str());
              
  // Command extender to the calculated position
  float current_extender = 0.0f;
  {
    std::lock_guard<std::mutex> sensor_lock(g_sensor_mutex);
    current_extender = static_cast<float>(g_sensor_state.extender_position);
  }

  auto msg = sigyn_interfaces::msg::GripperPositionCommand();
  msg.elevator_position = -1.0; // No change to elevator
  msg.extender_position = std::max(0.0f, std::min(0.3418f, static_cast<float>(extension_distance)));

  // Safety: this action is named "ExtendTowardsCan", so never command a retract here.
  if (msg.extender_position < current_extender) {
    RCLCPP_WARN(node_->get_logger(),
                "[ExtendTowardsCan] Computed target %.4fm is behind current %.4fm; holding current to avoid retract",
                msg.extender_position, current_extender);
    msg.extender_position = current_extender;
  }

  const char* direction = (msg.extender_position > current_extender + 1e-4f) ? "EXTEND" :
                          (msg.extender_position < current_extender - 1e-4f) ? "RETRACT" : "HOLD";
  
  g_resources.gripper_position_pub->publish(msg);
  RCLCPP_INFO(node_->get_logger(),
              "[ExtendTowardsCan] Commanded extender to %.4fm (current=%.4fm, %s; clamped to 0.0-0.3418)",
              msg.extender_position, current_extender, direction);

  {
    std::lock_guard<std::mutex> lock(g_sensor_mutex);
    g_sensor_state.last_extender_command = msg.extender_position;
  }

  std::this_thread::sleep_for(2s);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus AdjustExtenderToCenterCan::tick()
{
  std::string object_name;
  getInput("objectOfInterest", object_name);
  double max_center_error_deg = 2.0;
  getInput("max_center_error_deg", max_center_error_deg);
  
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
  // Apply target offset to compensate for camera/gripper physical alignment
  const double x_offset = det_point.x - ObjectDetectionState::PI_HORIZONTAL_TARGET_OFFSET;
  const double distance = det_point.z;
  if (distance <= 1e-3) {
    RCLCPP_WARN(node_->get_logger(),
                "[AdjustExtenderToCenterCan] Invalid detection distance z=%.4f",
                distance);
    return BT::NodeStatus::FAILURE;
  }

  if (max_center_error_deg <= 0.0) {
    max_center_error_deg = 2.0;
  }

  const double max_center_error_rad = max_center_error_deg * M_PI / 180.0;
  const double angle_error_rad = std::atan2(x_offset, distance);
  
  // If already centered, we're done
  if (std::abs(angle_error_rad) <= max_center_error_rad) {
    RCLCPP_INFO(node_->get_logger(),
                "[AdjustExtenderToCenterCan] Already centered: angle_err=%.2fdeg <= %.2fdeg",
                angle_error_rad * 180.0 / M_PI, max_center_error_deg);
    return BT::NodeStatus::SUCCESS;
  }

  // Calculate rotation needed
  // Positive x_offset means can is to the right (Camera Frame X+).
  // Robot Base Frame: Positive Z is Left (CCW), Negative Z is Right (CW).
  // Therefore, we need a NEGATIVE angular_z for a POSITIVE x_offset.
  
  // Use distance to calculate precise rotation angle needed:

  // We desire to zero this angle error.
  
  // Damped incremental correction to avoid overshoot with low camera frame rate.
  // We intentionally DO NOT apply full error in one pulse.
  constexpr double kNominalPulseSec = 0.20;
  constexpr double kCorrectionGain = 0.45;
  constexpr double kMaxCommandAngleRad = 4.0 * M_PI / 180.0;  // 4 deg max per tick

  const double target_angle_rad = -angle_error_rad * kCorrectionGain;
  const double commanded_angle_rad = std::clamp(target_angle_rad, -kMaxCommandAngleRad, kMaxCommandAngleRad);

  double duration_sec = kNominalPulseSec;
  double required_omega = commanded_angle_rad / duration_sec;
  
  RCLCPP_INFO(node_->get_logger(), 
            "[AdjustExtenderToCenterCan] Plan: err=%.4fm, dist=%.3fm -> angle_err=%.4f rad (%.2fdeg, limit=%.2fdeg), cmd_angle=%.2fdeg. Need omega=%.3f rad/s for %.2fs", 
             x_offset, distance, angle_error_rad, angle_error_rad * 180.0 / M_PI, max_center_error_deg,
             commanded_angle_rad * 180.0 / M_PI, required_omega, duration_sec);
  
  const double min_angular = 0.15; // Minimum robust rotation speed
  double angular_z = 0.0;
  
    if (std::abs(required_omega) < min_angular) {
      // If required velocity is too low for the robot to move reliably,
      // use min velocity and adjust pulse duration to achieve commanded angle.
      double direction = (required_omega >= 0) ? 1.0 : -1.0;
      angular_z = direction * min_angular;
      
      // New duration: time = angle / velocity
      duration_sec = std::abs(commanded_angle_rad / min_angular);
      
      RCLCPP_INFO(node_->get_logger(), 
        "[AdjustExtenderToCenterCan] Micro-adjust: Required %.3f < min %.3f. Boosting to %.3f, duration %.3fs", 
         required_omega, min_angular, angular_z, duration_sec);

      // Ensure we don't have excessively short pulses
        if (duration_sec < 0.04) {
          duration_sec = 0.04;
      }
  } else {
      angular_z = required_omega;
  }

  // Cap at maximum angular velocity
  const double max_angular = 1.0; 
  if (std::abs(angular_z) > max_angular) {
      angular_z = (angular_z > 0) ? max_angular : -max_angular;
      // Recompute duration to preserve commanded angle with capped speed.
      duration_sec = std::abs(commanded_angle_rad / max_angular);
      RCLCPP_WARN(node_->get_logger(), 
            "[AdjustExtenderToCenterCan] Saturated: %.3f > max %.3f. Capped at %.3f", 
             std::abs(required_omega), max_angular, angular_z);
  }

    // Keep pulses bounded to reduce open-loop rotation drift.
    duration_sec = std::clamp(duration_sec, 0.04, 0.40);

  initializeSharedResources(node_);
  
  // Publish twist command to rotate robot
  // Use cmd_vel_smoothed to bypass velocity smoother latency for small movements
  // and go directly to the twist multiplexer.
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = angular_z;
  
  long duration_ms = static_cast<long>(duration_sec * 1000);
  
  g_resources.cmd_vel_smoothed_pub->publish(twist);
  
  RCLCPP_INFO(node_->get_logger(), 
              "[AdjustExtenderToCenterCan] EXECUTING: z=%.3f rad/s for %ldms", 
              angular_z, duration_ms);

  // Sleep for the calculated duration to allow movement
  std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
  
  // Stop rotation
  twist.angular.z = 0.0;
  g_resources.cmd_vel_smoothed_pub->publish(twist);
  
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
  RCLCPP_INFO(node_->get_logger(), "### EXECUTING FROM REAL ROBOT (bt_nodes_real.cpp) ###");
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
  RCLCPP_INFO(node_->get_logger(), "[WaitForDetection] Waiting for camera system to come online (heartbeat)...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDetection::onRunning()
{
  initializeObjectDetection(node_);
  
  bool alive = false;
  double age = 999.0;
  
  {
      std::lock_guard<std::mutex> lock(g_detection_mutex);
      alive = g_detection_state.oakd_alive;
      if (alive) {
          age = (node_->now() - g_detection_state.oakd_last_heartbeat_time).seconds();
      }
  }

  if (alive && age < 1.0) {
      RCLCPP_INFO(node_->get_logger(), "[WaitForDetection] OAK-D system is online (heartbeat age=%.3fs)", age);
      return BT::NodeStatus::SUCCESS;
  }

  // Keep the tree alive while we wait
  std::this_thread::sleep_for(100ms);

  const double elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > 15.0) {
    RCLCPP_ERROR(node_->get_logger(), "[WaitForDetection] Timed out waiting for OAK-D heartbeat (%.1fs)", elapsed);
    return BT::NodeStatus::FAILURE;
  }

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, 
        "[WaitForDetection] Still waiting for OAK-D... (elapsed=%.1fs)", elapsed);

  return BT::NodeStatus::RUNNING;
}

void WaitForDetection::onHalted()
{
  // Nothing to clean up.
}

BT::NodeStatus WaitForNewOAKDFrame::onStart()
{
  // Capture the current OAK-D detection timestamp
  initializeObjectDetection(node_);
  
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    last_frame_timestamp_ = g_detection_state.oakd_last_detection_time;
  }
  
  // Immediately check if a new frame is available (might already be newer)
  return onRunning();
}

BT::NodeStatus WaitForNewOAKDFrame::onRunning()
{
  initializeObjectDetection(node_);
  
  rclcpp::Time current_detection_timestamp;
  bool system_alive = false;
  
  {
    std::lock_guard<std::mutex> lock(g_detection_mutex);
    system_alive = g_detection_state.oakd_alive;
    if (system_alive && g_detection_state.oakd_can_detected) {
      current_detection_timestamp = g_detection_state.oakd_last_detection_time;

      // Check if we have a new detection sample
      if (current_detection_timestamp.nanoseconds() > last_frame_timestamp_.nanoseconds()) {
        last_frame_timestamp_ = current_detection_timestamp;
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  // Optional: Add a timeout to prevent infinite blocking if camera dies mid-rotation
  // But strictly, WaitForDetection should have handled the "alive" check.
  
  // No new frame yet, return RUNNING (BT will tick us again soon)
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
