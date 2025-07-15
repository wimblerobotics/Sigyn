#include <memory>
#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"

namespace sigyn_house_patroller {

class LocalizationCorrectorNode : public rclcpp::Node {
public:
  explicit LocalizationCorrectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("localization_corrector", options),
        last_pose_update_(std::chrono::steady_clock::now()),
        last_scan_update_(std::chrono::steady_clock::now()) {
    
    // Declare parameters
    declare_parameter("monitoring_frequency", 2.0);
    declare_parameter("pose_timeout", 30.0);
    declare_parameter("scan_timeout", 30.0);
    declare_parameter("pose_topic", "/amcl_pose");
    declare_parameter("scan_topic", "/scan");
    declare_parameter("map_topic", "/map");
    declare_parameter("initialpose_topic", "/initialpose");
    declare_parameter("map_frame", "map");
    declare_parameter("robot_frame", "base_link");
    declare_parameter("odom_frame", "odom");
    declare_parameter("pose_covariance_threshold", 0.5);
    declare_parameter("position_jump_threshold", 1.0);  // meters
    declare_parameter("orientation_jump_threshold", 0.5);  // radians
    declare_parameter("scan_match_threshold", 0.8);
    declare_parameter("correction_cooldown", 60.0);  // seconds
    declare_parameter("enable_auto_correction", true);
    declare_parameter("min_scan_points", 100);
    
    // Get parameters
    monitoring_frequency_ = get_parameter("monitoring_frequency").as_double();
    pose_timeout_ = get_parameter("pose_timeout").as_double();
    scan_timeout_ = get_parameter("scan_timeout").as_double();
    pose_topic_ = get_parameter("pose_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    initialpose_topic_ = get_parameter("initialpose_topic").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    pose_covariance_threshold_ = get_parameter("pose_covariance_threshold").as_double();
    position_jump_threshold_ = get_parameter("position_jump_threshold").as_double();
    orientation_jump_threshold_ = get_parameter("orientation_jump_threshold").as_double();
    scan_match_threshold_ = get_parameter("scan_match_threshold").as_double();
    correction_cooldown_ = get_parameter("correction_cooldown").as_double();
    enable_auto_correction_ = get_parameter("enable_auto_correction").as_bool();
    min_scan_points_ = get_parameter("min_scan_points").as_int();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize state
    pose_healthy_ = true;
    scan_healthy_ = true;
    localization_quality_ = 1.0;
    last_correction_time_ = std::chrono::steady_clock::now() - 
                           std::chrono::seconds(static_cast<int>(correction_cooldown_));
    
    // Publishers
    threat_alert_pub_ = create_publisher<msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    health_pub_ = create_publisher<msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, rclcpp::QoS(10).reliable());
    
    // Subscribers
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalizationCorrectorNode::PoseCallback, this, std::placeholders::_1));
    
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&LocalizationCorrectorNode::ScanCallback, this, std::placeholders::_1));
    
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      std::bind(&LocalizationCorrectorNode::MapCallback, this, std::placeholders::_1));
    
    // Timers
    monitoring_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_frequency_)),
      std::bind(&LocalizationCorrectorNode::MonitoringTimerCallback, this));
    
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&LocalizationCorrectorNode::HealthTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Localization corrector started - Auto correction: %s", 
                enable_auto_correction_ ? "enabled" : "disabled");
  }

private:
  void PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // Store previous pose for jump detection
    if (current_pose_) {
      previous_pose_ = *current_pose_;
    }
    
    current_pose_ = msg;
    last_pose_update_ = std::chrono::steady_clock::now();
    
    // Check pose quality
    CheckPoseQuality();
    
    // Check for pose jumps
    if (previous_pose_) {
      CheckPoseJumps();
    }
    
    // Update pose history
    UpdatePoseHistory();
  }
  
  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_scan_ = msg;
    last_scan_update_ = std::chrono::steady_clock::now();
    
    // Check scan quality
    CheckScanQuality();
    
    // Perform scan matching if we have a map and pose
    if (current_map_ && current_pose_) {
      CheckScanMatch();
    }
  }
  
  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_map_ = msg;
    RCLCPP_INFO(get_logger(), "Map received - Size: %dx%d, Resolution: %.3f", 
                msg->info.width, msg->info.height, msg->info.resolution);
  }
  
  void MonitoringTimerCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    auto now = std::chrono::steady_clock::now();
    
    // Check pose communication health
    auto pose_time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
      now - last_pose_update_).count();
    
    if (pose_time_since_update > pose_timeout_) {
      pose_healthy_ = false;
      
      if (pose_time_since_update > pose_timeout_ * 2) {
        SendCommunicationAlert("pose", pose_time_since_update);
      }
    } else {
      pose_healthy_ = true;
    }
    
    // Check scan communication health
    auto scan_time_since_update = std::chrono::duration_cast<std::chrono::seconds>(
      now - last_scan_update_).count();
    
    if (scan_time_since_update > scan_timeout_) {
      scan_healthy_ = false;
      
      if (scan_time_since_update > scan_timeout_ * 2) {
        SendCommunicationAlert("scan", scan_time_since_update);
      }
    } else {
      scan_healthy_ = true;
    }
    
    // Update overall localization quality
    UpdateLocalizationQuality();
  }
  
  void CheckPoseQuality() {
    if (!current_pose_) {
      return;
    }
    
    // Check covariance matrix for pose uncertainty
    const auto& cov = current_pose_->pose.covariance;
    
    // Calculate position uncertainty (x, y variances)
    double pos_uncertainty = std::sqrt(cov[0] + cov[7]);  // sqrt(var_x + var_y)
    
    // Calculate orientation uncertainty
    double orient_uncertainty = std::sqrt(cov[35]);  // sqrt(var_yaw)
    
    if (pos_uncertainty > pose_covariance_threshold_ || 
        orient_uncertainty > pose_covariance_threshold_) {
      
      // Send localization quality alert
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = map_frame_;
      alert.threat_id = "poor_localization";
      alert.threat_type = "localization_quality";
      alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
      alert.description = "Poor localization quality - Position uncertainty: " + 
                         std::to_string(pos_uncertainty) + "m, " +
                         "Orientation uncertainty: " + std::to_string(orient_uncertainty) + "rad";
      alert.confidence = 1.0;
      alert.timestamp = this->now();
      alert.sensor_data = "{\"position_uncertainty\": " + std::to_string(pos_uncertainty) + 
                         ", \"orientation_uncertainty\": " + std::to_string(orient_uncertainty) + "}";
      
      threat_alert_pub_->publish(alert);
      
      RCLCPP_WARN(get_logger(), "Poor localization quality detected");
      
      // Attempt correction if enabled
      if (enable_auto_correction_) {
        AttemptLocalizationCorrection();
      }
    }
  }
  
  void CheckPoseJumps() {
    if (!current_pose_ || !previous_pose_) {
      return;
    }
    
    // Calculate position jump
    double dx = current_pose_->pose.pose.position.x - previous_pose_->pose.pose.position.x;
    double dy = current_pose_->pose.pose.position.y - previous_pose_->pose.pose.position.y;
    double position_jump = std::sqrt(dx*dx + dy*dy);
    
    // Calculate orientation jump
    tf2::Quaternion current_q, previous_q;
    tf2::fromMsg(current_pose_->pose.pose.orientation, current_q);
    tf2::fromMsg(previous_pose_->pose.pose.orientation, previous_q);
    
    double orientation_jump = std::abs(tf2::getYaw(current_q) - tf2::getYaw(previous_q));
    
    // Normalize orientation jump to [-pi, pi]
    while (orientation_jump > M_PI) orientation_jump -= 2*M_PI;
    while (orientation_jump < -M_PI) orientation_jump += 2*M_PI;
    orientation_jump = std::abs(orientation_jump);
    
    if (position_jump > position_jump_threshold_ || 
        orientation_jump > orientation_jump_threshold_) {
      
      // Send pose jump alert
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = map_frame_;
      alert.threat_id = "pose_jump";
      alert.threat_type = "localization_jump";
      alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
      alert.description = "Pose jump detected - Position: " + std::to_string(position_jump) + 
                         "m, Orientation: " + std::to_string(orientation_jump) + "rad";
      alert.confidence = 1.0;
      alert.timestamp = this->now();
      alert.sensor_data = "{\"position_jump\": " + std::to_string(position_jump) + 
                         ", \"orientation_jump\": " + std::to_string(orientation_jump) + "}";
      
      threat_alert_pub_->publish(alert);
      
      RCLCPP_WARN(get_logger(), "Pose jump detected: %.2fm, %.2frad", 
                  position_jump, orientation_jump);
      
      // Attempt correction if enabled
      if (enable_auto_correction_) {
        AttemptLocalizationCorrection();
      }
    }
  }
  
  void CheckScanQuality() {
    if (!current_scan_) {
      return;
    }
    
    // Count valid scan points
    int valid_points = 0;
    for (const auto& range : current_scan_->ranges) {
      if (range >= current_scan_->range_min && range <= current_scan_->range_max) {
        valid_points++;
      }
    }
    
    if (valid_points < min_scan_points_) {
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = robot_frame_;
      alert.threat_id = "poor_scan_quality";
      alert.threat_type = "sensor_quality";
      alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
      alert.description = "Poor scan quality - Valid points: " + 
                         std::to_string(valid_points) + "/" + 
                         std::to_string(current_scan_->ranges.size());
      alert.confidence = 1.0;
      alert.timestamp = this->now();
      alert.sensor_data = "{\"valid_points\": " + std::to_string(valid_points) + 
                         ", \"total_points\": " + std::to_string(current_scan_->ranges.size()) + "}";
      
      threat_alert_pub_->publish(alert);
      
      RCLCPP_WARN(get_logger(), "Poor scan quality: %d valid points", valid_points);
    }
  }
  
  void CheckScanMatch() {
    // Simple scan matching check - in a real implementation, this would use
    // more sophisticated algorithms like ICP or scan matching libraries
    
    if (!current_pose_ || !current_scan_ || !current_map_) {
      return;
    }
    
    // This is a placeholder for scan matching logic
    // In practice, you would:
    // 1. Transform scan to map coordinates
    // 2. Compare with expected map features
    // 3. Calculate match score
    
    // For now, just check if we have reasonable data
    double match_score = 0.9;  // Placeholder
    
    if (match_score < scan_match_threshold_) {
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = map_frame_;
      alert.threat_id = "poor_scan_match";
      alert.threat_type = "scan_match_failure";
      alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
      alert.description = "Poor scan match - Score: " + std::to_string(match_score);
      alert.confidence = 1.0;
      alert.timestamp = this->now();
      alert.sensor_data = "{\"match_score\": " + std::to_string(match_score) + "}";
      
      threat_alert_pub_->publish(alert);
      
      RCLCPP_WARN(get_logger(), "Poor scan match detected: %.2f", match_score);
      
      // Attempt correction if enabled
      if (enable_auto_correction_) {
        AttemptLocalizationCorrection();
      }
    }
  }
  
  void UpdatePoseHistory() {
    if (!current_pose_) {
      return;
    }
    
    PoseHistoryEntry entry;
    entry.timestamp = std::chrono::steady_clock::now();
    entry.pose = current_pose_->pose.pose;
    
    pose_history_.push_back(entry);
    
    // Keep only recent history
    const size_t max_history = 100;
    if (pose_history_.size() > max_history) {
      pose_history_.erase(pose_history_.begin());
    }
  }
  
  void UpdateLocalizationQuality() {
    // Calculate overall localization quality based on multiple factors
    double quality = 1.0;
    
    // Factor 1: Communication health
    if (!pose_healthy_ || !scan_healthy_) {
      quality *= 0.5;
    }
    
    // Factor 2: Pose uncertainty
    if (current_pose_) {
      const auto& cov = current_pose_->pose.covariance;
      double pos_uncertainty = std::sqrt(cov[0] + cov[7]);
      quality *= std::max(0.1, 1.0 - pos_uncertainty);
    }
    
    // Factor 3: Scan quality
    if (current_scan_) {
      int valid_points = 0;
      for (const auto& range : current_scan_->ranges) {
        if (range >= current_scan_->range_min && range <= current_scan_->range_max) {
          valid_points++;
        }
      }
      double scan_quality = std::min(1.0, static_cast<double>(valid_points) / min_scan_points_);
      quality *= scan_quality;
    }
    
    localization_quality_ = quality;
  }
  
  void AttemptLocalizationCorrection() {
    auto now = std::chrono::steady_clock::now();
    auto time_since_correction = std::chrono::duration_cast<std::chrono::seconds>(
      now - last_correction_time_).count();
    
    if (time_since_correction < correction_cooldown_) {
      return;  // Still in cooldown period
    }
    
    // Attempt to reinitialize localization
    // This is a simple approach - in practice, you might use more sophisticated methods
    
    if (current_pose_) {
      // Publish current pose with high uncertainty to trigger relocalization
      geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
      init_pose.header.stamp = this->now();
      init_pose.header.frame_id = map_frame_;
      init_pose.pose.pose = current_pose_->pose.pose;
      
      // Set high covariance to trigger particle filter spreading
      for (int i = 0; i < 36; ++i) {
        init_pose.pose.covariance[i] = 0.0;
      }
      init_pose.pose.covariance[0] = 0.5;   // x variance
      init_pose.pose.covariance[7] = 0.5;   // y variance
      init_pose.pose.covariance[35] = 0.1;  // yaw variance
      
      initialpose_pub_->publish(init_pose);
      
      last_correction_time_ = now;
      
      RCLCPP_INFO(get_logger(), "Attempted localization correction");
      
      // Send correction alert
      msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = map_frame_;
      alert.threat_id = "localization_correction";
      alert.threat_type = "localization_correction";
      alert.severity = msg::ThreatAlert::SEVERITY_INFO;
      alert.description = "Attempted automatic localization correction";
      alert.confidence = 0.8;
      alert.timestamp = this->now();
      alert.sensor_data = "{}";
      
      threat_alert_pub_->publish(alert);
    }
  }
  
  void SendCommunicationAlert(const std::string& sensor_type, long seconds_since_update) {
    msg::ThreatAlert alert;
    alert.header.stamp = this->now();
    alert.header.frame_id = robot_frame_;
    alert.threat_id = sensor_type + "_comm_loss";
    alert.threat_type = "communication_failure";
    alert.severity = msg::ThreatAlert::SEVERITY_WARNING;
    alert.description = sensor_type + " communication lost for " + 
                       std::to_string(seconds_since_update) + " seconds";
    alert.confidence = 1.0;
    alert.timestamp = this->now();
    alert.sensor_data = "{}";
    
    threat_alert_pub_->publish(alert);
  }
  
  void HealthTimerCallback() {
    msg::SystemHealth health;
    health.header.stamp = this->now();
    health.header.frame_id = robot_frame_;
    health.overall_health = localization_quality_;
    health.navigation_health = localization_quality_;
    health.detection_health = (pose_healthy_ && scan_healthy_) ? 1.0 : 0.0;
    
    health.component_status.push_back("localization_corrector: " + 
                                     (pose_healthy_ && scan_healthy_ ? "OK" : "ERROR"));
    health.component_status.push_back("localization_quality: " + 
                                     std::to_string(localization_quality_));
    health.component_status.push_back("pose_healthy: " + 
                                     (pose_healthy_ ? "OK" : "ERROR"));
    health.component_status.push_back("scan_healthy: " + 
                                     (scan_healthy_ ? "OK" : "ERROR"));
    health.component_status.push_back("auto_correction: " + 
                                     (enable_auto_correction_ ? "enabled" : "disabled"));
    
    health_pub_->publish(health);
  }
  
  struct PoseHistoryEntry {
    std::chrono::steady_clock::time_point timestamp;
    geometry_msgs::msg::Pose pose;
  };
  
  // Parameters
  double monitoring_frequency_;
  double pose_timeout_;
  double scan_timeout_;
  std::string pose_topic_;
  std::string scan_topic_;
  std::string map_topic_;
  std::string initialpose_topic_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string odom_frame_;
  double pose_covariance_threshold_;
  double position_jump_threshold_;
  double orientation_jump_threshold_;
  double scan_match_threshold_;
  double correction_cooldown_;
  bool enable_auto_correction_;
  int min_scan_points_;
  
  // State
  bool pose_healthy_;
  bool scan_healthy_;
  double localization_quality_;
  std::chrono::steady_clock::time_point last_pose_update_;
  std::chrono::steady_clock::time_point last_scan_update_;
  std::chrono::steady_clock::time_point last_correction_time_;
  
  // Data
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr previous_pose_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  std::vector<PoseHistoryEntry> pose_history_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // ROS2 interfaces
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_alert_pub_;
  rclcpp::Publisher<msg::SystemHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  
  std::mutex data_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::LocalizationCorrectorNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("localization_corrector"), "Starting localization corrector node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
