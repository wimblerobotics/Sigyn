#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"

namespace sigyn_house_patroller {

class LocalizationCorrectorNode : public rclcpp::Node {
public:
  explicit LocalizationCorrectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("localization_corrector", options)
        {
    
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
    last_correction_time_ = this->get_clock()->now() - rclcpp::Duration::from_seconds(correction_cooldown_);
    last_pose_update_ = this->get_clock()->now();
    last_scan_update_ = this->get_clock()->now();
    last_jump_threat_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_scan_quality_threat_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Publishers
    threat_alert_pub_ = create_publisher<sigyn_house_patroller::msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    health_pub_ = create_publisher<sigyn_house_patroller::msg::SystemHealth>(
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
    // Add missing member variables
    std::unordered_map<std::string, rclcpp::Time> last_comm_alerts_;
    std::unordered_map<std::string, rclcpp::Time> last_localization_alerts_;

  void PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg;
    pose_healthy_ = true;
    last_pose_update_ = this->get_clock()->now();

    if (previous_pose_) {
      CheckPoseJumps();
    }

    CheckPoseQuality();
    
    if (previous_pose_ == nullptr) {
      previous_pose_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    }
    *previous_pose_ = *current_pose_;
  }

  void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    current_scan_ = msg;
    last_scan_update_ = this->get_clock()->now();
    
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
    
    auto now = this->get_clock()->now();
    
    // Check pose communication health
    auto pose_time_since_update = (now - last_pose_update_).seconds();
    
    if (pose_time_since_update > pose_timeout_) {
      pose_healthy_ = false;
      
      if (pose_time_since_update > pose_timeout_ * 2) {
        SendCommunicationAlert("pose", last_pose_update_);
      }
    } else {
      pose_healthy_ = true;
    }
    
    // Check scan communication health
    auto scan_time_since_update = (now - last_scan_update_).seconds();
    
    if (scan_time_since_update > scan_timeout_) {
      scan_healthy_ = false;
      
      if (scan_time_since_update > scan_timeout_ * 2) {
        SendCommunicationAlert("scan", last_scan_update_);
      }
    } else {
      scan_healthy_ = true;
    }
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
      sigyn_house_patroller::msg::ThreatAlert alert;
      alert.header.stamp = this->now();
      alert.header.frame_id = map_frame_;
      alert.threat_id = "poor_localization";
      alert.threat_type = "localization_quality";
      alert.severity_level = alert.SEVERITY_WARNING;
      alert.description = "Poor localization quality - Position uncertainty: " + 
                         std::to_string(pos_uncertainty) + "m, " +
                         "Orientation uncertainty: " + std::to_string(orient_uncertainty) + "rad";
      alert.confidence = 1.0;
      alert.sensor_data_json = "{\"position_uncertainty\": " + std::to_string(pos_uncertainty) + 
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
    if (!previous_pose_) return;

    double position_diff =
        std::sqrt(std::pow(current_pose_->pose.pose.position.x - previous_pose_->pose.pose.position.x, 2) +
                  std::pow(current_pose_->pose.pose.position.y - previous_pose_->pose.pose.position.y, 2));

    tf2::Quaternion current_q, previous_q;
    tf2::fromMsg(current_pose_->pose.pose.orientation, current_q);
    tf2::fromMsg(previous_pose_->pose.pose.orientation, previous_q);

    double orientation_diff = current_q.angleShortestPath(previous_q);

    if (position_diff > position_jump_threshold_ || orientation_diff > orientation_jump_threshold_) {
      SendLocalizationAlert("pose_jump", position_diff,
                            "Pose jump detected.",
                            "{\"position_diff\": " + std::to_string(position_diff) +
                            ", \"orientation_diff\": " + std::to_string(orientation_diff) + "}");
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
      scan_healthy_ = false;
      SendLocalizationAlert("low_scan_points", valid_points,
                            "Low number of valid scan points.",
                            "{\"valid_points\": " + std::to_string(valid_points) + "}");
    } else {
      scan_healthy_ = true;
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
      localization_quality_ = std::min(localization_quality_, match_score);
      SendLocalizationAlert("poor_scan_match", scan_match_score_,
                            "Poor scan match against map.",
                            "{\"scan_match_score\": " + std::to_string(scan_match_score_) + "}");
      
      // Attempt to correct localization if score is very low
      if (scan_match_score_ < 0.5) {
        AttemptLocalizationCorrection();
      }
    }
  }
  
  void AttemptLocalizationCorrection() {
    if (!enable_auto_correction_) return;

    auto now = this->get_clock()->now();
    if ((now - last_correction_time_).seconds() < correction_cooldown_) {
        return; // Cooldown active
    }

    RCLCPP_WARN(get_logger(), "Attempting to re-localize robot due to poor localization quality.");
    RelocalizeRobot();
    last_correction_time_ = now;
  }

  void RelocalizeRobot() {
    if (!current_pose_ || !enable_auto_correction_) {
      return;
    }
    
    // Publish current pose with zero covariance to trigger immediate relocalization
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = this->now();
    init_pose.header.frame_id = map_frame_;
    init_pose.pose.pose = current_pose_->pose.pose;
    
    // Set covariance to zero to indicate high certainty
    for (int i = 0; i < 36; ++i) {
      init_pose.pose.covariance[i] = 0.0;
    }
    
    initialpose_pub_->publish(init_pose);
    
    RCLCPP_INFO(get_logger(), "Relocalization triggered");

    // For now, we just publish a threat to notify the system
    SendLocalizationAlert("relocalization_needed", 1.0,
                          "Robot requires relocalization.", "{}");
  }

  void HealthTimerCallback() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    sigyn_house_patroller::msg::SystemHealth health_msg;
    health_msg.header.stamp = this->get_clock()->now();
    
    bool is_healthy = pose_healthy_ && scan_healthy_ && (localization_quality_ > 0.5);
    
    health_msg.overall_health = is_healthy ? 1 : 2; // 1=HEALTHY, 2=DEGRADED
    
    std::string message = "Pose: " + std::string(pose_healthy_ ? "OK" : "FAIL") +
                          ", Scan: " + std::string(scan_healthy_ ? "OK" : "FAIL") +
                          ", Quality: " + std::to_string(localization_quality_);
    health_msg.system_status_description = message;
    
    health_msg.component_names.push_back(this->get_name());
    health_msg.component_health_status.push_back(is_healthy ? 1 : 3); // 1=HEALTHY, 3=UNHEALTHY
    health_msg.component_descriptions.push_back(message);
    health_msg.last_update_times.push_back(this->get_clock()->now());

    health_pub_->publish(health_msg);
  }

  void SendCommunicationAlert(const std::string& sensor_name, const rclcpp::Time& last_update_time) {
    auto now = this->get_clock()->now();
    auto time_since_update = (now - last_update_time).seconds();

    // Check cooldown
    auto& last_alert_time = last_comm_alerts_[sensor_name];
    if ((now - last_alert_time).seconds() < correction_cooldown_) {
        return;
    }

    sigyn_house_patroller::msg::ThreatAlert alert;
    alert.header.stamp = now;
    alert.threat_id = "comm_loss_" + sensor_name + "_" + std::to_string(now.seconds());
    alert.threat_type = "communication_loss";
    alert.severity_level = 3; // CRITICAL
    alert.description = "Communication lost with " + sensor_name + " sensor. No data for " + 
                       std::to_string(time_since_update) + " seconds.";
    alert.confidence = 1.0;
    alert.sensor_data_json = "{\"sensor\": \"" + sensor_name + "\"}";
    
    threat_alert_pub_->publish(alert);
    last_alert_time = now;
  }

  void SendLocalizationAlert(const std::string& alert_type, double value,
                             const std::string& description, const std::string& sensor_data) {
    auto now = this->get_clock()->now();

    // Check cooldown for this specific alert type
    auto& last_alert_time = last_localization_alerts_[alert_type];
    if ((now - last_alert_time).seconds() < correction_cooldown_) {
        return;
    }

    sigyn_house_patroller::msg::ThreatAlert alert;
    alert.header.stamp = now;
    alert.threat_id = "localization_" + alert_type + "_" + std::to_string(now.seconds());
    alert.threat_type = "localization_error";
    alert.severity_level = 2; // WARNING
    alert.description = description;
    alert.confidence = std::min(1.0, value / 10.0); // Normalize value somewhat arbitrarily
    alert.sensor_data_json = sensor_data;
    
    if (current_pose_) {
        alert.location = current_pose_->pose.pose.position;
    }

    threat_alert_pub_->publish(alert);
    last_alert_time = now;

    RCLCPP_WARN(get_logger(), "Localization Alert (%s): %s", alert_type.c_str(), description.c_str());
  }

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
  rclcpp::Time last_pose_update_;
  rclcpp::Time last_scan_update_;
  rclcpp::Time last_correction_time_;
  rclcpp::Time last_jump_threat_time_;
  rclcpp::Time last_scan_quality_threat_time_;
  
  // Data
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr current_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr previous_pose_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  double scan_match_score_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // ROS2 interfaces
  rclcpp::Publisher<sigyn_house_patroller::msg::ThreatAlert>::SharedPtr threat_alert_pub_;
  rclcpp::Publisher<sigyn_house_patroller::msg::SystemHealth>::SharedPtr health_pub_;
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
