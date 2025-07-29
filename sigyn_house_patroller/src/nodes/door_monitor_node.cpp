#include <memory>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/system_health.hpp"
#include "sigyn_house_patroller/msg/room_identification.hpp"

namespace sigyn_house_patroller {

class DoorMonitorNode : public rclcpp::Node {
public:
  explicit DoorMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("door_monitor", options),
        last_scan_update_(std::chrono::steady_clock::now()) {
    
    // Declare parameters
    declare_parameter("monitoring_frequency", 2.0);
    declare_parameter("scan_timeout", 30.0);
    declare_parameter("scan_topic", "/scan");
    declare_parameter("distance_tolerance", 0.1);
    declare_parameter("alert_cooldown", 120.0);  // 2 minutes
    declare_parameter("angle_tolerance", 0.1);   // radians
    declare_parameter("map_frame", "map");
    declare_parameter("robot_frame", "base_link");
    declare_parameter("enable_learning", true);
    declare_parameter("learning_samples", 20);
    
    // Door configuration parameters
    declare_parameter("doors.front_door.name", "Front Door");
    declare_parameter("doors.front_door.angle", 0.0);
    declare_parameter("doors.front_door.expected_distance", 0.5);
    declare_parameter("doors.front_door.expected_open", false);
    declare_parameter("doors.front_door.tolerance", 0.1);
    
    declare_parameter("doors.back_door.name", "Back Door");
    declare_parameter("doors.back_door.angle", 3.14159);
    declare_parameter("doors.back_door.expected_distance", 0.5);
    declare_parameter("doors.back_door.expected_open", false);
    declare_parameter("doors.back_door.tolerance", 0.1);
    
    declare_parameter("doors.bedroom_1_door.name", "Master Bedroom Door");
    declare_parameter("doors.bedroom_1_door.angle", 1.5708);
    declare_parameter("doors.bedroom_1_door.expected_distance", 0.3);
    declare_parameter("doors.bedroom_1_door.expected_open", true);
    declare_parameter("doors.bedroom_1_door.tolerance", 0.1);
    
    declare_parameter("doors.bathroom_door.name", "Bathroom Door");
    declare_parameter("doors.bathroom_door.angle", -1.5708);
    declare_parameter("doors.bathroom_door.expected_distance", 0.3);
    declare_parameter("doors.bathroom_door.expected_open", true);
    declare_parameter("doors.bathroom_door.tolerance", 0.1);
    
    // Get parameters
    monitoring_frequency_ = get_parameter("monitoring_frequency").as_double();
    scan_timeout_ = get_parameter("scan_timeout").as_double();
    scan_topic_ = get_parameter("scan_topic").as_string();
    distance_tolerance_ = get_parameter("distance_tolerance").as_double();
    alert_cooldown_ = get_parameter("alert_cooldown").as_double();
    angle_tolerance_ = get_parameter("angle_tolerance").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    enable_learning_ = get_parameter("enable_learning").as_bool();
    learning_samples_ = get_parameter("learning_samples").as_int();
    
    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Load door configurations
    LoadDoorConfigurations();
    
    // Initialize state
    scan_healthy_ = true;
    
    // Publishers
    threat_publisher_ = create_publisher<msg::ThreatAlert>(
      "~/threat_alerts", rclcpp::QoS(10).reliable());
    
    system_health_publisher_ = create_publisher<msg::SystemHealth>(
      "~/health", rclcpp::QoS(10).reliable());
    
    // Subscribers
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&DoorMonitorNode::LaserScanCallback, this, std::placeholders::_1));
    
    room_id_sub_ = create_subscription<msg::RoomIdentification>(
      "/sigyn_house_patroller/room_identification", rclcpp::QoS(10).reliable(),
      std::bind(&DoorMonitorNode::RoomIdentificationCallback, this, std::placeholders::_1));
    
    // Timers
    monitoring_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / monitoring_frequency_)),
      std::bind(&DoorMonitorNode::MonitoringTimerCallback, this));
    
    health_timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&DoorMonitorNode::HealthTimerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Door monitor started - Monitoring %zu doors", 
                door_configs_.size());
  }

private:
  struct DoorConfig {
    std::string name;
    double angle;  // Angle from robot to door in radians
    double expected_distance;
    bool expected_open;
    double tolerance;
    
    // Learning data
    std::vector<double> distance_history;
    double learned_closed_distance;
    double learned_open_distance;
    bool learning_complete;
    
    // State tracking
    std::chrono::steady_clock::time_point last_state_change_time;
    double min_notification_interval;  // seconds
    
    DoorConfig() : learning_complete(false), min_notification_interval(10.0) {}
  };
  
  void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    current_scan_ = msg;
    last_scan_update_ = std::chrono::steady_clock::now();
    
    // Check all doors
    CheckAllDoors();
  }
  
  void RoomIdentificationCallback(const msg::RoomIdentification::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    if (msg->confidence > 0.7) {
      current_room_ = msg->room_name;
    }
  }
  
  void MonitoringTimerCallback() {
    if (!scan_healthy_) {
      if (last_threat_time_ == rclcpp::Time(0, 0, RCL_ROS_TIME) ||
          (this->now() - last_threat_time_).seconds() > 60) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "door_monitor_stale_" + std::to_string(this->now().seconds());
        alert.threat_type = "door_monitor_stale";
        alert.severity_level = 2; // WARNING
        alert.description = "Door monitor is not receiving laser scan updates.";
        alert.room_name = current_room_;
        alert.sensor_data_json = "{}";
        threat_publisher_->publish(alert);
        last_threat_time_ = this->now();
      }
    }
  }
  
  void CheckAllDoors() {
    if (!current_scan_) {
      return;
    }
    
    for (auto& [door_id, config] : door_configs_) {
      CheckDoorState(door_id, config);
    }
  }
  
  void CheckDoorState(const std::string& door_id, DoorConfig& config) {
    double measured_distance = GetLaserReadingAtAngle(config.angle);
    
    if (std::isnan(measured_distance) || std::isinf(measured_distance)) {
      return;  // Invalid reading
    }
    
    // Update learning data if enabled
    if (enable_learning_ && !config.learning_complete) {
      config.distance_history.push_back(measured_distance);
      
      if (config.distance_history.size() >= learning_samples_) {
        CompleteLearning(config);
        config.learning_complete = true;
        
        RCLCPP_INFO(get_logger(), "Completed learning for %s - Closed: %.2fm, Open: %.2fm", 
                    config.name.c_str(), config.learned_closed_distance, config.learned_open_distance);
      }
      return;  // Don't check threats while learning
    }
    
    // Determine expected distance based on expected state
    double expected_distance = config.expected_distance;
    if (config.learning_complete) {
      expected_distance = config.expected_open ? config.learned_open_distance : config.learned_closed_distance;
    }
    
    // Check if door state matches expectation
    double distance_diff = std::abs(measured_distance - expected_distance);
    
    if (distance_diff > config.tolerance) {
      // Check alert cooldown
      auto now = std::chrono::steady_clock::now();
      auto last_alert_it = last_alerts_.find(door_id);
      
      if (last_alert_it != last_alerts_.end()) {
        auto time_since_alert = std::chrono::duration_cast<std::chrono::seconds>(
          now - last_alert_it->second).count();
        
        if (time_since_alert < alert_cooldown_) {
          return;  // Still in cooldown period
        }
      }
      
      // Determine what state the door is actually in
      bool door_is_open = DetermineDoorState(measured_distance, config);
      std::string expected_state = config.expected_open ? "open" : "closed";
      std::string actual_state = door_is_open ? "open" : "closed";
      
      // Only alert if state doesn't match expectation
      if (door_is_open != config.expected_open) {
        msg::ThreatAlert alert;
        alert.header.stamp = this->now();
        alert.threat_id = "door_state_" + door_id;
        alert.threat_type = "door_state_change";
        alert.severity_level = 2; // WARNING
        alert.description = config.name + " is " + actual_state + 
                           " (expected: " + expected_state + ")" +
                           " - Distance: " + std::to_string(measured_distance) + "m" +
                           " (expected: " + std::to_string(expected_distance) + "m)";
        alert.confidence = std::min(1.0, distance_diff / config.tolerance);
        alert.room_name = current_room_;
        alert.sensor_data_json = "{\"door_id\": \"" + door_id + 
                           "\", \"measured_distance\": " + std::to_string(measured_distance) + 
                           ", \"expected_distance\": " + std::to_string(expected_distance) + 
                           ", \"difference\": " + std::to_string(distance_diff) + 
                           ", \"actual_state\": \"" + actual_state + 
                           "\", \"expected_state\": \"" + expected_state + "\"}";
        
        threat_publisher_->publish(alert);
        
        // Update last alert time
        last_alerts_[door_id] = now;
        
        RCLCPP_WARN(get_logger(), "Door state change: %s is %s (expected: %s)", 
                    config.name.c_str(), actual_state.c_str(), expected_state.c_str());
      }
    }
  }
  
  bool DetermineDoorState(double measured_distance, const DoorConfig& config) {
    if (!config.learning_complete) {
      // Use simple threshold if not learned
      return measured_distance > config.expected_distance + config.tolerance;
    }
    
    // Use learned distances
    double closed_diff = std::abs(measured_distance - config.learned_closed_distance);
    double open_diff = std::abs(measured_distance - config.learned_open_distance);
    
    return open_diff < closed_diff;  // Closer to open distance
  }
  
  void CompleteLearning(DoorConfig& config) {
    if (config.distance_history.empty()) {
      return;
    }
    
    // Sort distances to find clusters
    std::vector<double> sorted_distances = config.distance_history;
    std::sort(sorted_distances.begin(), sorted_distances.end());
    
    // Find the two most common distance ranges (closed and open)
    // Simple approach: use quartiles
    size_t q1_idx = sorted_distances.size() / 4;
    size_t q3_idx = (sorted_distances.size() * 3) / 4;
    
    config.learned_closed_distance = sorted_distances[q1_idx];
    config.learned_open_distance = sorted_distances[q3_idx];
    
    // If they're too close, use min/max
    if (config.learned_open_distance - config.learned_closed_distance < 0.2) {
      config.learned_closed_distance = sorted_distances[0];
      config.learned_open_distance = sorted_distances.back();
    }
  }
  
  double GetLaserReadingAtAngle(double angle) {
    if (!current_scan_) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Normalize angle to scan range
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    
    // Find the corresponding index in the scan
    if (angle < current_scan_->angle_min || angle > current_scan_->angle_max) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    
    int index = static_cast<int>((angle - current_scan_->angle_min) / current_scan_->angle_increment);
    
    if (index < 0 || index >= static_cast<int>(current_scan_->ranges.size())) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    
    float range = current_scan_->ranges[index];
    
    // Check if reading is valid
    if (range < current_scan_->range_min || range > current_scan_->range_max) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    
    return static_cast<double>(range);
  }
  
  void LoadDoorConfigurations() {
    std::vector<std::string> door_ids = {"front_door", "back_door", "bedroom_1_door", "bathroom_door"};
    
    for (const auto& door_id : door_ids) {
      std::string base_param = "doors." + door_id + ".";
      
      if (has_parameter(base_param + "name")) {
        DoorConfig config;
        config.name = get_parameter(base_param + "name").as_string();
        config.angle = get_parameter(base_param + "angle").as_double();
        config.expected_distance = get_parameter(base_param + "expected_distance").as_double();
        config.expected_open = get_parameter(base_param + "expected_open").as_bool();
        config.tolerance = get_parameter(base_param + "tolerance").as_double();
        
        door_configs_[door_id] = config;
        
        RCLCPP_INFO(get_logger(), "Loaded door config: %s at angle %.2f rad, expected %s", 
                    config.name.c_str(), config.angle, 
                    config.expected_open ? "open" : "closed");
      }
    }
  }
  
  void HealthTimerCallback() {
    msg::SystemHealth health;
    health.header.stamp = this->now();
    health.overall_health = scan_healthy_ ? 1 : 2; // HEALTHY : DEGRADED
    health.system_status_description = scan_healthy_ ? "OK" : "Door monitor not receiving laser scan updates";

    health.component_names.push_back("door_monitor");
    health.component_health_status.push_back(scan_healthy_ ? 1 : 2); // HEALTHY : DEGRADED
    health.component_descriptions.push_back(std::string("door_monitor: ") + (scan_healthy_ ? "OK" : "ERROR"));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("doors_configured");
    health.component_health_status.push_back(1); // HEALTHY
    health.component_descriptions.push_back("doors_configured: " + std::to_string(door_configs_.size()));
    health.last_update_times.push_back(this->now());

    health.component_names.push_back("current_room");
    health.component_health_status.push_back(1); // HEALTHY
    health.component_descriptions.push_back("current_room: " + current_room_);
    health.last_update_times.push_back(this->now());

    for (auto const& [door_id, config] : door_configs_) {
      bool learning_complete = config.distance_history.size() >= learning_samples_;
      health.component_names.push_back("door_" + door_id);
      health.component_health_status.push_back(learning_complete ? 1 : 2); // HEALTHY : DEGRADED
      health.component_descriptions.push_back("learning_complete: " + std::string(learning_complete ? "YES" : "NO"));
      health.last_update_times.push_back(this->now());
    }

    system_health_publisher_->publish(health);
  }
  
  // Parameters
  double monitoring_frequency_;
  double scan_timeout_;
  std::string scan_topic_;
  double distance_tolerance_;
  double alert_cooldown_;
  double angle_tolerance_;
  std::string map_frame_;
  std::string robot_frame_;
  bool enable_learning_;
  int learning_samples_;
  
  // State
  bool scan_healthy_;
  std::string current_room_;
  std::chrono::steady_clock::time_point last_scan_update_;
  rclcpp::Time last_threat_time_{0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> last_alerts_;
  
  // Data
  std::unordered_map<std::string, DoorConfig> door_configs_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // ROS2 interfaces
  rclcpp::Publisher<msg::ThreatAlert>::SharedPtr threat_publisher_;
  rclcpp::Publisher<msg::SystemHealth>::SharedPtr system_health_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<msg::RoomIdentification>::SharedPtr room_id_sub_;
  rclcpp::TimerBase::SharedPtr monitoring_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  
  std::mutex scan_mutex_;
};

}  // namespace sigyn_house_patroller

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<sigyn_house_patroller::DoorMonitorNode>();
  
  RCLCPP_INFO(rclcpp::get_logger("door_monitor"), "Starting door monitor node");
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
