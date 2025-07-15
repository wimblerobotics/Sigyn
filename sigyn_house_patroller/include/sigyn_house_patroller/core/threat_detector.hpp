#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sigyn_house_patroller/msg/threat_alert.hpp"
#include "sigyn_house_patroller/msg/room_identification.hpp"

namespace sigyn_house_patroller {

/**
 * @brief Base class for all threat detection modules
 */
class ThreatDetector {
public:
  /**
   * @brief Threat severity levels
   */
  enum class ThreatSeverity {
    kInfo = 1,
    kWarning = 2,
    kCritical = 3,
    kEmergency = 4
  };

  explicit ThreatDetector(const std::string& detector_name);
  virtual ~ThreatDetector() = default;

  /**
   * @brief Initialize the threat detector
   * @param node ROS2 node for subscriptions and publishers
   * @return True if initialization successful
   */
  virtual bool Initialize(rclcpp::Node::SharedPtr node) = 0;

  /**
   * @brief Check for threats
   * @return List of detected threats
   */
  virtual std::vector<msg::ThreatAlert> CheckForThreats() = 0;

  /**
   * @brief Get detector name
   * @return Name of this detector
   */
  const std::string& GetName() const { return detector_name_; }

  /**
   * @brief Check if detector is healthy
   * @return True if detector is functioning properly
   */
  virtual bool IsHealthy() const = 0;

  /**
   * @brief Get last update time
   * @return Time of last threat check
   */
  std::chrono::system_clock::time_point GetLastUpdateTime() const { return last_update_time_; }

protected:
  /**
   * @brief Create a threat alert message
   * @param threat_type Type of threat detected
   * @param severity Severity level
   * @param location Location of threat
   * @param description Human-readable description
   * @param confidence Confidence level (0.0 to 1.0)
   * @param sensor_data JSON string of sensor data
   * @return Populated threat alert message
   */
  msg::ThreatAlert CreateThreatAlert(
    const std::string& threat_type,
    ThreatSeverity severity,
    const geometry_msgs::msg::Point& location,
    const std::string& description,
    double confidence,
    const std::string& sensor_data) const;

  /**
   * @brief Update the last update time
   */
  void UpdateLastUpdateTime() { last_update_time_ = std::chrono::system_clock::now(); }

private:
  std::string detector_name_;
  std::chrono::system_clock::time_point last_update_time_;
};

/**
 * @brief Battery level threat detector
 */
class BatteryThreatDetector : public ThreatDetector {
public:
  explicit BatteryThreatDetector();
  
  bool Initialize(rclcpp::Node::SharedPtr node) override;
  std::vector<msg::ThreatAlert> CheckForThreats() override;
  bool IsHealthy() const override;

private:
  void BatteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  sensor_msgs::msg::BatteryState::SharedPtr current_battery_state_;
  double critical_level_;
  double low_level_;
  mutable std::mutex battery_mutex_;
};

/**
 * @brief Temperature anomaly threat detector
 */
class TemperatureThreatDetector : public ThreatDetector {
public:
  explicit TemperatureThreatDetector();
  
  bool Initialize(rclcpp::Node::SharedPtr node) override;
  std::vector<msg::ThreatAlert> CheckForThreats() override;
  bool IsHealthy() const override;

private:
  void TemperatureCallback(const sensor_msgs::msg::Temperature::SharedPtr msg);
  bool IsTemperatureAnomalous(double temperature, const std::string& room) const;
  
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  std::vector<sensor_msgs::msg::Temperature> temperature_history_;
  std::unordered_map<std::string, double> room_baseline_temps_;
  double anomaly_threshold_;
  size_t history_size_;
  mutable std::mutex temp_mutex_;
};

/**
 * @brief Door state threat detector
 */
class DoorStateThreatDetector : public ThreatDetector {
public:
  explicit DoorStateThreatDetector();
  
  bool Initialize(rclcpp::Node::SharedPtr node) override;
  std::vector<msg::ThreatAlert> CheckForThreats() override;
  bool IsHealthy() const override;

private:
  struct DoorInfo {
    std::string door_name;
    geometry_msgs::msg::Point location;
    double expected_distance;
    double tolerance;
    bool expected_open;
    std::chrono::system_clock::time_point last_checked;
  };

  void LaserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  bool CheckDoorState(const DoorInfo& door) const;
  double GetLaserReadingAtAngle(double angle) const;
  void LoadDoorConfiguration();
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr current_scan_;
  std::vector<DoorInfo> known_doors_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  mutable std::mutex laser_mutex_;
};

/**
 * @brief Motion/change detection threat detector
 */
class ChangeDetectionThreatDetector : public ThreatDetector {
public:
  explicit ChangeDetectionThreatDetector();
  
  bool Initialize(rclcpp::Node::SharedPtr node) override;
  std::vector<msg::ThreatAlert> CheckForThreats() override;
  bool IsHealthy() const override;

private:
  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  bool DetectSignificantChanges() const;
  void UpdateReferenceModel();
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  
  // Reference models for each room
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2> room_reference_clouds_;
  std::unordered_map<std::string, sensor_msgs::msg::Image> room_reference_images_;
  
  sensor_msgs::msg::PointCloud2::SharedPtr current_pointcloud_;
  sensor_msgs::msg::Image::SharedPtr current_image_;
  
  double change_threshold_;
  mutable std::mutex data_mutex_;
};

/**
 * @brief Comprehensive threat detection manager
 */
class ThreatDetectionManager {
public:
  explicit ThreatDetectionManager(rclcpp::Node::SharedPtr node);
  ~ThreatDetectionManager() = default;

  /**
   * @brief Initialize all threat detectors
   * @return True if all detectors initialized successfully
   */
  bool Initialize();

  /**
   * @brief Run threat detection cycle
   * @return List of all detected threats
   */
  std::vector<msg::ThreatAlert> RunDetection();

  /**
   * @brief Check if all detectors are healthy
   * @return True if all detectors are functioning
   */
  bool AreDetectorsHealthy() const;

  /**
   * @brief Get detector health summary
   * @return Map of detector names to health status
   */
  std::unordered_map<std::string, bool> GetDetectorHealthStatus() const;

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<std::unique_ptr<ThreatDetector>> detectors_;
  mutable std::mutex detectors_mutex_;
};

}  // namespace sigyn_house_patroller
