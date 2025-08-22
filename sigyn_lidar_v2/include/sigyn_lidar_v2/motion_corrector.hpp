#pragma once

#include "lidar_types.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <memory>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sigyn_lidar_v2 {

  // Motion correction configuration
  struct MotionCorrectionConfig {
    bool enable_correction;
    double max_correction_time_s;  // Maximum time to look back for motion data
    bool use_cmd_vel;             // Use command velocity
    bool use_wheel_odom;          // Use wheel odometry
    bool use_imu;                 // Use IMU data
    std::string cmd_vel_topic;
    std::string wheel_odom_topic;
    std::string imu_topic;

    MotionCorrectionConfig() : enable_correction(false), max_correction_time_s(0.2),
      use_cmd_vel(true), use_wheel_odom(true), use_imu(false),
      cmd_vel_topic("/cmd_vel"), wheel_odom_topic("/sigyn/wheel_odom"),
      imu_topic("/imu/data") {
    }
  };

  // Motion corrector class for compensating LIDAR data during robot movement
  class MotionCorrector {
  public:
    explicit MotionCorrector(const MotionCorrectionConfig& config);
    ~MotionCorrector() = default;

    // Add motion data from various sources
    void add_cmd_vel(const geometry_msgs::msg::Twist& twist, uint64_t timestamp_ns);
    void add_wheel_odom(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns);
    void add_imu_data(const sensor_msgs::msg::Imu& imu, uint64_t timestamp_ns);

    // Apply motion correction to a LIDAR scan
    LidarScan correct_scan(const LidarScan& input_scan) const;

    // Configuration access
    const MotionCorrectionConfig& get_config() const { return config_; }
    void update_config(const MotionCorrectionConfig& config);

    // Statistics and diagnostics
    std::string get_correction_stats() const;
    size_t get_motion_data_count() const { return motion_history_.size(); }

    // Clear old motion data
    void cleanup_old_data(uint64_t current_time_ns);

  private:
    // Get interpolated motion at a specific timestamp
    MotionData interpolate_motion_at_time(uint64_t timestamp_ns) const;

    // Apply motion correction to a single point
    LidarPoint correct_point(const LidarPoint& point, const MotionData& motion_start,
      const MotionData& motion_end) const;

    // Convert twist to motion data
    MotionData twist_to_motion_data(const geometry_msgs::msg::Twist& twist, uint64_t timestamp_ns) const;

    // Extract motion data from odometry
    MotionData odom_to_motion_data(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) const;

    // Configuration
    MotionCorrectionConfig config_;

    // Motion data history (sorted by timestamp)
    std::deque<MotionData> motion_history_;

    // Statistics
    mutable struct CorrectionStats {
      size_t scans_corrected;
      size_t points_corrected;
      size_t interpolations_performed;
      size_t motion_data_points_used;

      CorrectionStats() : scans_corrected(0), points_corrected(0),
        interpolations_performed(0), motion_data_points_used(0) {
      }
    } stats_;

    // Constants for motion correction
    static constexpr double MAX_MOTION_DATA_AGE_S = 5.0;  // Maximum age of motion data to keep
  };

} // namespace sigyn_lidar_v2
