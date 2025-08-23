#pragma once

#include "lidar_types.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <deque>
#include <memory>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sigyn_lidar_v2 {

  // Motion correction configuration (simplified: odom-only)
  struct MotionCorrectionConfig {
    bool enable_correction{ false };
    double max_correction_time_s{ 0.2 };  // Maximum time to look back for motion data
    std::string odom_topic{ "/sigyn/odom" };
  };

  // Motion corrector class for compensating LIDAR data during robot movement
  class MotionCorrector {
  public:
    explicit MotionCorrector(const MotionCorrectionConfig& config);
    ~MotionCorrector() = default;

    // Add motion data (odom only)
    void add_odom(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns);

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

    // Extract motion data from odometry
    MotionData odom_to_motion_data(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) const;

    // Configuration
    MotionCorrectionConfig config_;

    // Motion data history (sorted by timestamp)
    std::deque<MotionData> motion_history_;

    // Statistics
    mutable struct CorrectionStats {
      size_t scans_corrected{ 0 };
      size_t points_corrected{ 0 };
      size_t interpolations_performed{ 0 };
      size_t motion_data_points_used{ 0 };
    } stats_;

    // Constants for motion correction
    static constexpr double MAX_MOTION_DATA_AGE_S = 5.0;  // Maximum age of motion data to keep
  };

} // namespace sigyn_lidar_v2
