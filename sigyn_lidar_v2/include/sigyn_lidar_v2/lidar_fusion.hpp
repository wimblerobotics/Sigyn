#pragma once

#include "lidar_types.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <memory>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sigyn_lidar_v2 {

  // Configuration for scan fusion
  struct FusionConfig {
    std::string fused_frame_id;
    std::string fused_topic_name;
    double angle_min;           // Minimum angle for fused scan (radians)
    double angle_max;           // Maximum angle for fused scan (radians)
    double angle_increment;     // Angular resolution (radians)
    double range_min;           // Minimum valid range (meters)
    double range_max;           // Maximum valid range (meters)
    size_t expected_scan_count; // Expected number of input scans

    FusionConfig() : fused_frame_id("base_link"), fused_topic_name("/scan"),
      angle_min(0.0), angle_max(2.0 * M_PI),
      angle_increment(M_PI / 180.0 * 0.8), // ~0.8 degrees
      range_min(0.02), range_max(15.0),
      expected_scan_count(1) {
    }
  };

  // LIDAR scan fusion class
  class LidarFusion {
  public:
    explicit LidarFusion(const FusionConfig& config);
    ~LidarFusion() = default;

    // Add a scan from an individual LIDAR
    void add_scan(const LidarScan& scan);

    // Check if we have enough scans to generate a fused result
    bool is_ready_to_fuse() const;

    // Generate fused LaserScan message
    sensor_msgs::msg::LaserScan create_fused_scan(uint64_t timestamp_ns) const;

    // Clear accumulated scans
    void clear();

    // Configuration access
    const FusionConfig& get_config() const { return config_; }
    void update_config(const FusionConfig& config);

    // Statistics
    size_t get_accumulated_scan_count() const { return accumulated_scans_.size(); }
    std::string get_fusion_stats() const;

  private:
    // Convert LidarScan to intermediate representation for fusion
    struct FusionPoint {
      double angle_rad;
      double range_m;
      double intensity;
      uint64_t timestamp_ns;
      std::string source_frame;

      FusionPoint(double angle, double range, double intens, uint64_t stamp, const std::string& frame)
        : angle_rad(angle), range_m(range), intensity(intens), timestamp_ns(stamp), source_frame(frame) {
      }
    };

    // Normalize angle to [0, 2π)
    static double normalize_angle(double angle);

    // Convert individual scan to fusion points
    std::vector<FusionPoint> scan_to_fusion_points(const LidarScan& scan) const;

    // Merge multiple point clouds into a single LaserScan
    sensor_msgs::msg::LaserScan merge_scans() const;

    // Configuration
    FusionConfig config_;

    // Accumulated scan data
    std::vector<LidarScan> accumulated_scans_;

    // Statistics
    mutable struct FusionStats {
      size_t total_points_processed;
      size_t total_scans_fused;
      size_t points_out_of_range;
      size_t duplicate_angle_overwrites;

      FusionStats() : total_points_processed(0), total_scans_fused(0),
        points_out_of_range(0), duplicate_angle_overwrites(0) {
      }
    } stats_;
  };

} // namespace sigyn_lidar_v2
