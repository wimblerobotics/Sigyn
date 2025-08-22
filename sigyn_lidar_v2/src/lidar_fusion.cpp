#include "sigyn_lidar_v2/lidar_fusion.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace sigyn_lidar_v2 {

LidarFusion::LidarFusion(const FusionConfig& config) : config_(config) {
  accumulated_scans_.reserve(config_.expected_scan_count);
}

void LidarFusion::add_scan(const LidarScan& scan) {
  accumulated_scans_.push_back(scan);
  
  // Keep only recent scans to prevent unlimited growth
  if (accumulated_scans_.size() > config_.expected_scan_count * 2) {
    accumulated_scans_.erase(accumulated_scans_.begin());
  }
}

bool LidarFusion::is_ready_to_fuse() const {
  return accumulated_scans_.size() >= config_.expected_scan_count;
}

sensor_msgs::msg::LaserScan LidarFusion::create_fused_scan(uint64_t timestamp_ns) const {
  auto fused_scan = merge_scans();
  
  // Set timestamp
  fused_scan.header.stamp.sec = timestamp_ns / 1000000000ULL;
  fused_scan.header.stamp.nanosec = timestamp_ns % 1000000000ULL;
  fused_scan.header.frame_id = config_.fused_frame_id;
  
  stats_.total_scans_fused++;
  
  return fused_scan;
}

void LidarFusion::clear() {
  accumulated_scans_.clear();
}

void LidarFusion::update_config(const FusionConfig& config) {
  config_ = config;
}

std::string LidarFusion::get_fusion_stats() const {
  std::stringstream ss;
  ss << "Fusion Stats: ";
  ss << "Total_Points=" << stats_.total_points_processed;
  ss << ", Scans_Fused=" << stats_.total_scans_fused;
  ss << ", Out_of_Range=" << stats_.points_out_of_range;
  ss << ", Overwrites=" << stats_.duplicate_angle_overwrites;
  ss << ", Accumulated=" << accumulated_scans_.size();
  return ss.str();
}

double LidarFusion::normalize_angle(double angle) {
  while (angle < 0.0) angle += 2.0 * M_PI;
  while (angle >= 2.0 * M_PI) angle -= 2.0 * M_PI;
  return angle;
}

std::vector<LidarFusion::FusionPoint> LidarFusion::scan_to_fusion_points(const LidarScan& scan) const {
  std::vector<FusionPoint> fusion_points;
  fusion_points.reserve(scan.points.size());
  
  for (const auto& point : scan.points) {
    // Convert angle to radians and normalize
    double angle_rad = normalize_angle(point.angle_deg * M_PI / 180.0);
    
    // Convert distance to meters and check range
    double range_m = point.distance_mm / 1000.0;
    if (range_m < config_.range_min || range_m > config_.range_max) {
      stats_.points_out_of_range++;
      continue;
    }
    
    // Check angle bounds
    if (angle_rad < config_.angle_min || angle_rad > config_.angle_max) {
      continue;
    }
    
    fusion_points.emplace_back(angle_rad, range_m, point.intensity, 
                               point.timestamp_ns, scan.frame_id);
    stats_.total_points_processed++;
  }
  
  return fusion_points;
}

sensor_msgs::msg::LaserScan LidarFusion::merge_scans() const {
  sensor_msgs::msg::LaserScan fused_scan;
  
  // Configure scan parameters
  fused_scan.angle_min = config_.angle_min;
  fused_scan.angle_max = config_.angle_max;
  fused_scan.angle_increment = config_.angle_increment;
  fused_scan.range_min = config_.range_min;
  fused_scan.range_max = config_.range_max;
  
  // Calculate number of range bins
  size_t range_count = static_cast<size_t>(
    std::ceil((config_.angle_max - config_.angle_min) / config_.angle_increment));
  
  // Initialize arrays
  fused_scan.ranges.assign(range_count, std::numeric_limits<float>::infinity());
  fused_scan.intensities.assign(range_count, 0.0f);
  
  // Track timing for scan_time calculation
  uint64_t earliest_timestamp = UINT64_MAX;
  uint64_t latest_timestamp = 0;
  
  // Process all accumulated scans
  for (const auto& scan : accumulated_scans_) {
    auto fusion_points = scan_to_fusion_points(scan);
    
    // Update timing
    if (!scan.points.empty()) {
      earliest_timestamp = std::min(earliest_timestamp, scan.scan_start_ns);
      latest_timestamp = std::max(latest_timestamp, scan.scan_end_ns);
    }
    
    // Merge points into fused scan
    for (const auto& point : fusion_points) {
      // Calculate bin index
      double relative_angle = point.angle_rad - config_.angle_min;
      int bin_index = static_cast<int>(std::round(relative_angle / config_.angle_increment));
      
      if (bin_index >= 0 && bin_index < static_cast<int>(range_count)) {
        // Keep the closest valid range measurement
        float current_range = static_cast<float>(point.range_m);
        if (std::isfinite(current_range)) {
          if (!std::isfinite(fused_scan.ranges[bin_index]) || 
              current_range < fused_scan.ranges[bin_index]) {
            if (std::isfinite(fused_scan.ranges[bin_index])) {
              stats_.duplicate_angle_overwrites++;
            }
            fused_scan.ranges[bin_index] = current_range;
            fused_scan.intensities[bin_index] = static_cast<float>(point.intensity);
          }
        }
      }
    }
  }
  
  // Calculate scan timing
  if (earliest_timestamp != UINT64_MAX && latest_timestamp > earliest_timestamp) {
    fused_scan.scan_time = (latest_timestamp - earliest_timestamp) / 1e9;
  } else {
    fused_scan.scan_time = 0.1; // Default 10 Hz
  }
  
  if (range_count > 0) {
    fused_scan.time_increment = fused_scan.scan_time / range_count;
  } else {
    fused_scan.time_increment = 0.0;
  }
  
  return fused_scan;
}

} // namespace sigyn_lidar_v2
