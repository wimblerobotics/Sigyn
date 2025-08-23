#include "sigyn_lidar_v2/motion_corrector.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace sigyn_lidar_v2 {

  MotionCorrector::MotionCorrector(const MotionCorrectionConfig& config) : config_(config) {}

  void MotionCorrector::add_odom(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) {
    if (!config_.enable_correction) return;

    MotionData motion = odom_to_motion_data(odom, timestamp_ns);

    auto insert_pos = std::upper_bound(motion_history_.begin(), motion_history_.end(), motion,
      [](const MotionData& a, const MotionData& b) { return a.timestamp_ns < b.timestamp_ns; });

    motion_history_.insert(insert_pos, motion);
    cleanup_old_data(timestamp_ns);
  }

  LidarScan MotionCorrector::correct_scan(const LidarScan& input_scan) const {
    if (!config_.enable_correction || motion_history_.empty()) {
      return input_scan;
    }

    LidarScan corrected_scan = input_scan;

    MotionData motion_start = interpolate_motion_at_time(input_scan.scan_start_ns);
    MotionData motion_end = interpolate_motion_at_time(input_scan.scan_end_ns);

    for (auto& point : corrected_scan.points) {
      point = correct_point(point, motion_start, motion_end);
    }

    stats_.scans_corrected++;
    stats_.points_corrected += corrected_scan.points.size();

    return corrected_scan;
  }

  void MotionCorrector::update_config(const MotionCorrectionConfig& config) { config_ = config; }

  std::string MotionCorrector::get_correction_stats() const {
    std::stringstream ss;
    ss << "Motion Correction Stats: Scans_Corrected=" << stats_.scans_corrected
      << ", Points_Corrected=" << stats_.points_corrected
      << ", Interpolations=" << stats_.interpolations_performed
      << ", Motion_Points=" << motion_history_.size();
    return ss.str();
  }

  void MotionCorrector::cleanup_old_data(uint64_t current_time_ns) {
    uint64_t cutoff_time = current_time_ns - static_cast<uint64_t>(MAX_MOTION_DATA_AGE_S * 1e9);
    auto cutoff_it = std::lower_bound(motion_history_.begin(), motion_history_.end(), cutoff_time,
      [](const MotionData& motion, uint64_t timestamp) { return motion.timestamp_ns < timestamp; });
    motion_history_.erase(motion_history_.begin(), cutoff_it);
  }

  MotionData MotionCorrector::interpolate_motion_at_time(uint64_t timestamp_ns) const {
    if (motion_history_.empty()) return MotionData();

    auto upper_it = std::upper_bound(motion_history_.begin(), motion_history_.end(), timestamp_ns,
      [](uint64_t timestamp, const MotionData& motion) { return timestamp < motion.timestamp_ns; });

    if (upper_it == motion_history_.begin()) return motion_history_.front();
    if (upper_it == motion_history_.end()) return motion_history_.back();

    auto lower_it = upper_it - 1;
    const MotionData& before = *lower_it;
    const MotionData& after = *upper_it;

    double time_range = after.timestamp_ns - before.timestamp_ns;
    if (time_range == 0) return before;

    double ratio = static_cast<double>(timestamp_ns - before.timestamp_ns) / time_range;

    MotionData interpolated;
    interpolated.timestamp_ns = timestamp_ns;
    interpolated.linear_x = before.linear_x + ratio * (after.linear_x - before.linear_x);
    interpolated.linear_y = before.linear_y + ratio * (after.linear_y - before.linear_y);
    interpolated.angular_z = before.angular_z + ratio * (after.angular_z - before.angular_z);
    interpolated.pos_x = before.pos_x + ratio * (after.pos_x - before.pos_x);
    interpolated.pos_y = before.pos_y + ratio * (after.pos_y - before.pos_y);
    interpolated.yaw = before.yaw + ratio * (after.yaw - before.yaw);

    stats_.interpolations_performed++;
    return interpolated;
  }

  LidarPoint MotionCorrector::correct_point(const LidarPoint& point, const MotionData& motion_start,
    const MotionData& motion_end) const {
    LidarPoint corrected = point;

    double scan_duration_s = (motion_end.timestamp_ns - motion_start.timestamp_ns) / 1e9;
    if (scan_duration_s <= 0) return corrected;

    double point_time_ratio = std::clamp(
      (point.timestamp_ns - motion_start.timestamp_ns) / static_cast<double>(motion_end.timestamp_ns - motion_start.timestamp_ns),
      0.0, 1.0);

    MotionData point_motion;
    point_motion.linear_x = motion_start.linear_x + point_time_ratio * (motion_end.linear_x - motion_start.linear_x);
    point_motion.linear_y = motion_start.linear_y + point_time_ratio * (motion_end.linear_y - motion_start.linear_y);
    point_motion.angular_z = motion_start.angular_z + point_time_ratio * (motion_end.angular_z - motion_start.angular_z);

    double dt = point_time_ratio * scan_duration_s;
    double linear_displacement_x = point_motion.linear_x * dt;
    double linear_displacement_y = point_motion.linear_y * dt;
    double angular_displacement = point_motion.angular_z * dt;

    double angle_rad = point.angle_deg * M_PI / 180.0;
    double range_m = point.distance_mm / 1000.0;
    double point_x = range_m * std::cos(angle_rad);
    double point_y = range_m * std::sin(angle_rad);

    double corrected_x = point_x - linear_displacement_x;
    double corrected_y = point_y - linear_displacement_y;

    double cos_theta = std::cos(-angular_displacement);
    double sin_theta = std::sin(-angular_displacement);
    double final_x = corrected_x * cos_theta - corrected_y * sin_theta;
    double final_y = corrected_x * sin_theta + corrected_y * cos_theta;

    double final_range_m = std::sqrt(final_x * final_x + final_y * final_y);
    corrected.distance_mm = static_cast<uint16_t>(final_range_m * 1000.0);
    corrected.angle_deg = std::atan2(final_y, final_x) * 180.0 / M_PI;

    while (corrected.angle_deg >= 360.0f) corrected.angle_deg -= 360.0f;
    while (corrected.angle_deg < 0.0f) corrected.angle_deg += 360.0f;

    corrected.intensity = point.intensity;
    corrected.timestamp_ns = point.timestamp_ns;
    return corrected;
  }

  MotionData MotionCorrector::odom_to_motion_data(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) const {
    MotionData motion;
    motion.timestamp_ns = timestamp_ns;
    motion.linear_x = odom.twist.twist.linear.x;
    motion.linear_y = odom.twist.twist.linear.y;
    motion.angular_z = odom.twist.twist.angular.z;
    motion.pos_x = odom.pose.pose.position.x;
    motion.pos_y = odom.pose.pose.position.y;

    auto& q = odom.pose.pose.orientation;
    motion.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return motion;
  }

} // namespace sigyn_lidar_v2
