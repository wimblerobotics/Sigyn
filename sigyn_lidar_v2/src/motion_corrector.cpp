#include "sigyn_lidar_v2/motion_corrector.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace sigyn_lidar_v2 {

MotionCorrector::MotionCorrector(const MotionCorrectionConfig& config) : config_(config) {
}

void MotionCorrector::add_cmd_vel(const geometry_msgs::msg::Twist& twist, uint64_t timestamp_ns) {
  if (!config_.enable_correction || !config_.use_cmd_vel) {
    return;
  }
  
  MotionData motion = twist_to_motion_data(twist, timestamp_ns);
  
  // Insert motion data in chronological order
  auto insert_pos = std::upper_bound(motion_history_.begin(), motion_history_.end(), motion,
    [](const MotionData& a, const MotionData& b) { return a.timestamp_ns < b.timestamp_ns; });
  
  motion_history_.insert(insert_pos, motion);
  
  // Clean up old data
  cleanup_old_data(timestamp_ns);
}

void MotionCorrector::add_wheel_odom(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) {
  if (!config_.enable_correction || !config_.use_wheel_odom) {
    return;
  }
  
  MotionData motion = odom_to_motion_data(odom, timestamp_ns);
  
  // Insert motion data in chronological order
  auto insert_pos = std::upper_bound(motion_history_.begin(), motion_history_.end(), motion,
    [](const MotionData& a, const MotionData& b) { return a.timestamp_ns < b.timestamp_ns; });
  
  motion_history_.insert(insert_pos, motion);
  
  // Clean up old data
  cleanup_old_data(timestamp_ns);
}

void MotionCorrector::add_imu_data(const sensor_msgs::msg::Imu& imu, uint64_t timestamp_ns) {
  if (!config_.enable_correction || !config_.use_imu) {
    return;
  }
  
  // For now, we'll just store angular velocity from IMU
  // In a full implementation, this would integrate with other motion sources
  MotionData motion;
  motion.timestamp_ns = timestamp_ns;
  motion.angular_z = imu.angular_velocity.z;
  
  auto insert_pos = std::upper_bound(motion_history_.begin(), motion_history_.end(), motion,
    [](const MotionData& a, const MotionData& b) { return a.timestamp_ns < b.timestamp_ns; });
  
  motion_history_.insert(insert_pos, motion);
  
  cleanup_old_data(timestamp_ns);
}

LidarScan MotionCorrector::correct_scan(const LidarScan& input_scan) const {
  if (!config_.enable_correction || motion_history_.empty()) {
    return input_scan; // Return unmodified scan
  }
  
  LidarScan corrected_scan = input_scan;
  
  // Get motion data at scan start and end
  MotionData motion_start = interpolate_motion_at_time(input_scan.scan_start_ns);
  MotionData motion_end = interpolate_motion_at_time(input_scan.scan_end_ns);
  
  // Apply correction to each point
  for (auto& point : corrected_scan.points) {
    point = correct_point(point, motion_start, motion_end);
  }
  
  stats_.scans_corrected++;
  stats_.points_corrected += corrected_scan.points.size();
  
  return corrected_scan;
}

void MotionCorrector::update_config(const MotionCorrectionConfig& config) {
  config_ = config;
}

std::string MotionCorrector::get_correction_stats() const {
  std::stringstream ss;
  ss << "Motion Correction Stats: ";
  ss << "Scans_Corrected=" << stats_.scans_corrected;
  ss << ", Points_Corrected=" << stats_.points_corrected;
  ss << ", Interpolations=" << stats_.interpolations_performed;
  ss << ", Motion_Points=" << motion_history_.size();
  return ss.str();
}

void MotionCorrector::cleanup_old_data(uint64_t current_time_ns) {
  uint64_t cutoff_time = current_time_ns - static_cast<uint64_t>(MAX_MOTION_DATA_AGE_S * 1e9);
  
  auto cutoff_it = std::lower_bound(motion_history_.begin(), motion_history_.end(), cutoff_time,
    [](const MotionData& motion, uint64_t timestamp) { return motion.timestamp_ns < timestamp; });
  
  motion_history_.erase(motion_history_.begin(), cutoff_it);
}

MotionData MotionCorrector::interpolate_motion_at_time(uint64_t timestamp_ns) const {
  if (motion_history_.empty()) {
    return MotionData();
  }
  
  // Find the motion data points surrounding the timestamp
  auto upper_it = std::upper_bound(motion_history_.begin(), motion_history_.end(), timestamp_ns,
    [](uint64_t timestamp, const MotionData& motion) { return timestamp < motion.timestamp_ns; });
  
  if (upper_it == motion_history_.begin()) {
    // Timestamp is before all motion data, use first point
    return motion_history_.front();
  }
  
  if (upper_it == motion_history_.end()) {
    // Timestamp is after all motion data, use last point
    return motion_history_.back();
  }
  
  // Interpolate between two points
  auto lower_it = upper_it - 1;
  const MotionData& before = *lower_it;
  const MotionData& after = *upper_it;
  
  double time_range = after.timestamp_ns - before.timestamp_ns;
  if (time_range == 0) {
    return before;
  }
  
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
  // For now, implement a basic motion correction
  // In a full implementation, this would properly transform points based on robot motion
  
  LidarPoint corrected = point;
  
  // Simple correction: adjust angle based on angular velocity
  double scan_duration = (motion_end.timestamp_ns - motion_start.timestamp_ns) / 1e9;
  if (scan_duration > 0) {
    double point_time_ratio = (point.timestamp_ns - motion_start.timestamp_ns) / 1e9 / scan_duration;
    double angular_correction = motion_start.angular_z * point_time_ratio * scan_duration;
    
    // Apply angular correction
    corrected.angle_deg += angular_correction * 180.0 / M_PI;
    
    // Normalize angle
    while (corrected.angle_deg >= 360.0f) corrected.angle_deg -= 360.0f;
    while (corrected.angle_deg < 0.0f) corrected.angle_deg += 360.0f;
  }
  
  return corrected;
}

MotionData MotionCorrector::twist_to_motion_data(const geometry_msgs::msg::Twist& twist, uint64_t timestamp_ns) const {
  MotionData motion;
  motion.timestamp_ns = timestamp_ns;
  motion.linear_x = twist.linear.x;
  motion.linear_y = twist.linear.y;
  motion.angular_z = twist.angular.z;
  return motion;
}

MotionData MotionCorrector::odom_to_motion_data(const nav_msgs::msg::Odometry& odom, uint64_t timestamp_ns) const {
  MotionData motion;
  motion.timestamp_ns = timestamp_ns;
  motion.linear_x = odom.twist.twist.linear.x;
  motion.linear_y = odom.twist.twist.linear.y;
  motion.angular_z = odom.twist.twist.angular.z;
  motion.pos_x = odom.pose.pose.position.x;
  motion.pos_y = odom.pose.pose.position.y;
  
  // Extract yaw from quaternion
  auto& q = odom.pose.pose.orientation;
  motion.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  return motion;
}

} // namespace sigyn_lidar_v2
