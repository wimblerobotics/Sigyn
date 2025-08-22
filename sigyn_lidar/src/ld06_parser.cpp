#include "sigyn_lidar/ld06_parser.hpp"
#include "sigyn_lidar/ld06_protocol.hpp"  // Use existing implementation
#include <algorithm>
#include <cmath>

namespace sigyn_lidar {

// Implementation that wraps existing LD06Parser to avoid licensing issues
class ModularLD06Parser::Impl {
public:
  // Use existing proven LD06Parser implementation
  sigyn_lidar::LD06Parser legacy_parser_;
  std::deque<LidarScanFrame> completed_frames_;
  
  // Convert from legacy format to new interface format
  LidarScanFrame convertFrame(const ScanFrame& legacy_frame) {
    LidarScanFrame frame;
    frame.scan_start_time = legacy_frame.scan_start_time;
    frame.scan_end_time = legacy_frame.scan_end_time;
    frame.revolution_hz = legacy_frame.revolution_hz;
    frame.is_motion_compensated = legacy_frame.is_motion_compensated;
    frame.source_parser = "ld06";
    
    // Convert beams
    for (const auto& legacy_beam : legacy_frame.beams) {
      LidarBeam beam;
      beam.angle_rad = legacy_beam.angle_rad;
      beam.range_m = legacy_beam.range_m;
      beam.confidence = legacy_beam.confidence;
      beam.relative_time = 0.0; // Legacy doesn't have this
      beam.deskewed_angle_rad = legacy_beam.angle_rad;
      beam.is_deskewed = false;
      frame.beams.push_back(beam);
    }
    
    return frame;
  }
};

ModularLD06Parser::ModularLD06Parser() : impl_(std::make_unique<Impl>()) {
  RCLCPP_DEBUG(rclcpp::get_logger("sigyn_lidar"), "DEBUG: ModularLD06Parser constructor called");
}

ModularLD06Parser::~ModularLD06Parser() = default;

bool ModularLD06Parser::parseBytes(const uint8_t* data, size_t len, double now_sec) {
  stats_.bytes += len;
  
  // Use existing proven implementation
  bool found_data = impl_->legacy_parser_.parseBytes(data, len, now_sec);
  
  // Check for completed frames and convert them
  while (impl_->legacy_parser_.hasCompleteFrame()) {
    ScanFrame legacy_frame = impl_->legacy_parser_.takeFrame();
    if (legacy_frame.valid()) {
      LidarScanFrame converted_frame = impl_->convertFrame(legacy_frame);
      
      // Apply motion compensation if enabled
      if (enable_motion_compensation_) {
        applyMotionCompensation(converted_frame);
        if (converted_frame.is_motion_compensated) {
          stats_.motion_compensated_frames++;
        }
      }
      
      impl_->completed_frames_.push_back(std::move(converted_frame));
      if (impl_->completed_frames_.size() > 4) {
        impl_->completed_frames_.pop_front();
      }
      stats_.frames++;
    }
  }
  
  return found_data;
}

bool ModularLD06Parser::hasCompleteFrame() const {
  return !impl_->completed_frames_.empty();
}

LidarScanFrame ModularLD06Parser::takeFrame() {
  if (impl_->completed_frames_.empty()) {
    return {};
  }
  
  LidarScanFrame frame = std::move(impl_->completed_frames_.front());
  impl_->completed_frames_.pop_front();
  return frame;
}

void LD06Parser::reset() {
  impl_->legacy_parser_.reset();
  impl_->completed_frames_.clear();
  motion_buffer_.clear();
}

void LD06Parser::setSkipCRC(bool skip) {
  skip_crc_ = skip;
  impl_->legacy_parser_.setSkipCRC(skip);
}

void LD06Parser::setMotionCompensation(bool enable) {
  enable_motion_compensation_ = enable;
}

void LD06Parser::updateMotionData(const MotionData& motion) {
  motion_buffer_.push_back(motion);
  while (motion_buffer_.size() > 100) {
    motion_buffer_.pop_front();
  }
  stats_.motion_data_available = !motion_buffer_.empty();
}

std::string LD06Parser::getStatsString() const {
  char buf[256];
  snprintf(buf, sizeof(buf), "bytes=%lu pkts=%lu frames=%lu motion_comp=%lu motion_avail=%s",
    stats_.bytes, stats_.packets, stats_.frames, stats_.motion_compensated_frames,
    stats_.motion_data_available ? "yes" : "no");
  return std::string(buf);
}

void LD06Parser::applyMotionCompensation(LidarScanFrame& frame) {
  if (motion_buffer_.empty() || frame.beams.empty()) {
    return;
  }
  
  // Apply motion compensation to each beam
  for (auto& beam : frame.beams) {
    double beam_timestamp = frame.scan_start_time + beam.relative_time;
    MotionData motion = interpolateMotion(beam_timestamp);
    
    if (motion.valid) {
      double time_offset = beam.relative_time;
      double yaw_correction = motion.yaw_rate * time_offset;
      
      beam.deskewed_angle_rad = beam.angle_rad - yaw_correction;
      while (beam.deskewed_angle_rad < 0) beam.deskewed_angle_rad += 2.0 * M_PI;
      while (beam.deskewed_angle_rad >= 2.0 * M_PI) beam.deskewed_angle_rad -= 2.0 * M_PI;
      
      beam.is_deskewed = true;
    } else {
      beam.deskewed_angle_rad = beam.angle_rad;
      beam.is_deskewed = false;
    }
  }
  
  frame.is_motion_compensated = true;
}

MotionData LD06Parser::interpolateMotion(double timestamp) const {
  if (motion_buffer_.size() < 2) {
    return motion_buffer_.empty() ? MotionData{} : motion_buffer_.back();
  }
  
  // Simple interpolation logic
  auto lower = std::lower_bound(motion_buffer_.begin(), motion_buffer_.end(), timestamp,
    [](const MotionData& m, double t) { return m.timestamp < t; });
  
  if (lower == motion_buffer_.begin()) return motion_buffer_.front();
  if (lower == motion_buffer_.end()) return motion_buffer_.back();
  
  auto upper = lower--;
  double dt = upper->timestamp - lower->timestamp;
  if (dt <= 0.0) return *lower;
  
  double alpha = (timestamp - lower->timestamp) / dt;
  
  MotionData result;
  result.timestamp = timestamp;
  result.yaw_rate = lower->yaw_rate + alpha * (upper->yaw_rate - lower->yaw_rate);
  result.yaw_angle = lower->yaw_angle + alpha * (upper->yaw_angle - lower->yaw_angle);
  result.valid = lower->valid && upper->valid;
  
  return result;
}

// Factory implementation
std::unique_ptr<LidarParserInterface> LidarParserFactory::createParser(const std::string& parser_type) {
  if (parser_type == "ld06") {
    return std::make_unique<LD06Parser>();
  }
  
  // Future parsers: ld19, rplidar, etc.
  return nullptr;
}

std::vector<std::string> LidarParserFactory::getSupportedParsers() {
  return {"ld06"};
}

}
