#include "sigyn_lidar/ld06_motion_compensated.hpp"
#include <cstring>
#include <cmath>
#include <algorithm>
#include <sensor_msgs/msg/laser_scan.hpp>

// Include LDROBOT's proven parsing logic
extern "C" {
  // We'll use their data structures and CRC logic directly
  static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
  };

  enum {
    PKG_HEADER = 0x54,
    PKG_VER_LEN = 0x2C,
    POINT_PER_PACK = 12,
  };

  typedef struct __attribute__((packed)) {
    uint16_t distance;
    uint8_t confidence;
  } LidarPointStructDef;

  typedef struct __attribute__((packed)) {
    uint8_t header;
    uint8_t ver_len;
    uint16_t speed;
    uint16_t start_angle;
    LidarPointStructDef point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
  } LiDARFrameTypeDef;
}

namespace sigyn_lidar {

// PIMPL implementation using LDROBOT's proven parsing
class LD06MotionCompensatedParser::Impl {
public:
  std::vector<uint8_t> data_buffer_;
  std::deque<EnhancedScanFrame> completed_frames_;
  std::vector<EnhancedBeam> current_frame_beams_;
  
  double frame_start_time_ {0.0};
  bool frame_in_progress_ {false};
  uint16_t last_end_angle_ {0};
  
  // Statistics
  uint64_t error_times_ {0};
  uint16_t speed_ {0};
  uint16_t timestamp_ {0};

  // LDROBOT's proven packet parsing logic (adapted)
  bool parsePacket(const uint8_t* data, size_t len, double now_sec, bool skip_crc) {
    data_buffer_.insert(data_buffer_.end(), data, data + len);
    
    bool found_packet = false;
    while (data_buffer_.size() >= sizeof(LiDARFrameTypeDef)) {
      // Find packet header using LDROBOT's approach
      auto it = std::find(data_buffer_.begin(), data_buffer_.end(), PKG_HEADER);
      if (it == data_buffer_.end()) {
        data_buffer_.clear();
        break;
      }
      
      size_t offset = std::distance(data_buffer_.begin(), it);
      if (offset > 0) {
        data_buffer_.erase(data_buffer_.begin(), it);
      }
      
      if (data_buffer_.size() < sizeof(LiDARFrameTypeDef)) break;
      
      // Check for valid version/length
      if (data_buffer_[1] != PKG_VER_LEN) {
        data_buffer_.erase(data_buffer_.begin());
        continue;
      }
      
      // Parse packet using LDROBOT's exact logic
      LiDARFrameTypeDef* pkg = (LiDARFrameTypeDef*)data_buffer_.data();
      
      // CRC validation (LDROBOT's method)
      bool crc_valid = true;
      if (!skip_crc) {
        uint8_t crc = 0;
        for (uint32_t i = 0; i < sizeof(LiDARFrameTypeDef) - 1; i++) {
          crc = CrcTable[(crc ^ data_buffer_[i]) & 0xff];
        }
        crc_valid = (crc == pkg->crc8);
      }
      
      if (crc_valid) {
        // LDROBOT's angle validation
        double diff = fmod((pkg->end_angle / 100.0 - pkg->start_angle / 100.0 + 360), 360);
        if (diff <= (double)pkg->speed * POINT_PER_PACK / 4500.0 * 3.0 / 2.0) {
          processValidPacket(*pkg, now_sec);
          found_packet = true;
        } else {
          error_times_++;
        }
      }
      
      // Remove processed packet
      data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + sizeof(LiDARFrameTypeDef));
    }
    
    return found_packet;
  }
  
private:
  void processValidPacket(const LiDARFrameTypeDef& pkg, double now_sec) {
    speed_ = pkg.speed;
    timestamp_ = pkg.timestamp;
    
    // LDROBOT's exact angle calculation
    uint32_t diff = ((uint32_t)pkg.end_angle + 36000 - (uint32_t)pkg.start_angle) % 36000;
    float step = diff / (POINT_PER_PACK - 1) / 100.0f;
    float start = (double)pkg.start_angle / 100.0f;
    float end = (double)(pkg.end_angle % 36000) / 100.0f;
    
    // Start new frame if needed
    if (!frame_in_progress_) {
      frame_start_time_ = now_sec;
      frame_in_progress_ = true;
      current_frame_beams_.clear();
    }
    
    // Add beams using LDROBOT's proven method
    double packet_duration = 1.0 / (speed_ / 60.0) * (POINT_PER_PACK / 360.0); // rough estimate
    for (int i = 0; i < POINT_PER_PACK; i++) {
      EnhancedBeam beam;
      beam.angle_rad = (start + i * step) * M_PI / 180.0;
      if (beam.angle_rad >= 2.0 * M_PI) beam.angle_rad -= 2.0 * M_PI;
      
      beam.range_m = (float)pkg.point[i].distance / 1000.0f; // LDROBOT uses mm
      beam.confidence = pkg.point[i].confidence;
      beam.relative_time = (now_sec - frame_start_time_) + (i * packet_duration / POINT_PER_PACK);
      beam.deskewed_angle_rad = beam.angle_rad; // Will be updated by motion compensation
      beam.is_deskewed = false;
      
      current_frame_beams_.push_back(beam);
    }
    
    // LDROBOT's frame completion logic - when angle wraps around
    bool frame_complete = false;
    if (current_frame_beams_.size() > 24) { // Need minimum data
      // Check for 360-degree completion using LDROBOT's approach
      float curr_end = end;
      if (!current_frame_beams_.empty()) {
        float prev_angle = current_frame_beams_[current_frame_beams_.size() - 24].angle_rad * 180.0f / M_PI;
        if (curr_end < 20.0f && prev_angle > 340.0f) {
          frame_complete = true;
        }
      }
      
      // Fallback: force completion if too many points
      if (!frame_complete && current_frame_beams_.size() > 400) {
        frame_complete = true;
      }
    }
    
    if (frame_complete) {
      // Finalize frame
      EnhancedScanFrame frame;
      frame.beams = std::move(current_frame_beams_);
      frame.scan_start_time = frame_start_time_;
      frame.scan_end_time = now_sec;
      frame.revolution_hz = frame.scan_end_time > frame.scan_start_time ? 
        1.0 / (frame.scan_end_time - frame.scan_start_time) : 0.0;
      frame.is_motion_compensated = false;
      
      completed_frames_.push_back(std::move(frame));
      if (completed_frames_.size() > 4) {
        completed_frames_.pop_front();
      }
      
      // Reset for next frame
      current_frame_beams_.clear();
      frame_in_progress_ = false;
    }
    
    last_end_angle_ = pkg.end_angle;
  }
};

LD06MotionCompensatedParser::LD06MotionCompensatedParser() 
  : impl_(std::make_unique<Impl>()) {
  // Debug message to confirm this parser is being used
  printf("DEBUG: LD06MotionCompensatedParser constructor called\n");
}

LD06MotionCompensatedParser::~LD06MotionCompensatedParser() = default;

bool LD06MotionCompensatedParser::parseBytes(const uint8_t* data, size_t len, double now_sec) {
  if (!data || len == 0) return false;
  
  stats_.bytes += len;
  bool found_packet = impl_->parsePacket(data, len, now_sec, skip_crc_);
  if (found_packet) {
    stats_.packets++;
  }
  
  return found_packet;
}

bool LD06MotionCompensatedParser::hasCompleteFrame() const {
  return !impl_->completed_frames_.empty();
}

EnhancedScanFrame LD06MotionCompensatedParser::takeFrame() {
  if (impl_->completed_frames_.empty()) return {};
  
  EnhancedScanFrame frame = std::move(impl_->completed_frames_.front());
  impl_->completed_frames_.pop_front();
  
  // Apply motion compensation if enabled
  if (enable_motion_compensation_) {
    applyMotionCompensation(frame);
    if (frame.is_motion_compensated) {
      stats_.motion_compensated_frames++;
    }
  }
  
  stats_.frames++;
  return frame;
}

void LD06MotionCompensatedParser::reset() {
  impl_->data_buffer_.clear();
  impl_->completed_frames_.clear();
  impl_->current_frame_beams_.clear();
  impl_->frame_in_progress_ = false;
  motion_buffer_.clear();
}

void LD06MotionCompensatedParser::updateMotionData(const MotionData& motion) {
  motion_buffer_.push_back(motion);
  
  // Keep buffer size manageable
  while (motion_buffer_.size() > MAX_MOTION_BUFFER_SIZE) {
    motion_buffer_.pop_front();
  }
  
  stats_.motion_data_available = !motion_buffer_.empty();
}

void LD06MotionCompensatedParser::applyMotionCompensation(EnhancedScanFrame& frame) {
  if (motion_buffer_.empty() || frame.beams.empty()) {
    return;
  }
  
  // Apply motion compensation to each beam
  for (auto& beam : frame.beams) {
    double beam_timestamp = frame.scan_start_time + beam.relative_time;
    MotionData motion = interpolateMotion(beam_timestamp);
    
    if (motion.valid) {
      // Calculate motion compensation
      // Assume robot rotates during scan - compensate for yaw motion
      double time_offset = beam.relative_time;
      double yaw_correction = motion.yaw_rate * time_offset;
      
      // Apply correction to angle
      beam.deskewed_angle_rad = beam.angle_rad - yaw_correction;
      
      // Normalize angle
      while (beam.deskewed_angle_rad < 0) beam.deskewed_angle_rad += 2.0 * M_PI;
      while (beam.deskewed_angle_rad >= 2.0 * M_PI) beam.deskewed_angle_rad -= 2.0 * M_PI;
      
      beam.is_deskewed = true;
    } else {
      beam.deskewed_angle_rad = beam.angle_rad;
      beam.is_deskewed = false;
    }
  }
  
  // Sort beams by deskewed angle for proper ordering
  std::sort(frame.beams.begin(), frame.beams.end(), 
    [](const EnhancedBeam& a, const EnhancedBeam& b) {
      float angle_a = a.is_deskewed ? a.deskewed_angle_rad : a.angle_rad;
      float angle_b = b.is_deskewed ? b.deskewed_angle_rad : b.angle_rad;
      return angle_a < angle_b;
    });
  
  frame.is_motion_compensated = true;
}

MotionData LD06MotionCompensatedParser::interpolateMotion(double timestamp) const {
  if (motion_buffer_.size() < 2) {
    return motion_buffer_.empty() ? MotionData{} : motion_buffer_.back();
  }
  
  // Find bounding motion data points
  auto lower = std::lower_bound(motion_buffer_.begin(), motion_buffer_.end(), timestamp,
    [](const MotionData& m, double t) { return m.timestamp < t; });
  
  if (lower == motion_buffer_.begin()) {
    return motion_buffer_.front();
  }
  if (lower == motion_buffer_.end()) {
    return motion_buffer_.back();
  }
  
  // Interpolate between two points
  auto upper = lower;
  --lower;
  
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

// Utility functions (preserved from original implementation)
void applySegments(EnhancedScanFrame& frame, const std::vector<AngleSegment>& segments) {
  if (segments.empty()) return;
  
  frame.beams.erase(
    std::remove_if(frame.beams.begin(), frame.beams.end(),
      [&segments](const EnhancedBeam& beam) {
        float angle = beam.is_deskewed ? beam.deskewed_angle_rad : beam.angle_rad;
        for (const auto& seg : segments) {
          if (angle >= seg.start_rad && angle <= seg.end_rad) {
            return false; // Keep this beam
          }
        }
        return true; // Remove this beam
      }),
    frame.beams.end());
}

sensor_msgs::msg::LaserScan buildMsgFromFrame(const EnhancedScanFrame& frame, const std::string& frame_id) {
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = frame_id;
  
  if (frame.beams.empty()) return msg;
  
  // Use deskewed angles if available
  float min_angle = 0.0f, max_angle = 2.0f * M_PI;
  float angle_increment = 2.0f * M_PI / 360.0f; // 1 degree resolution
  
  msg.angle_min = min_angle;
  msg.angle_max = max_angle;
  msg.angle_increment = angle_increment;
  msg.range_min = 0.05f;
  msg.range_max = 12.0f;
  
  size_t num_readings = static_cast<size_t>((max_angle - min_angle) / angle_increment);
  msg.ranges.assign(num_readings, std::numeric_limits<float>::infinity());
  msg.intensities.assign(num_readings, 0.0f);
  
  for (const auto& beam : frame.beams) {
    float angle = beam.is_deskewed ? beam.deskewed_angle_rad : beam.angle_rad;
    int index = static_cast<int>((angle - min_angle) / angle_increment);
    
    if (index >= 0 && index < static_cast<int>(num_readings)) {
      if (beam.range_m > msg.range_min && beam.range_m < msg.range_max) {
        msg.ranges[index] = beam.range_m;
        msg.intensities[index] = beam.confidence;
      }
    }
  }
  
  return msg;
}

// Remove the MultiLidarFuser implementation from this file to avoid conflicts

}
