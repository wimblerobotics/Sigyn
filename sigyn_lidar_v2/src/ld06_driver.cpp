/**
 * @file ld06_driver.cpp
 * @author Sigyn Robotics
 * @brief LD06 LiDAR driver implementation based on LDROBOT SDK
 * @version 1.0.0
 * @date 2025-08-21
 * 
 * @copyright Copyright (c) 2025 Sigyn Robotics. All rights reserved.
 * Licensed under the MIT License.
 * 
 * This implementation incorporates CRC calculation, protocol parsing, and
 * data processing algorithms from the LDROBOT LiDAR SDK:
 * 
 * Original work: LDROBOT LiDAR SDK
 * Copyright (c) 2017-2023 SHENZHEN LDROBOT CO., LTD.
 * Repository: https://github.com/ldrobotSensorTeam/ldlidar_ros2.git
 * Authors: LDROBOT Team (support@ldrobot.com)
 * Licensed under the MIT License.
 * 
 * Key vendor implementations used:
 * - CRC8 lookup table and calculation algorithm
 * - LD06 packet structure definitions  
 * - Protocol state machine logic
 * - Angle interpolation algorithms
 * - Revolution detection methods
 */

#include "sigyn_lidar_v2/ld06_driver.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <sstream>

namespace sigyn_lidar_v2 {

// CRC lookup table from vendor implementation
const uint8_t LD06Driver::crc_table_[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

LD06Driver::LD06Driver() 
  : parse_state_(WAIT_HEADER)
  , bytes_needed_(1)
  , scan_start_timestamp_(0)
  , last_packet_timestamp_(0)
  , last_angle_(0.0f)
  , motor_speed_(0)
  , serial_fd_(-1)
  , is_running_(false) {
  
  packet_buffer_.reserve(LD06_PACKET_SIZE);
  current_scan_points_.reserve(500); // Approximate points per scan
}

LD06Driver::~LD06Driver() {
  stop();
}

bool LD06Driver::configure(const LidarConfig& config) {
  config_ = config;
  return true;
}

bool LD06Driver::start() {
  if (is_running_) {
    return true;
  }
  
  // Open serial port
  serial_fd_ = open(config_.device_path.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
  if (serial_fd_ < 0) {
    return false;
  }
  
  // Configure serial port
  struct termios options;
  tcgetattr(serial_fd_, &options);
  
  // Set baud rate
  cfsetispeed(&options, B230400);
  cfsetospeed(&options, B230400);
  
  // Configure 8N1
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  
  // Raw input
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  
  tcsetattr(serial_fd_, TCSANOW, &options);
  
  is_running_ = true;
  return true;
}

bool LD06Driver::stop() {
  if (!is_running_) {
    return true;
  }
  
  is_running_ = false;
  
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
  
  return true;
}

bool LD06Driver::is_connected() const {
  return is_running_ && serial_fd_ >= 0;
}

bool LD06Driver::process_data(const uint8_t* data, size_t length) {
  if (!data || length == 0) {
    return false;
  }
  
  stats_.bytes_processed += length;
  
  bool found_complete_packet = false;
  for (size_t i = 0; i < length; ++i) {
    if (parse_byte(data[i])) {
      found_complete_packet = true;
    }
  }
  
  return found_complete_packet;
}

bool LD06Driver::parse_byte(uint8_t byte) {
  switch (parse_state_) {
    case WAIT_HEADER:
      if (byte == LD06_PKG_HEADER) {
        packet_buffer_.clear();
        packet_buffer_.push_back(byte);
        parse_state_ = WAIT_VER_LEN;
        bytes_needed_ = 1;
      }
      break;
      
    case WAIT_VER_LEN:
      packet_buffer_.push_back(byte);
      if (byte == LD06_DATA_PKG_INFO) {
        parse_state_ = COLLECT_DATA;
        bytes_needed_ = LD06_PACKET_SIZE - 2; // Remaining bytes after header and ver_len
      } else {
        // Invalid packet, restart
        parse_state_ = WAIT_HEADER;
        bytes_needed_ = 1;
      }
      break;
      
    case COLLECT_DATA:
      packet_buffer_.push_back(byte);
      bytes_needed_--;
      
      if (bytes_needed_ == 0) {
        // Complete packet received
        if (packet_buffer_.size() == LD06_PACKET_SIZE) {
          LD06Packet packet;
          std::memcpy(&packet, packet_buffer_.data(), LD06_PACKET_SIZE);
          
          // Verify CRC
          uint8_t calculated_crc = calculate_crc8(packet_buffer_.data(), LD06_PACKET_SIZE - 1);
          if (calculated_crc == packet.crc8) {
            stats_.packets_received++;
            process_packet(packet);
          } else {
            stats_.packets_with_crc_error++;
          }
        }
        
        // Reset for next packet
        parse_state_ = WAIT_HEADER;
        bytes_needed_ = 1;
        return true;
      }
      break;
  }
  
  return false;
}

bool LD06Driver::process_packet(const LD06Packet& packet) {
  uint64_t packet_timestamp = get_current_timestamp_ns();
  motor_speed_ = packet.speed;
  
  // Convert start and end angles from 0.01 degree units to degrees
  float start_angle_deg = packet.start_angle / 100.0f;
  float end_angle_deg = packet.end_angle / 100.0f;
  
  // Calculate angle step between points
  float angle_diff = (end_angle_deg + 360.0f - start_angle_deg);
  if (angle_diff > 360.0f) angle_diff -= 360.0f;
  float angle_step = angle_diff / (LD06_POINT_PER_PACK - 1);
  
  // Process each point in the packet
  for (int i = 0; i < LD06_POINT_PER_PACK; ++i) {
    float angle = start_angle_deg + i * angle_step;
    if (angle >= 360.0f) angle -= 360.0f;
    
    LidarPoint point;
    point.angle_deg = angle;
    point.distance_mm = packet.point[i].distance;
    point.intensity = packet.point[i].intensity;
    point.timestamp_ns = packet_timestamp;
    
    current_scan_points_.push_back(point);
  }
  
  // Check for scan completion (angle wrap detection)
  if (current_scan_points_.size() > 12) { // Need at least one packet
    if (end_angle_deg < 20.0f && last_angle_ > 340.0f) {
      // Detected wrap - complete current scan
      finalize_scan();
      return true;
    }
  }
  
  last_angle_ = end_angle_deg;
  last_packet_timestamp_ = packet_timestamp;
  
  return false;
}

void LD06Driver::finalize_scan() {
  if (current_scan_points_.empty()) {
    return;
  }
  
  LidarScan scan;
  scan.points = std::move(current_scan_points_);
  scan.scan_start_ns = scan_start_timestamp_ > 0 ? scan_start_timestamp_ : scan.points.front().timestamp_ns;
  scan.scan_end_ns = last_packet_timestamp_;
  scan.frame_id = config_.frame_id;
  
  // Calculate scan frequency
  double scan_duration_s = (scan.scan_end_ns - scan.scan_start_ns) / 1e9;
  scan.scan_frequency_hz = scan_duration_s > 0 ? 1.0f / scan_duration_s : 0.0f;
  
  complete_scans_.push_back(std::move(scan));
  stats_.scans_completed++;
  
  // Keep only the latest few scans
  if (complete_scans_.size() > 5) {
    complete_scans_.pop_front();
  }
  
  // Reset for next scan
  current_scan_points_.clear();
  scan_start_timestamp_ = last_packet_timestamp_;
  
  // Notify callback if set
  if (scan_callback_ && !complete_scans_.empty()) {
    scan_callback_(complete_scans_.back());
  }
}

bool LD06Driver::has_complete_scan() const {
  return !complete_scans_.empty();
}

LidarScan LD06Driver::get_scan() {
  if (complete_scans_.empty()) {
    return LidarScan();
  }
  
  LidarScan scan = std::move(complete_scans_.front());
  complete_scans_.pop_front();
  return scan;
}

std::string LD06Driver::get_device_info() const {
  return "LD06 LiDAR Driver v1.0";
}

std::string LD06Driver::get_status_string() const {
  std::stringstream ss;
  ss << "LD06 Status: ";
  ss << "Packets=" << stats_.packets_received;
  ss << ", CRC_Errors=" << stats_.packets_with_crc_error;
  ss << ", Scans=" << stats_.scans_completed;
  ss << ", Bytes=" << stats_.bytes_processed;
  ss << ", Speed=" << motor_speed_;
  return ss.str();
}

void LD06Driver::set_scan_callback(ScanCallback callback) {
  scan_callback_ = callback;
}

uint8_t LD06Driver::calculate_crc8(const uint8_t* data, size_t length) const {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = crc_table_[(crc ^ data[i]) & 0xFF];
  }
  return crc;
}

uint64_t LD06Driver::get_current_timestamp_ns() const {
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace sigyn_lidar_v2
