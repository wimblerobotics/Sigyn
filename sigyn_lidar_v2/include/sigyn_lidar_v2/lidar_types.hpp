/**
 * @file lidar_types.hpp
 * @author Sigyn Robotics
 * @brief Common data structures for multi-LIDAR system
 * @version 1.0.0
 * @date 2025-08-21
 *
 * @copyright Copyright (c) 2025 Sigyn Robotics. All rights reserved.
 * Licensed under the MIT License.
 */

#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace sigyn_lidar_v2 {

  // Raw point data from LIDAR (based on vendor structure)
  struct LidarPoint {
    float angle_deg;        // Angle in degrees [0, 360)
    uint16_t distance_mm;   // Distance in millimeters
    uint8_t intensity;      // Intensity value [0, 255] (vendor calls this "confidence")
    uint64_t timestamp_ns;  // Timestamp in nanoseconds

    LidarPoint() : angle_deg(0.0f), distance_mm(0), intensity(0), timestamp_ns(0) {}
    LidarPoint(float angle, uint16_t dist, uint8_t intens, uint64_t stamp = 0)
      : angle_deg(angle), distance_mm(dist), intensity(intens), timestamp_ns(stamp) {
    }
  };

  // Collection of points from a single LIDAR revolution
  struct LidarScan {
    std::vector<LidarPoint> points;
    uint64_t scan_start_ns;
    uint64_t scan_end_ns;
    float scan_frequency_hz;
    std::string frame_id;
    uint16_t motor_speed; // raw speed value from packets for dynamic angle_increment

    LidarScan() : scan_start_ns(0), scan_end_ns(0), scan_frequency_hz(0.0f), motor_speed(0) {}
  };

  // LIDAR device configuration
  struct LidarConfig {
    std::string device_path;      // e.g., "/dev/lidar_front_center"
    std::string frame_id;         // e.g., "lidar_frame_top_lidar"
    std::string topic_name;       // e.g., "/sigyn/lidar_front_center"
    std::string device_type;      // e.g., "LD06"
    int serial_baudrate;          // e.g., 230400
    bool enable_motion_correction;

    LidarConfig() : serial_baudrate(230400), enable_motion_correction(false) {}
  };

  // Motion data for correction
  struct MotionData {
    uint64_t timestamp_ns;
    double linear_x;        // m/s
    double linear_y;        // m/s  
    double angular_z;       // rad/s
    double pos_x;           // m
    double pos_y;           // m
    double yaw;             // rad

    MotionData() : timestamp_ns(0), linear_x(0), linear_y(0), angular_z(0),
      pos_x(0), pos_y(0), yaw(0) {
    }
  };

} // namespace sigyn_lidar_v2
