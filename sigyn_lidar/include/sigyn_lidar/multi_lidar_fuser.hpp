#pragma once
#include <vector>
#include <string>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "ld06_protocol.hpp"

namespace sigyn_lidar {

struct AngleSegment { double start_rad; double end_rad; }; // inclusive start, exclusive end

struct LidarConfig {
  std::string device;        // /dev/ttyXXX
  std::string frame_id;      // frame for this lidar
  std::vector<AngleSegment> pass_segments; // segments to keep
  bool invert = false;       // optional angle inversion
};

struct FusedScanConfig {
  std::string fused_frame = "lidar_frame_fused";
  double range_min = 0.05;
  double range_max = 15.0;
  double min_confidence = 0.0;
};

class MultiLidarFuser {
public:
  static sensor_msgs::msg::LaserScan buildScanMsg(const ScanFrame& frame, const LidarConfig& cfg);
  static sensor_msgs::msg::LaserScan fuse(const std::vector<sensor_msgs::msg::LaserScan>& scans, const FusedScanConfig& cfg);
  static bool allowAngle(double ang, const std::vector<AngleSegment>& segs);
};

}
