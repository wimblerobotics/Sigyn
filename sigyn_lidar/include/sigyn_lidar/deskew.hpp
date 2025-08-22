#pragma once
#include <vector>
#include <functional>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include "sigyn_lidar/ld06_protocol.hpp"

namespace sigyn_lidar {

// Provides yaw interpolation for deskew using IMU angular velocity (gz) + optional wheel odom yaw rate fallback.
class YawInterpolator {
public:
  struct Sample { double t; double yaw; double yaw_rate; };
  void addSample(double t, double yaw, double yaw_rate);
  // Returns yaw at time t (linear interp), or std::nullopt
  bool interpolate(double t, double& yaw_out) const;
private:
  std::vector<Sample> samples_; // time-ordered small ring buffer
};

// Deskew a frame in-place given yaw(t) function.
struct DeskewConfig { bool enabled = true; };

inline void deskewBeams(std::vector<Beam>& beams, double start_t, const YawInterpolator& interp, const DeskewConfig& cfg) {
  if(!cfg.enabled) return;
  double yaw0; if(!interp.interpolate(start_t, yaw0)) return;
  for(auto& b : beams) {
    double yaw_i; if(!interp.interpolate(start_t + b.relative_time, yaw_i)) continue;
    double dyaw = yaw_i - yaw0;
    b.angle_rad -= dyaw; // compensate rotation of base during acquisition
    // normalize
    while(b.angle_rad < 0) b.angle_rad += 2*M_PI;
    while(b.angle_rad >= 2*M_PI) b.angle_rad -= 2*M_PI;
  }
}

}
