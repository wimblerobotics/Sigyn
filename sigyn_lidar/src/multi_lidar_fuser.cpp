#include "sigyn_lidar/multi_lidar_fuser.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

namespace sigyn_lidar {

    bool MultiLidarFuser::allowAngle(double ang, const std::vector<AngleSegment>& segs) {
        if (segs.empty()) return true;
        for (const auto& s : segs) {
            double a = ang;
            if (s.start_rad <= s.end_rad) {
                if (a >= s.start_rad && a < s.end_rad) return true;
            }
            else { // wrap
                if (a >= s.start_rad || a < s.end_rad) return true;
            }
        }
        return false;
    }

    sensor_msgs::msg::LaserScan MultiLidarFuser::buildScanMsg(const ScanFrame& frame, const LidarConfig& cfg) {
        sensor_msgs::msg::LaserScan out;
        out.header.stamp = rclcpp::Time((int64_t)(frame.scan_end_time * 1e9));
        out.header.frame_id = cfg.frame_id;
        out.angle_min = 0.0; out.angle_max = 2 * M_PI;
        size_t beam_count = frame.beams.size();
        out.angle_increment = beam_count > 1 ? (2 * M_PI) / beam_count : 0.0;
        out.scan_time = frame.scan_end_time - frame.scan_start_time;
        out.time_increment = (beam_count > 0) ? out.scan_time / beam_count : 0.0;
        out.range_min = 0.05; out.range_max = 15.0;
        size_t discrete = beam_count; // keep original count (simple assumption for now)
        if (discrete == 0) return out;
        out.ranges.assign(discrete, std::numeric_limits<float>::infinity());
        out.intensities.assign(discrete, 0.0f);
        for (size_t i = 0;i < frame.beams.size();++i) {
            const auto& b = frame.beams[i];
            double ang = b.angle_rad;
            if (cfg.invert) ang = 2 * M_PI - ang;
            if (!allowAngle(ang, cfg.pass_segments)) continue;
            int idx = (int)((ang - out.angle_min) / (2 * M_PI) * discrete);
            if (idx < 0) idx = 0; if (idx >= (int)discrete) idx = (int)discrete - 1;
            // choose nearer
            if (std::isinf(out.ranges[idx]) || b.range_m < out.ranges[idx]) {
                out.ranges[idx] = b.range_m;
                out.intensities[idx] = b.confidence;
            }
        }
        return out;
    }

    sensor_msgs::msg::LaserScan MultiLidarFuser::fuse(const std::vector<sensor_msgs::msg::LaserScan>& scans, const FusedScanConfig& cfg) {
        sensor_msgs::msg::LaserScan fused;
        if (scans.empty()) return fused;
        fused = scans.front();
        fused.header.frame_id = cfg.fused_frame;
        for (size_t s = 1; s < scans.size(); ++s) {
            const auto& other = scans[s];
            for (size_t i = 0;i < fused.ranges.size() && i < other.ranges.size(); ++i) {
                float r1 = fused.ranges[i]; float r2 = other.ranges[i];
                if (std::isinf(r1) || (r2 < r1 && !std::isinf(r2))) {
                    fused.ranges[i] = r2; fused.intensities[i] = other.intensities[i];
                }
            }
        }
        return fused;
    }

}
