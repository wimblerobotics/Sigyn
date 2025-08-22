#pragma once
#include <cstdint>
#include <vector>
#include <optional>
#include <deque>
#include <cmath>
#include <array>
#include <memory>
#include <string>
#include <functional>
#include <sensor_msgs/msg/laser_scan.hpp>

// Use existing structures from ld06_protocol.hpp to avoid redefinition
#include "sigyn_lidar/ld06_protocol.hpp"
#include "sigyn_lidar/multi_lidar_fuser.hpp"  // For AngleSegment

namespace sigyn_lidar {

    // Motion compensation data structure for IMU integration
    struct MotionData {
        double timestamp{ 0.0 };     // ROS time when motion was measured
        double yaw_rate{ 0.0 };      // rad/s - rotational velocity around Z-axis
        double yaw_angle{ 0.0 };     // rad - absolute yaw orientation
        bool valid{ false };         // whether this data is reliable
    };

    // Extend existing Beam structure with motion compensation fields
    struct EnhancedBeam : public Beam {
        double relative_time{ 0.0 };    // seconds since start of scan
        float deskewed_angle_rad{ 0.f }; // motion-compensated angle
        bool is_deskewed{ false };      // whether motion compensation was applied
    };

    // Extend existing ScanFrame with motion compensation
    struct EnhancedScanFrame {
        std::vector<EnhancedBeam> beams; // ordered by angle
        double scan_start_time{ 0.0 };  // ROS time (seconds) at first beam
        double scan_end_time{ 0.0 };    // ROS time (seconds) at last beam
        double revolution_hz{ 0.0 };    // estimated rotational frequency
        bool is_motion_compensated{ false }; // whether de-skewing was applied
        bool valid() const { return !beams.empty(); }
    };

    // Wrapper around LDROBOT's proven parsing with motion compensation
    class LD06MotionCompensatedParser {
    public:
        LD06MotionCompensatedParser();
        ~LD06MotionCompensatedParser();

        // Core parsing using LDROBOT's proven logic
        bool parseBytes(const uint8_t* data, size_t len, double now_sec);
        bool hasCompleteFrame() const;
        EnhancedScanFrame takeFrame();
        void reset();

        // Motion compensation settings
        void setMotionCompensation(bool enable) { enable_motion_compensation_ = enable; }
        void updateMotionData(const MotionData& motion);

        // Configuration
        void setSkipCRC(bool skip) { skip_crc_ = skip; }

        // Statistics
        struct Stats {
            uint64_t bytes{ 0 }, packets{ 0 }, frames{ 0 }, crc_fail{ 0 };
            uint64_t motion_compensated_frames{ 0 };
            bool motion_data_available{ false };
        };
        Stats getStats() const { return stats_; }

    private:
        class Impl;  // Forward declaration for PIMPL pattern
        std::unique_ptr<Impl> impl_;

        bool enable_motion_compensation_{ false };
        bool skip_crc_{ false };
        Stats stats_;

        // Motion data buffer for interpolation
        std::deque<MotionData> motion_buffer_;
        static constexpr size_t MAX_MOTION_BUFFER_SIZE = 100;

        // Apply motion compensation to a frame
        void applyMotionCompensation(EnhancedScanFrame& frame);
        MotionData interpolateMotion(double timestamp) const;
    };

    // Utility functions for enhanced frames
    void applySegments(EnhancedScanFrame& frame, const std::vector<AngleSegment>& segments);
    sensor_msgs::msg::LaserScan buildMsgFromFrame(const EnhancedScanFrame& frame, const std::string& frame_id);

}
