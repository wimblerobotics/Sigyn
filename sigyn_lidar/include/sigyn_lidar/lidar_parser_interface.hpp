#pragma once

#include <vector>
#include <cstdint>
#include <memory>
#include <string>

namespace sigyn_lidar {

    // Generic beam structure for all lidar types
    struct LidarBeam {
        float angle_rad{ 0.f };         // radians [0,2pi) - original angle
        float range_m{ 0.f };           // meters
        float confidence{ 0.f };        // 0-255 intensity/reflectance (LD06 calls this "confidence")
        double relative_time{ 0.0 };    // seconds since start of scan
        float deskewed_angle_rad{ 0.f }; // motion-compensated angle
        bool is_deskewed{ false };      // whether motion compensation was applied
    };

    // Generic scan frame for all lidar types
    struct LidarScanFrame {
        std::vector<LidarBeam> beams;
        double scan_start_time{ 0.0 };
        double scan_end_time{ 0.0 };
        double revolution_hz{ 0.0 };
        bool is_motion_compensated{ false };
        std::string source_parser;  // Which parser generated this frame
        uint16_t raw_speed{0};      // Optional raw speed field from device (e.g., LD06) for angle increment estimation
        bool valid() const { return !beams.empty(); }
    };

    // Motion compensation data interface
    struct MotionData {
        double timestamp{ 0.0 };
        double yaw_rate{ 0.0 };
        double yaw_angle{ 0.0 };
        bool valid{ false };
    };

    // Abstract parser interface for all lidar types
    class LidarParserInterface {
    public:
        virtual ~LidarParserInterface() = default;

        // Core parsing API
        virtual bool parseBytes(const uint8_t* data, size_t len, double now_sec) = 0;
        virtual bool hasCompleteFrame() const = 0;
        virtual LidarScanFrame takeFrame() = 0;
        virtual void reset() = 0;

        // Configuration
        virtual void setSkipCRC(bool skip) = 0;
        virtual void setMotionCompensation(bool enable) = 0;
        virtual void updateMotionData(const MotionData& motion) = 0;

        // Statistics - each parser can define its own stats
        virtual std::string getStatsString() const = 0;

        // Parser identification
        virtual std::string getParserType() const = 0;
    };

    // Factory for creating parser instances
    class LidarParserFactory {
    public:
        static std::unique_ptr<LidarParserInterface> createParser(const std::string& parser_type);
        static std::vector<std::string> getSupportedParsers();
    };

}
