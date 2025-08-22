#include "sigyn_lidar/ld06_parser.hpp"
#include "sigyn_lidar/ld06_protocol.hpp"  // Use existing implementation
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

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
            // Note: is_motion_compensated may not exist in legacy struct
            frame.is_motion_compensated = false;  // Default to false for now

            // Convert beam data
            for (const auto& legacy_beam : legacy_frame.beams) {
                LidarBeam beam;
                beam.angle_rad = legacy_beam.angle_rad;
                beam.range_m = legacy_beam.range_m;
                beam.confidence = legacy_beam.confidence;
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
        // Parse using legacy parser
        bool parsed = impl_->legacy_parser_.parseBytes(data, len, now_sec);

        // Check for completed frames and convert them
        while (impl_->legacy_parser_.hasCompleteFrame()) {
            try {
                ScanFrame legacy_frame = impl_->legacy_parser_.takeFrame();
                LidarScanFrame modular_frame = impl_->convertFrame(legacy_frame);
                impl_->completed_frames_.push_back(modular_frame);
            }
            catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("sigyn_lidar"),
                    "Error converting frame: %s", e.what());
                break;
            }
        }

        return parsed;
    }

    bool ModularLD06Parser::hasCompleteFrame() const {
        return !impl_->completed_frames_.empty();
    }

    LidarScanFrame ModularLD06Parser::takeFrame() {
        if (impl_->completed_frames_.empty()) {
            throw std::runtime_error("No complete frames available");
        }

        LidarScanFrame frame = impl_->completed_frames_.front();
        impl_->completed_frames_.pop_front();
        return frame;
    }

    void ModularLD06Parser::reset() {
        impl_->legacy_parser_.reset();
        impl_->completed_frames_.clear();
    }

    void ModularLD06Parser::setSkipCRC(bool skip) {
        impl_->legacy_parser_.setSkipCRC(skip);
        // Also store locally for debugging
        RCLCPP_INFO(rclcpp::get_logger("sigyn_lidar"), "CRC skip setting: %s", skip ? "enabled" : "disabled");
    }

    void ModularLD06Parser::setMotionCompensation(bool enable) {
        // Stub implementation - motion compensation will be handled in this parser
        (void)enable;  // Suppress unused parameter warning
    }

    void ModularLD06Parser::updateMotionData(const MotionData& motion) {
        // Stub implementation - store motion data for future use
        (void)motion;  // Suppress unused parameter warning
    }

    std::string ModularLD06Parser::getStatsString() const {
        // Return basic stats - could be enhanced to include motion compensation stats
        return "ModularLD06Parser: active";
    }

    // Motion compensation methods (these would need proper implementation)
    void ModularLD06Parser::applyMotionCompensation(LidarScanFrame& frame) {
        // Placeholder implementation
        // In practice, this would need proper IMU integration
        RCLCPP_DEBUG_THROTTLE(rclcpp::get_logger("sigyn_lidar"), *rclcpp::Clock::make_shared().get(), 5000,
            "Motion compensation not yet implemented in modular parser");
    }

    MotionData ModularLD06Parser::interpolateMotion(double timestamp) const {
        // Placeholder implementation
        MotionData motion;
        motion.timestamp = timestamp;
        motion.yaw_rate = 0.0;
        motion.valid = false;
        return motion;
    }

    // Factory implementation
    std::unique_ptr<LidarParserInterface> LidarParserFactory::createParser(const std::string& parser_type) {
        if (parser_type == "ld06") {
            return std::make_unique<ModularLD06Parser>();
        }
        // Add more parser types here in the future
        return nullptr;
    }

    std::vector<std::string> LidarParserFactory::getSupportedParsers() {
        return { "ld06" };
    }

} // namespace sigyn_lidar
