#include "sigyn_lidar/ld06_parser.hpp"
#include "sigyn_lidar/ld06_protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>

namespace sigyn_lidar {

    class ModularLD06Parser::Impl {
    public:
        LD06Parser legacy_parser_; // low-level protocol parser
        std::deque<LidarScanFrame> completed_frames_;

        LidarScanFrame convertFrame(const ScanFrame& legacy_frame) {
            LidarScanFrame frame;
            frame.scan_start_time = legacy_frame.scan_start_time;
            frame.scan_end_time = legacy_frame.scan_end_time;
            frame.revolution_hz = legacy_frame.revolution_hz;
            frame.is_motion_compensated = false;
            frame.source_parser = "ld06";
            frame.raw_speed = legacy_frame.raw_speed;
            for (const auto& legacy_beam : legacy_frame.beams) {
                LidarBeam beam;
                beam.angle_rad = legacy_beam.angle_rad;
                beam.range_m = legacy_beam.range_m;
                beam.confidence = legacy_beam.confidence;
                beam.relative_time = legacy_beam.relative_time;
                beam.deskewed_angle_rad = legacy_beam.angle_rad;
                beam.is_deskewed = false;
                frame.beams.push_back(beam);
            }
            return frame;
        }
    };

    ModularLD06Parser::ModularLD06Parser() : impl_(std::make_unique<Impl>()) {}
    ModularLD06Parser::~ModularLD06Parser() = default;

    bool ModularLD06Parser::parseBytes(const uint8_t* data, size_t len, double now_sec) {
        stats_.bytes += len;
        bool found_data = impl_->legacy_parser_.parseBytes(data, len, now_sec);
        while (impl_->legacy_parser_.hasCompleteFrame()) {
            ScanFrame legacy_frame = impl_->legacy_parser_.takeFrame();
            if (legacy_frame.valid()) {
                LidarScanFrame converted = impl_->convertFrame(legacy_frame);
                if (enable_motion_compensation_) {
                    applyMotionCompensation(converted);
                    if (converted.is_motion_compensated) stats_.motion_compensated_frames++;
                }
                impl_->completed_frames_.push_back(std::move(converted));
                if (impl_->completed_frames_.size() > 4) impl_->completed_frames_.pop_front();
                stats_.frames++;
            }
        }
        return found_data;
    }

    bool ModularLD06Parser::hasCompleteFrame() const { return !impl_->completed_frames_.empty(); }

    LidarScanFrame ModularLD06Parser::takeFrame() {
        if (impl_->completed_frames_.empty()) return {};
        LidarScanFrame f = std::move(impl_->completed_frames_.front());
        impl_->completed_frames_.pop_front();
        return f;
    }

    void ModularLD06Parser::reset() {
        impl_->legacy_parser_.reset();
        impl_->completed_frames_.clear();
        motion_buffer_.clear();
    }

    void ModularLD06Parser::setSkipCRC(bool skip) {
        skip_crc_ = skip;
        impl_->legacy_parser_.setSkipCRC(skip);
    }

    void ModularLD06Parser::setMotionCompensation(bool enable) { enable_motion_compensation_ = enable; }

    void ModularLD06Parser::updateMotionData(const MotionData& motion) {
        motion_buffer_.push_back(motion);
        while (motion_buffer_.size() > 100) motion_buffer_.pop_front();
        stats_.motion_data_available = !motion_buffer_.empty();
    }

    std::string ModularLD06Parser::getStatsString() const {
        ParserStats p = impl_->legacy_parser_.getStats();
        char buf[256];
        snprintf(buf, sizeof(buf), "bytes=%lu pkts=%lu frames=%lu crc_fail=%lu desync=%lu", p.bytes, p.packets, p.frames, p.crc_fail, p.desync);
        return std::string(buf);
    }

    void ModularLD06Parser::applyMotionCompensation(LidarScanFrame& frame) {
        if (motion_buffer_.empty() || frame.beams.empty()) return;
        for (auto& b : frame.beams) {
            double beam_ts = frame.scan_start_time + b.relative_time;
            MotionData m = interpolateMotion(beam_ts);
            if (m.valid) {
                double yaw_correction = m.yaw_rate * b.relative_time;
                b.deskewed_angle_rad = b.angle_rad - yaw_correction;
                while (b.deskewed_angle_rad < 0) b.deskewed_angle_rad += 2*M_PI;
                while (b.deskewed_angle_rad >= 2*M_PI) b.deskewed_angle_rad -= 2*M_PI;
                b.is_deskewed = true;
            } else {
                b.deskewed_angle_rad = b.angle_rad; b.is_deskewed = false;
            }
        }
        frame.is_motion_compensated = true;
    }

    MotionData ModularLD06Parser::interpolateMotion(double timestamp) const {
        if (motion_buffer_.size() < 2) return motion_buffer_.empty()?MotionData{}:motion_buffer_.back();
        auto lower = std::lower_bound(motion_buffer_.begin(), motion_buffer_.end(), timestamp,
                                      [](const MotionData& m, double t){ return m.timestamp < t; });
        if (lower == motion_buffer_.begin()) return motion_buffer_.front();
        if (lower == motion_buffer_.end()) return motion_buffer_.back();
        auto upper = lower--;
        double dt = upper->timestamp - lower->timestamp;
        if (dt <= 0) return *lower;
        double alpha = (timestamp - lower->timestamp)/dt;
        MotionData r; r.timestamp=timestamp; r.yaw_rate=lower->yaw_rate + alpha*(upper->yaw_rate-lower->yaw_rate); r.yaw_angle=lower->yaw_angle + alpha*(upper->yaw_angle-lower->yaw_angle); r.valid=lower->valid && upper->valid; return r;
    }

    std::unique_ptr<LidarParserInterface> LidarParserFactory::createParser(const std::string& parser_type) {
        if (parser_type == "ld06") return std::make_unique<ModularLD06Parser>();
        return nullptr;
    }

    std::vector<std::string> LidarParserFactory::getSupportedParsers() { return {"ld06"}; }

}
