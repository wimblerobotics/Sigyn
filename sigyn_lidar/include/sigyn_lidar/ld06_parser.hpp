#pragma once

#include "sigyn_lidar/lidar_parser_interface.hpp"
#include <memory>
#include <deque>

namespace sigyn_lidar {

    // LD06-specific parser that leverages existing ldlidar package
    class ModularLD06Parser : public LidarParserInterface {
    public:
        ModularLD06Parser();
        ~ModularLD06Parser() override;

        // LidarParserInterface implementation
        bool parseBytes(const uint8_t* data, size_t len, double now_sec) override;
        bool hasCompleteFrame() const override;
        LidarScanFrame takeFrame() override;
        void reset() override;

        void setSkipCRC(bool skip) override;
        void setMotionCompensation(bool enable) override;
        void updateMotionData(const MotionData& motion) override;

        std::string getStatsString() const override;
        std::string getParserType() const override { return "ld06"; }

    private:
        class Impl;
        std::unique_ptr<Impl> impl_;

        bool enable_motion_compensation_{ false };
        bool skip_crc_{ false };
        std::deque<MotionData> motion_buffer_;

        struct Stats {
            uint64_t bytes{ 0 }, packets{ 0 }, frames{ 0 };
            uint64_t motion_compensated_frames{ 0 };
            bool motion_data_available{ false };
        } stats_;

        void applyMotionCompensation(LidarScanFrame& frame);
        MotionData interpolateMotion(double timestamp) const;
    };

}
