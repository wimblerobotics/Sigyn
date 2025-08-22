#pragma once
#include <cstdint>
#include <vector>
#include <optional>
#include <deque>
#include <cmath>
#include <array>

namespace sigyn_lidar {

    // LD06 packet constants (public / reverse engineered)
    struct LD06Point { uint16_t distance_mm; uint8_t confidence; };  // Note: "confidence" is actually intensity (0-255)
    struct LD06PacketRaw {
        uint8_t header;     // 0x54
        uint8_t ver_len;    // 0x2C (or length | version bits)
        uint16_t speed;     // raw speed field used for angle increment derivation
        uint16_t start_angle; // hundredth degrees
        LD06Point points[12];
        uint16_t end_angle;   // hundredth degrees
        uint16_t timestamp;   // ms (rollover)
        uint8_t crc8;
    } __attribute__((packed));

    struct Beam {
        float angle_rad{ 0.f };      // radians [0,2pi)
        float range_m{ 0.f };        // meters
        float confidence{ 0.f };     // 0-255 intensity from LD06
        double relative_time{ 0.0 }; // seconds since start of scan
        float deskewed_angle_rad{ 0.f }; // motion-compensated angle
        bool is_deskewed{ false };   // whether motion compensation was applied
    };

    struct ScanFrame {
        std::vector<Beam> beams; // ordered by angle
        double scan_start_time{ 0.0 };  // ROS time (seconds) at first beam
        double scan_end_time{ 0.0 };    // ROS time (seconds) at last beam
        double revolution_hz{ 0.0 };    // estimated rotational frequency
        uint16_t raw_speed{0};          // raw speed field from last packet in frame
        bool valid() const { return !beams.empty(); }
    };

    struct ParserStats {
        uint64_t bytes = 0, packets = 0, crc_fail = 0, len_mismatch = 0, frames = 0, desync = 0;
        std::array<uint8_t, 8> first_bytes{};
        bool have_sample = false;
        uint8_t last_ver_len = 0;
        uint8_t last_sample_count = 0;
        uint16_t last_expected_len = 0;
    };

    class LD06Parser {
    public:
        // Parse bytes with current host time (seconds).
        bool parseBytes(const uint8_t* data, size_t len, double now_sec);
        bool hasCompleteFrame() const { return !frames_.empty(); }
        ScanFrame takeFrame();
        void reset();
        ParserStats getStats() const { return stats_; }
        void setSkipCRC(bool v) { skip_crc_ = v; }
    private:
        bool tryExtractOnePacket(double now_sec);
        void addPacketBeams(const LD06PacketRaw& pkt, double now_sec);
        void finalizeFrame(double now_sec);
        static inline double normalizeAngleRad(double a) {
            while (a < 0) a += 2.0 * 3.14159265358979323846;
            while (a >= 2.0 * 3.14159265358979323846) a -= 2.0 * 3.14159265358979323846;
            return a;
        }
        std::vector<uint8_t> buffer_;
        std::vector<Beam> accumulating_;
        std::deque<ScanFrame> frames_;
        bool have_start_{ false };
        uint16_t last_end_angle_{ 0 }; // hundredth degrees of previous packet end
        double last_packet_time_{ 0.0 };
        double frame_start_time_{ 0.0 };
        ParserStats stats_;
        bool skip_crc_{ false };
        uint16_t first_start_angle_{ 0 };
        uint16_t current_speed_{0};
    };

}
