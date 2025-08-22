#include "sigyn_lidar/ld06_protocol.hpp"
#include <cstring>
#include <cmath>
#include <cstdio>

namespace sigyn_lidar {

    static const uint8_t HEADER = 0x54;
    static const size_t PACKET_LEN = sizeof(LD06PacketRaw); // 47 bytes fixed for LD06 classic
    // CRC table from LDROBOT docs
    static const uint8_t CRC_TABLE[256] = {
      0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
      0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
      0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
      0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
      0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
      0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
      0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
      0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
      0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
      0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
      0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
      0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
      0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
      0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
      0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
      0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8
    };

    static inline uint8_t crc8_calc(const uint8_t* data, size_t len) {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; i++)
            crc = CRC_TABLE[(crc ^ data[i]) & 0xFF];
        return crc;
    }

    static inline double normalizeAngleRad(double rad) {
        while (rad >= 2 * M_PI) rad -= 2 * M_PI;
        while (rad < 0) rad += 2 * M_PI;
        return rad;
    }

    LD06Parser::LD06Parser() : skip_crc_(true), have_start_(false), first_start_angle_(0), last_end_angle_(0), frame_start_time_(0.0), last_packet_time_(0.0) {
        stats_ = {};
    }

    void LD06Parser::parseBytes(const uint8_t* data, size_t len) {
        if (!data || len == 0) return;

        stats_.bytes += len;
        buffer_.insert(buffer_.end(), data, data + len);

        while (tryExtractOnePacket()) {
            // continue processing packets
        }
    }

    bool LD06Parser::tryExtractOnePacket() {
        double now_sec = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();

        // Look for header pattern: 0x54 followed by 0x2C or 0xEC
        size_t start = 0;
        while (start < buffer_.size()) {
            if (buffer_[start] == 0x54 && (start + 1) < buffer_.size()) {
                uint8_t second = buffer_[start + 1];
                if (second == 0x2C || second == 0xEC) {
                    break; // Found valid header
                }
            }
            start++;
        }

        if (start >= buffer_.size()) return false; // No header found

        if (start != 0) {
            // Remove bytes before header
            buffer_.erase(buffer_.begin(), buffer_.begin() + start);
            stats_.desync += start;
        }

        if (buffer_.size() < PACKET_LEN) return false;

        // Capture first packet for diagnostics
        if (!stats_.have_sample) {
            for (size_t i = 0; i < 8 && i < buffer_.size(); ++i)
                stats_.first_bytes[i] = buffer_[i];
            stats_.have_sample = true;
        }

        LD06PacketRaw pkt;
        std::memcpy(&pkt, buffer_.data(), PACKET_LEN);
        stats_.last_ver_len = pkt.ver_len;

        // CRC validation - skip for now to test frame detection
        bool crc_valid = true;
        if (!skip_crc_) {
            uint8_t computed = crc8_calc(reinterpret_cast<const uint8_t*>(&pkt), PACKET_LEN - 1);
            crc_valid = (computed == pkt.crc8);
            if (!crc_valid) {
                printf("DEBUG: CRC mismatch - computed=0x%02X, packet=0x%02X\n", computed, pkt.crc8);
            }
        }

        if (crc_valid) {
            stats_.packets++;
            addPacketBeams(pkt, now_sec);
            buffer_.erase(buffer_.begin(), buffer_.begin() + PACKET_LEN);
            return true;
        }
        else {
            stats_.crc_fail++;
            // Remove just the header like ldlidar does, not whole packet
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
        }

        return false;
    }

    void LD06Parser::addPacketBeams(const LD06PacketRaw& pkt, double now_sec) {
        uint16_t start_a = pkt.start_angle;
        uint16_t end_a = pkt.end_angle;

        // Debug: log first few packets
        static int packet_count = 0;
        if (packet_count < 5) {
            printf("DEBUG: Packet %d - start_angle=%u, end_angle=%u, speed=%u, timestamp=%u\n",
                packet_count, start_a, end_a, pkt.speed, pkt.timestamp);
            packet_count++;
        }

        if (!have_start_) {
            have_start_ = true;
            frame_start_time_ = now_sec;
            accumulating_.clear();
            first_start_angle_ = start_a;
        }

        // Calculate span for this packet
        uint32_t span = (end_a + 36000 - start_a) % 36000;
        double step_deg = (span / 100.0) / (12 - 1);
        double start_deg = start_a / 100.0;
        double end_deg = end_a / 100.0;

        // Add beams from this packet
        for (int i = 0; i < 12; i++) {
            double angle_deg = start_deg + i * step_deg;
            if (angle_deg >= 360.0) angle_deg -= 360.0;
            double angle_rad = angle_deg * M_PI / 180.0;
            float range_m = pkt.points[i].distance_mm / 1000.0f;
            float conf = pkt.points[i].confidence;
            Beam b{ (float)normalizeAngleRad(angle_rad), range_m, conf, 0.0 };
            accumulating_.push_back(b);
        }

        // Frame completion detection based on official LDROBOT implementation
        // Check for wrap-around condition: when angle drops significantly (completing revolution)
        if (!accumulating_.empty() && accumulating_.size() > 24) { // need minimum data
            // Use end angle for comparison (more reliable than start angle)
            double curr_end_deg = end_deg;

            // Check last few beams for angle wrap detection
            bool wrap_detected = false;
            if (accumulating_.size() >= 12) {
                // Look at angle from 12 beams ago vs current end angle
                double prev_angle_deg = accumulating_[accumulating_.size() - 12].angle_rad * 180.0 / M_PI;

                // Wrap condition: current end is small (< 20°) and previous was large (> 340°)
                // This matches official LDROBOT logic: "(n.angle < 20.0) && (last_angle > 340.0)"
                if (curr_end_deg < 20.0 && prev_angle_deg > 340.0) {
                    wrap_detected = true;
                    printf("DEBUG: Frame completed - angle wrap. Prev: %.1f°, Current: %.1f°, Points: %zu\n",
                        prev_angle_deg, curr_end_deg, accumulating_.size());
                }
            }

            // Force completion fallback
            if (!wrap_detected && accumulating_.size() > 360) {
                wrap_detected = true;
                printf("DEBUG: Frame completed - max points reached: %zu\n", accumulating_.size());
            }

            if (wrap_detected) {
                finalizeFrame(now_sec);
                have_start_ = true;
                frame_start_time_ = now_sec;
                accumulating_.clear();
                first_start_angle_ = start_a;
            }
        }

        last_end_angle_ = end_a;
        last_packet_time_ = now_sec;
    }

    void LD06Parser::finalizeFrame(double now_sec) {
        if (accumulating_.empty()) return;

        double duration = now_sec - frame_start_time_;
        size_t N = accumulating_.size();
        if (duration <= 0) duration = 0.1;

        // Assign relative timestamps to each beam
        for (size_t i = 0; i < N; i++) {
            accumulating_[i].relative_time = duration * (double)i / (double)N;
        }

        ScanFrame f;
        f.beams = std::move(accumulating_);
        accumulating_.clear();
        f.scan_start_time = frame_start_time_;
        f.scan_end_time = now_sec;
        f.revolution_hz = duration > 0 ? 1.0 / duration : 0.0;

        frames_.push_back(std::move(f));
        stats_.frames++;

        if (frames_.size() > 4) frames_.pop_front();
    }

    ScanFrame LD06Parser::takeFrame() {
        ScanFrame f;
        if (frames_.empty()) return f;
        f = std::move(frames_.front());
        frames_.pop_front();
        return f;
    }

    void LD06Parser::reset() {
        buffer_.clear();
        accumulating_.clear();
        frames_.clear();
        have_start_ = false;
        last_end_angle_ = 0;
    }

}
