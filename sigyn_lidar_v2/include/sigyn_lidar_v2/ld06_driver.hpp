#pragma once

#include "lidar_base.hpp"
#include <deque>
#include <cstdint>

namespace sigyn_lidar_v2 {

// LD06 protocol constants (from vendor implementation)
static constexpr uint8_t LD06_PKG_HEADER = 0x54;
static constexpr uint8_t LD06_DATA_PKG_INFO = 0x2C;
static constexpr uint8_t LD06_POINT_PER_PACK = 12;
static constexpr size_t LD06_PACKET_SIZE = 47; // Total packet size in bytes

// LD06 packet structures (from vendor, packed for exact byte layout)
#pragma pack(push, 1)
struct LD06Point {
  uint16_t distance;    // Distance in mm
  uint8_t intensity;    // Intensity [0-255] (vendor calls this "confidence")
};

struct LD06Packet {
  uint8_t header;                          // 0x54
  uint8_t ver_len;                         // 0x2C for data packets
  uint16_t speed;                          // Motor speed (used for timing)
  uint16_t start_angle;                    // Start angle in 0.01 degree units
  LD06Point point[LD06_POINT_PER_PACK];    // 12 measurement points
  uint16_t end_angle;                      // End angle in 0.01 degree units
  uint16_t timestamp;                      // Timestamp in ms (rolls over)
  uint8_t crc8;                           // CRC8 checksum
};
#pragma pack(pop)

// LD06 driver implementation
class LD06Driver : public LidarBase {
public:
  LD06Driver();
  ~LD06Driver() override;
  
  // LidarBase interface implementation
  bool configure(const LidarConfig& config) override;
  bool start() override;
  bool stop() override;
  bool is_connected() const override;
  
  bool process_data(const uint8_t* data, size_t length) override;
  bool has_complete_scan() const override;
  LidarScan get_scan() override;
  
  std::string get_device_info() const override;
  std::string get_status_string() const override;
  void set_scan_callback(ScanCallback callback) override;

private:
  // Protocol parsing state machine
  enum ParseState {
    WAIT_HEADER,
    WAIT_VER_LEN,
    COLLECT_DATA
  };
  
  // Parsing and data assembly
  bool parse_byte(uint8_t byte);
  bool process_packet(const LD06Packet& packet);
  bool is_scan_complete() const;
  void finalize_scan();
  
  // CRC calculation (from vendor implementation)
  uint8_t calculate_crc8(const uint8_t* data, size_t length) const;
  
  // Timestamp handling
  uint64_t get_current_timestamp_ns() const;
  
  // State variables
  ParseState parse_state_;
  std::vector<uint8_t> packet_buffer_;
  size_t bytes_needed_;
  
  // Data accumulation
  std::vector<LidarPoint> current_scan_points_;
  uint64_t scan_start_timestamp_;
  uint64_t last_packet_timestamp_;
  float last_angle_;
  uint16_t motor_speed_;
  
  // Scan completion tracking
  std::deque<LidarScan> complete_scans_;
  
  // Statistics
  struct Statistics {
    uint64_t packets_received;
    uint64_t packets_with_crc_error;
    uint64_t scans_completed;
    uint64_t bytes_processed;
    
    Statistics() : packets_received(0), packets_with_crc_error(0), 
                   scans_completed(0), bytes_processed(0) {}
  } stats_;
  
  // File descriptor for serial port
  int serial_fd_;
  bool is_running_;
  
  // CRC lookup table (from vendor implementation)
  static const uint8_t crc_table_[256];
};

} // namespace sigyn_lidar_v2
