#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sigyn_lidar/lidar_parser_interface.hpp"
#include "sigyn_lidar/ld06_parser.hpp"
#include "sigyn_lidar/multi_lidar_fuser.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <cmath>

namespace sigyn_lidar {

struct DeviceHandle {
  std::string port;
  std::string frame_id;
  std::string parser_type;
  std::unique_ptr<LidarParserInterface> parser;
  std::vector<AngleSegment> segments;
  int fd {-1};
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
};

class ModularSigynLidarNode : public rclcpp::Node {
public:
  ModularSigynLidarNode(): Node("modular_sigyn_lidar") {
    declare_parameter<std::vector<std::string>>("devices", {"/dev/ttyUSB0"});
    declare_parameter<std::vector<std::string>>("frames", {"lidar_frame"});
    declare_parameter<std::vector<std::string>>("parsers", {"ld06"});
    declare_parameter<std::vector<std::string>>("topics", {"scan_lidar"});
    
    // Declare device-specific segments parameters (optional)
    declare_parameter<std::vector<std::string>>("device0_segments", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("device1_segments", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("device2_segments", std::vector<std::string>{});
    declare_parameter<std::vector<std::string>>("device3_segments", std::vector<std::string>{});
    
    declare_parameter<std::string>("fused_frame", "lidar_frame_fused");
    declare_parameter<std::string>("fused_topic", "scan_fused");
    declare_parameter<bool>("deskew", false);
    declare_parameter<bool>("skip_crc", false);
    declare_parameter<double>("output_frequency", 10.0);  // Hz for output rate

    get_parameter("devices", device_ports_);
    get_parameter("frames", frames_);
    get_parameter("parsers", parser_types_);
    get_parameter("topics", topics_);
    get_parameter("fused_frame", fused_frame_id_);
    get_parameter("fused_topic", fused_topic_);
    get_parameter("deskew", deskew_enabled_);
    get_parameter("skip_crc", skip_crc_);
    get_parameter("output_frequency", output_frequency_);

    validateConfiguration();
    setupDevices();
    
    if (!devices_.empty()) {
      setupPublishers();
      setupTimer();
    } else {
      RCLCPP_ERROR(get_logger(), "No devices successfully configured!");
    }
  }

  ~ModularSigynLidarNode() {
    for (auto& device : devices_) {
      if (device->fd >= 0) {
        close(device->fd);
      }
    }
  }

private:
  void validateConfiguration() {
    if (frames_.size() != device_ports_.size()) {
      RCLCPP_WARN(get_logger(), "Frame count (%zu) != device count (%zu); using defaults for missing",
                  frames_.size(), device_ports_.size());
    }
    
    if (parser_types_.size() != device_ports_.size()) {
      RCLCPP_WARN(get_logger(), "Parser type count (%zu) != device count (%zu); using defaults for missing",
                  parser_types_.size(), device_ports_.size());
    }
    
    if (topics_.size() != device_ports_.size()) {
      RCLCPP_WARN(get_logger(), "Topic count (%zu) != device count (%zu); using defaults for missing",
                  topics_.size(), device_ports_.size());
    }
  }

  void setupDevices() {
    for (size_t i = 0; i < device_ports_.size(); ++i) {
      auto device = std::make_shared<DeviceHandle>();
      device->port = device_ports_[i];
      device->frame_id = (i < frames_.size()) ? frames_[i] : frames_.empty() ? "lidar_frame" : frames_[0];
      device->parser_type = (i < parser_types_.size()) ? parser_types_[i] : parser_types_.empty() ? "ld06" : parser_types_[0];
      
      // Parse segments for this device using device-specific parameter
      std::string param_name = "device" + std::to_string(i) + "_segments";
      std::vector<std::string> device_segments;
      if (has_parameter(param_name)) {
        get_parameter(param_name, device_segments);
        for (const auto& seg_str : device_segments) {
          double start_deg, end_deg;
          char delimiter;
          std::stringstream ss(seg_str);
          if (ss >> start_deg >> delimiter >> end_deg && delimiter == ':') {
            AngleSegment seg{start_deg * M_PI / 180.0, end_deg * M_PI / 180.0};
            device->segments.push_back(seg);
            RCLCPP_INFO(get_logger(), "Device %zu segment: %.1f° to %.1f°", i, start_deg, end_deg);
          } else {
            RCLCPP_WARN(get_logger(), "Invalid segment format: %s", seg_str.c_str());
          }
        }
      } else {
        // Default to no segments if not configured
        RCLCPP_INFO(get_logger(), "Device %zu: no segments configured (will use all angles)", i);
      }
      
      // Create parser
      device->parser = LidarParserFactory::createParser(device->parser_type);
      if (!device->parser) {
        RCLCPP_ERROR(get_logger(), "Unknown parser type: %s", device->parser_type.c_str());
        continue;
      }
      
      // Configure parser
      device->parser->setSkipCRC(skip_crc_);
      device->parser->setMotionCompensation(deskew_enabled_);
      
      // Open serial port
      device->fd = open(device->port.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
      if (device->fd < 0) {
        RCLCPP_WARN(get_logger(), "Failed to open %s: %s", device->port.c_str(), strerror(errno));
        continue;
      }
      
      // Configure serial port for lidar
      struct termios options;
      tcgetattr(device->fd, &options);
      cfsetispeed(&options, B230400);
      cfsetospeed(&options, B230400);
      options.c_cflag |= (CLOCAL | CREAD);
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      options.c_cflag &= ~CSIZE;
      options.c_cflag |= CS8;
      options.c_iflag &= ~(IXON | IXOFF | IXANY);
      options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
      options.c_oflag &= ~OPOST;
      tcsetattr(device->fd, TCSANOW, &options);
      
      devices_.push_back(device);
      RCLCPP_INFO(get_logger(), "Configured device %zu: %s (%s parser)", i, device->port.c_str(), device->parser_type.c_str());
    }
  }

  void setupPublishers() {
    for (size_t i = 0; i < devices_.size(); ++i) {
      auto& device = devices_[i];
      
      // Use configured topic name if available, otherwise generate from device name
      std::string topic;
      if (i < topics_.size()) {
        topic = topics_[i];
      } else {
        // Fallback: extract device name from port path (remove /dev/ prefix)
        std::string device_name = device->port;
        if (device_name.substr(0, 5) == "/dev/") {
          device_name = device_name.substr(5);  // Remove "/dev/" prefix
        }
        topic = "scan_" + device_name;
      }
      
      device->publisher = create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);
      RCLCPP_INFO(get_logger(), "Publishing %s on topic %s", device->frame_id.c_str(), topic.c_str());
    }
    
    // Setup fused publisher
    fused_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(fused_topic_, 10);
  }

  void setupTimer() {
    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / output_frequency_));
    timer_ = create_wall_timer(period, [this]() { pollLoop(); });
    RCLCPP_INFO(get_logger(), "Started polling at %.1f Hz", output_frequency_);
  }

  void pollLoop() {
    constexpr size_t BUFFER_SIZE = 1024;
    uint8_t buf[BUFFER_SIZE];
    
    double now_sec = now().seconds();
    
    std::vector<LidarScanFrame> frames_to_fuse;
    
    for (auto& device : devices_) {
      if (device->fd < 0) continue;
      
      // Read available data
      ssize_t n = read(device->fd, buf, BUFFER_SIZE);
      if (n > 0) {
        device->parser->parseBytes(buf, n, now_sec);
      }
      
      // Process completed frames
      while (device->parser->hasCompleteFrame()) {
        LidarScanFrame frame = device->parser->takeFrame();
        if (frame.beams.empty()) continue;
        
        // Apply segments filter
        applySegments(frame, device->segments);
        
        if (!device->segments.empty()) {
          // Publish filtered frame
          auto raw_msg = buildMsgFromFrame(frame, device->frame_id);
          device->publisher->publish(raw_msg);
        }
        
        // Save for fusion (include all frames, filtered or not)
        applySegments(frame, device->segments);
        frames_to_fuse.push_back(frame);
      }
    }
    
    // Publish fused scan
    if (!frames_to_fuse.empty()) {
      // Convert LidarScanFrames to LaserScan messages
      std::vector<sensor_msgs::msg::LaserScan> scans_to_fuse;
      for (const auto& frame : frames_to_fuse) {
        auto scan = buildMsgFromFrame(frame, fused_frame_id_);
        scans_to_fuse.push_back(scan);
      }
      
      // Use fuser to combine scans
      FusedScanConfig fuse_config;
      fuse_config.fused_frame = fused_frame_id_;
      auto fused = MultiLidarFuser::fuse(scans_to_fuse, fuse_config);
      fused.header.stamp = this->now();
      fused_publisher_->publish(fused);
    }
    
    // Log stats periodically
    if (++stats_counter_ % static_cast<int>(output_frequency_ * 5) == 0) {
      for (size_t i = 0; i < devices_.size(); ++i) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                        "Device %zu (%s): %s", 
                        i, devices_[i]->parser_type.c_str(), devices_[i]->parser->getStatsString().c_str());
      }
    }
  }

  void applySegments(LidarScanFrame& frame, const std::vector<AngleSegment>& segments) {
    if (segments.empty()) return;  // No filtering
    
    std::vector<LidarBeam> kept;
    kept.reserve(frame.beams.size());
    
    for (const auto& beam : frame.beams) {
      for (const auto& seg : segments) {
        if (beam.angle_rad >= seg.start_rad && beam.angle_rad <= seg.end_rad) {
          kept.push_back(beam);
          break;
        }
      }
    }
    
    frame.beams = std::move(kept);
  }

  sensor_msgs::msg::LaserScan buildMsgFromFrame(const LidarScanFrame& frame, const std::string& frame_id) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id;
    
    if (frame.beams.empty()) return msg;
    
    // Find angle range
    auto [min_angle, max_angle] = std::minmax_element(
      frame.beams.begin(), frame.beams.end(),
      [](const auto& a, const auto& b) { return a.angle_rad < b.angle_rad; });
    
    msg.angle_min = min_angle->angle_rad;
    msg.angle_max = max_angle->angle_rad;
    msg.angle_increment = (frame.beams.size() > 1) ? 
      (msg.angle_max - msg.angle_min) / (frame.beams.size() - 1) : 0.01;
    
    msg.scan_time = frame.scan_end_time - frame.scan_start_time;
    msg.range_min = 0.02;
    msg.range_max = 8.0;
    
    msg.time_increment = !frame.beams.empty() ? msg.scan_time / frame.beams.size() : 0.0;
    
    // Convert beams to ranges
    for (const auto& beam : frame.beams) {
      float range = beam.range_m;
      
      // Filter out .inf values by checking confidence and range limits
      if (beam.confidence < 50 || range < msg.range_min || range > msg.range_max) {
        range = std::numeric_limits<float>::quiet_NaN();
      }
      
      msg.ranges.push_back(range);
      msg.intensities.push_back(0.0f);  // Default intensity since not available in LidarBeam
    }
    
    return msg;
  }

  // Configuration
  std::vector<std::string> device_ports_;
  std::vector<std::string> frames_;
  std::vector<std::string> parser_types_;
  std::vector<std::string> topics_;
  std::string fused_frame_id_;
  std::string fused_topic_;
  bool deskew_enabled_ {false};
  bool skip_crc_ {false};
  double output_frequency_ {10.0};

  // Runtime state
  std::vector<std::shared_ptr<DeviceHandle>> devices_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  int stats_counter_ {0};
};

} // namespace sigyn_lidar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sigyn_lidar::ModularSigynLidarNode>());
  rclcpp::shutdown();
  return 0;
}
