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
#include <algorithm>

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
  ModularSigynLidarNode();
  ~ModularSigynLidarNode();
private:
  void validateConfiguration();
  void setupDevices();
  void setupPublishers();
  void setupTimers();
  void readLoop();
  void applySegments(LidarScanFrame& frame, const std::vector<AngleSegment>& segments);
  sensor_msgs::msg::LaserScan buildMsgFromFrame(const LidarScanFrame& frame, const std::string& frame_id);
  // Configuration
  std::vector<std::string> device_ports_; std::vector<std::string> frames_; std::vector<std::string> parser_types_; std::vector<std::string> topics_;
  std::string fused_frame_id_; std::string fused_topic_; bool deskew_enabled_{false}; bool skip_crc_{false}; double output_frequency_{10.0}; double read_frequency_{1000.0};
  // Runtime
  std::vector<std::shared_ptr<DeviceHandle>> devices_; rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_publisher_; rclcpp::TimerBase::SharedPtr read_timer_; int stats_counter_{0};
};

ModularSigynLidarNode::ModularSigynLidarNode(): Node("modular_sigyn_lidar") {
  declare_parameter<std::vector<std::string>>("devices", {"/dev/ttyUSB0"});
  declare_parameter<std::vector<std::string>>("frames", {"lidar_frame"});
  declare_parameter<std::vector<std::string>>("parsers", {"ld06"});
  declare_parameter<std::vector<std::string>>("topics", {"scan_lidar"});
  declare_parameter<std::vector<std::string>>("device0_segments", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("device1_segments", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("device2_segments", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("device3_segments", std::vector<std::string>{});
  declare_parameter<std::string>("fused_frame", "lidar_frame_fused");
  declare_parameter<std::string>("fused_topic", "scan_fused");
  declare_parameter<bool>("deskew", false);
  declare_parameter<bool>("skip_crc", false);
  declare_parameter<double>("output_frequency", 10.0); // publish (frames arrive naturally ~10Hz)
  declare_parameter<double>("read_frequency", 1000.0);  // high rate serial polling (vendor style continuous read)

  get_parameter("devices", device_ports_);
  get_parameter("frames", frames_);
  get_parameter("parsers", parser_types_);
  get_parameter("topics", topics_);
  get_parameter("fused_frame", fused_frame_id_);
  get_parameter("fused_topic", fused_topic_);
  get_parameter("deskew", deskew_enabled_);
  get_parameter("skip_crc", skip_crc_);
  get_parameter("output_frequency", output_frequency_);
  get_parameter("read_frequency", read_frequency_);

  validateConfiguration();
  setupDevices();
  if (!devices_.empty()) { setupPublishers(); setupTimers(); } else { RCLCPP_ERROR(get_logger(), "No devices successfully configured!"); }
}

ModularSigynLidarNode::~ModularSigynLidarNode() {
  for (auto& device : devices_) if (device->fd >= 0) close(device->fd);
}

void ModularSigynLidarNode::validateConfiguration() {
  if (frames_.size() != device_ports_.size()) {
    RCLCPP_WARN(get_logger(), "Frame count (%zu) != device count (%zu); using defaults for missing", frames_.size(), device_ports_.size());
  }
  if (parser_types_.size() != device_ports_.size()) {
    RCLCPP_WARN(get_logger(), "Parser type count (%zu) != device count (%zu); using defaults for missing", parser_types_.size(), device_ports_.size());
  }
  if (topics_.size() != device_ports_.size()) {
    RCLCPP_WARN(get_logger(), "Topic count (%zu) != device count (%zu); using defaults for missing", topics_.size(), device_ports_.size());
  }
}

void ModularSigynLidarNode::setupDevices() {
  for (size_t i = 0; i < device_ports_.size(); ++i) {
    auto device = std::make_shared<DeviceHandle>();
    device->port = device_ports_[i];
    device->frame_id = (i < frames_.size()) ? frames_[i] : frames_.empty() ? "lidar_frame" : frames_[0];
    device->parser_type = (i < parser_types_.size()) ? parser_types_[i] : parser_types_.empty() ? "ld06" : parser_types_[0];

    std::string param_name = "device" + std::to_string(i) + "_segments";
    std::vector<std::string> device_segments;
    if (has_parameter(param_name)) {
      get_parameter(param_name, device_segments);
      for (const auto& seg_str : device_segments) {
        double start_deg, end_deg; char delimiter; std::stringstream ss(seg_str);
        if (ss >> start_deg >> delimiter >> end_deg && delimiter == ':') {
          AngleSegment seg{start_deg * M_PI / 180.0, end_deg * M_PI / 180.0};
          device->segments.push_back(seg);
          RCLCPP_INFO(get_logger(), "Device %zu segment: %.1f° to %.1f°", i, start_deg, end_deg);
        } else {
          RCLCPP_WARN(get_logger(), "Invalid segment format: %s", seg_str.c_str());
        }
      }
    } else {
      RCLCPP_INFO(get_logger(), "Device %zu: no segments configured (will use all angles)", i);
    }

    device->parser = LidarParserFactory::createParser(device->parser_type);
    if (!device->parser) { RCLCPP_ERROR(get_logger(), "Unknown parser type: %s", device->parser_type.c_str()); continue; }
    device->parser->setSkipCRC(skip_crc_);
    device->parser->setMotionCompensation(deskew_enabled_);

    device->fd = open(device->port.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
    if (device->fd < 0) { RCLCPP_WARN(get_logger(), "Failed to open %s: %s", device->port.c_str(), strerror(errno)); continue; }

    struct termios options; tcgetattr(device->fd, &options);
    cfsetispeed(&options, B230400); cfsetospeed(&options, B230400);
    options.c_cflag |= (CLOCAL | CREAD); options.c_cflag &= ~PARENB; options.c_cflag &= ~CSTOPB; options.c_cflag &= ~CSIZE; options.c_cflag |= CS8;
    options.c_iflag &= ~(IXON | IXOFF | IXANY); options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); options.c_oflag &= ~OPOST; tcsetattr(device->fd, TCSANOW, &options);

    devices_.push_back(device);
    RCLCPP_INFO(get_logger(), "Configured device %zu: %s (%s parser)", i, device->port.c_str(), device->parser_type.c_str());
  }
}

void ModularSigynLidarNode::setupPublishers() {
  for (size_t i = 0; i < devices_.size(); ++i) {
    auto& device = devices_[i];
    std::string topic;
    if (i < topics_.size()) topic = topics_[i]; else {
      std::string device_name = device->port; if (device_name.rfind("/dev/", 0) == 0) device_name = device_name.substr(5); topic = "scan_" + device_name;
    }
    device->publisher = create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);
    RCLCPP_INFO(get_logger(), "Publishing %s on topic %s", device->frame_id.c_str(), topic.c_str());
  }
  fused_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(fused_topic_, 10);
}

void ModularSigynLidarNode::setupTimers() {
  // High-rate read loop (continuous acquisition similar to vendor reference code)
  auto read_period = std::chrono::microseconds(static_cast<int64_t>(1e6 / std::max(50.0, read_frequency_))); // clamp min 50 Hz
  read_timer_ = create_wall_timer(read_period, [this]() { readLoop(); });
  RCLCPP_INFO(get_logger(), "Started read loop at %.1f Hz", read_frequency_);
}

void ModularSigynLidarNode::readLoop() {
  constexpr size_t BUFFER_SIZE = 1024; uint8_t buf[BUFFER_SIZE];
  double now_sec = now().seconds();
  std::vector<LidarScanFrame> frames_to_fuse;

  for (auto& device : devices_) {
    if (device->fd < 0) continue;
    ssize_t n = read(device->fd, buf, BUFFER_SIZE);
    if (n > 0) {
      device->parser->parseBytes(buf, n, now_sec); // parser internally accumulates
    }
    while (device->parser->hasCompleteFrame()) {
      LidarScanFrame frame = device->parser->takeFrame();
      if (frame.beams.empty()) continue;
      applySegments(frame, device->segments);
      auto raw_msg = buildMsgFromFrame(frame, device->frame_id);
      device->publisher->publish(raw_msg);
      frames_to_fuse.push_back(frame);
    }
  }

  if (!frames_to_fuse.empty()) {
    std::vector<sensor_msgs::msg::LaserScan> scans_to_fuse; scans_to_fuse.reserve(frames_to_fuse.size());
    for (const auto& frame : frames_to_fuse) scans_to_fuse.push_back(buildMsgFromFrame(frame, fused_frame_id_));
    FusedScanConfig fuse_config; fuse_config.fused_frame = fused_frame_id_;
    auto fused = MultiLidarFuser::fuse(scans_to_fuse, fuse_config); fused.header.stamp = this->now(); fused_publisher_->publish(fused);
  }

  if (++stats_counter_ % 500 == 0) { // roughly every 0.5s at 1000 Hz default
    for (size_t i = 0; i < devices_.size(); ++i) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Device %zu (%s): %s", i, devices_[i]->parser_type.c_str(), devices_[i]->parser->getStatsString().c_str());
    }
  }
}

void ModularSigynLidarNode::applySegments(LidarScanFrame& frame, const std::vector<AngleSegment>& segments) {
  if (segments.empty()) return;
  std::vector<LidarBeam> kept; kept.reserve(frame.beams.size());
  for (const auto& beam : frame.beams) {
    for (const auto& seg : segments) {
      if (beam.angle_rad >= seg.start_rad && beam.angle_rad <= seg.end_rad) { kept.push_back(beam); break; }
    }
  }
  frame.beams = std::move(kept);
}

sensor_msgs::msg::LaserScan ModularSigynLidarNode::buildMsgFromFrame(const LidarScanFrame& frame, const std::string& frame_id) {
  sensor_msgs::msg::LaserScan msg; msg.header.stamp = this->now(); msg.header.frame_id = frame_id; if (frame.beams.empty()) return msg;
  // Fixed 450 bins (LD06: ~4500 points/sec at 10 Hz) matching vendor expectation.
  constexpr size_t BIN_COUNT = 450; // target rays
  msg.angle_min = 0.0; msg.angle_max = 2.0 * M_PI; msg.range_min = 0.02; msg.range_max = 15.0; // 2cm min per datasheet
  msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<double>(BIN_COUNT);
  msg.ranges.assign(BIN_COUNT, std::numeric_limits<float>::infinity());
  msg.intensities.assign(BIN_COUNT, 0.0f);

  // Place each beam into its bin (choose closest angle index). Keep nearest range if multiple map to same bin.
  for (const auto& b : frame.beams) {
    float ang = b.angle_rad; if (ang < 0.0f) ang += 2.0f * static_cast<float>(M_PI);
    if (ang >= 2.0f * static_cast<float>(M_PI)) ang -= 2.0f * static_cast<float>(M_PI);
    int idx = static_cast<int>(std::floor(ang / msg.angle_increment + 0.5)); // nearest
    if (idx < 0) idx = 0; if (idx >= static_cast<int>(BIN_COUNT)) idx = static_cast<int>(BIN_COUNT) - 1;
    float r = b.range_m; if (r <= 0.0f) r = std::numeric_limits<float>::quiet_NaN();
    float existing = msg.ranges[idx];
    bool take = false;
    if (std::isnan(r)) take = true; // prefer NaN over inf only if empty
    if (std::isfinite(r) && (!std::isfinite(existing) || r < existing)) take = true; // keep closest obstacle
    if (take) { msg.ranges[idx] = r; msg.intensities[idx] = b.confidence; }
  }

  msg.scan_time = frame.scan_end_time - frame.scan_start_time; if (msg.scan_time <= 0.0) msg.scan_time = 0.1; // fallback ~10Hz
  msg.time_increment = msg.scan_time / static_cast<double>(BIN_COUNT);
  return msg;
}

} // namespace sigyn_lidar

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<sigyn_lidar::ModularSigynLidarNode>()); rclcpp::shutdown(); return 0; }
