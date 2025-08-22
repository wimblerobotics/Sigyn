#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sigyn_lidar/ld06_protocol.hpp"
#include "sigyn_lidar/ld06_motion_compensated.hpp"
#include "sigyn_lidar/deskew.hpp"
#include "sigyn_lidar/multi_lidar_fuser.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include <sstream>
#include <unordered_map>

namespace sigyn_lidar {

struct DeviceHandle {
  std::string port;
  std::string frame_id;
  int fd {-1};
  sigyn_lidar::LD06MotionCompensatedParser parser;  // Fully qualified new hybrid parser
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;       // /scan/<name>
};

class SigynLidarNode : public rclcpp::Node {
public:
  SigynLidarNode(): Node("sigyn_lidar") {
    declare_parameter<std::vector<std::string>>("devices", {"/dev/ttyUSB0"});
    declare_parameter<std::vector<std::string>>("frames", {"lidar_frame"});
    declare_parameter<std::vector<std::string>>("segments", std::vector<std::string>{});
    declare_parameter<std::string>("fused_frame", "lidar_frame_fused");
    declare_parameter<bool>("deskew", false);
    declare_parameter<bool>("skip_crc", false);

    get_parameter("devices", device_ports_);
    get_parameter("frames", frames_);
    {
      std::vector<std::string> seg_default;
      segments_raw_ = get_parameter("segments").as_string_array();
    }
    get_parameter("fused_frame", fused_frame_id_);
    get_parameter("deskew", deskew_enabled_);
    get_parameter("skip_crc", skip_crc_);

    if(frames_.size() != device_ports_.size()) {
      RCLCPP_WARN(get_logger(), "frames count (%zu) != devices count (%zu); using first frame for all", frames_.size(), device_ports_.size());
    }

    // Parse segments: format "index:start_deg:end_deg"
    for(const auto& s : segments_raw_) {
      int idx; double a0,a1; char c1,c2; std::stringstream ss(s); if(!(ss>>idx>>c1>>a0>>c2>>a1) || c1!=':'||c2!=':') { RCLCPP_WARN(get_logger(), "Bad segment spec %s", s.c_str()); continue; }
      if(idx < 0 || (size_t)idx >= device_ports_.size()) { RCLCPP_WARN(get_logger(), "Segment index out of range %s", s.c_str()); continue; }
      AngleSegment seg{a0*M_PI/180.0, a1*M_PI/180.0};
      pass_segments_[idx].push_back(seg);
    }

    fused_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    raw_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/raw_scan", 10);

    for(size_t i=0;i<device_ports_.size();++i) {
      auto dh = std::make_shared<DeviceHandle>();
      dh->port = device_ports_[i];
      dh->frame_id = (i<frames_.size()? frames_[i] : frames_.empty()? std::string("lidar_frame") : frames_[0]);
      dh->fd = openSerial(dh->port.c_str());
      if(dh->fd < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s", dh->port.c_str());
      }
      // Configure parser with parameters
      dh->parser.setSkipCRC(skip_crc_);
      dh->parser.setMotionCompensation(deskew_enabled_);
      
      std::string base_name = baseName(dh->port);
      std::string topic = std::string("/scan/") + base_name;
      dh->pub = create_publisher<sensor_msgs::msg::LaserScan>(topic, 10);
      devices_.push_back(dh);
      RCLCPP_INFO(get_logger(), "Device %s -> topic %s frame %s", dh->port.c_str(), topic.c_str(), dh->frame_id.c_str());
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&SigynLidarNode::pollLoop, this));
  }
private:
  static std::string baseName(const std::string& path) {
    auto pos = path.find_last_of('/');
    return pos==std::string::npos? path : path.substr(pos+1);
  }

  int openSerial(const char* dev) {
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(fd < 0) return -1;
    termios tio{}; tcgetattr(fd, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, B230400);
    cfsetospeed(&tio, B230400);
    tio.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &tio);
    return fd;
  }

  void pollLoop() {
    double now_sec = this->now().seconds();
    bool any_frame=false;
    std::vector<sensor_msgs::msg::LaserScan> per_lidar_msgs;
    for(size_t idx=0; idx<devices_.size(); ++idx) {
      auto & d = devices_[idx];
      if(d->fd < 0) continue;
      uint8_t buf[512];
      int n = ::read(d->fd, buf, sizeof(buf));
      if(n > 0) {
        d->parser.parseBytes(buf, n, now_sec);
      }
      while(d->parser.hasCompleteFrame()) {
        auto frame = d->parser.takeFrame();
        if(!frame.beams.empty()) {  // Updated validity check
          any_frame=true;
          // raw frame publish (first device only for now)
          if(idx==0) raw_pub_->publish(buildMsgFromFrame(frame, d->frame_id));
          // apply segment filtering per device before fused
          applySegments(frame, pass_segments_[idx]);
          auto msg = buildMsgFromFrame(frame, d->frame_id);
          d->pub->publish(msg);
          per_lidar_msgs.push_back(msg);
        }
      }
    }
    if(!per_lidar_msgs.empty()) {
      auto fused = MultiLidarFuser::fuse(per_lidar_msgs, {.fused_frame = fused_frame_id_});
      fused.header.stamp = this->now();
      fused_pub_->publish(fused);
    }
    if(!any_frame) {
      for(size_t i=0;i<devices_.size();++i) {
        auto st = devices_[i]->parser.getStats();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, 
          "[%zu] bytes=%lu pkts=%lu frames=%lu motion_comp=%lu motion_avail=%s", 
          i, st.bytes, st.packets, st.frames, st.motion_compensated_frames,
          st.motion_data_available ? "yes" : "no");
      }
    }
  }

  void applySegments(sigyn_lidar::EnhancedScanFrame & frame, const std::vector<AngleSegment>& segs) {
    if(segs.empty()) return; // keep all
    std::vector<sigyn_lidar::EnhancedBeam> kept; kept.reserve(frame.beams.size());
    for(const auto& b : frame.beams) {
      for(const auto& seg : segs) {
        float angle = b.is_deskewed ? b.deskewed_angle_rad : b.angle_rad;
        if(angle >= seg.start_rad && angle <= seg.end_rad) {
          kept.push_back(b);
          break;
        }
      }
    }
    frame.beams.swap(kept);
  }

  sensor_msgs::msg::LaserScan buildMsgFromFrame(const sigyn_lidar::EnhancedScanFrame & frame, const std::string& frame_id) {
    // Use the utility function from the new implementation
    auto msg = sigyn_lidar::buildMsgFromFrame(frame, frame_id);
    msg.header.stamp = this->now();
    msg.scan_time = frame.scan_end_time - frame.scan_start_time;
    msg.time_increment = !frame.beams.empty() ? msg.scan_time / frame.beams.size() : 0.0;
    return msg;
  }

  std::vector<std::string> device_ports_;
  std::vector<std::string> frames_;
  std::vector<std::string> segments_raw_;
  std::string fused_frame_id_;
  bool deskew_enabled_ {false};
  bool skip_crc_{false};
  std::vector<std::shared_ptr<DeviceHandle>> devices_;
  std::unordered_map<int, std::vector<AngleSegment>> pass_segments_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sigyn_lidar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sigyn_lidar::SigynLidarNode>());
  rclcpp::shutdown();
  return 0;
}
