#include "sigyn_lidar_v2/multi_lidar_node.hpp"
#include "sigyn_lidar_v2/lidar_base.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <limits>
#include <cmath>

namespace sigyn_lidar_v2 {

  SingleLidarNode::SingleLidarNode(const rclcpp::NodeOptions& options)
    : Node("single_lidar_node", options) {
    declare_parameters();
    load_configuration();
    all_ld06_ = (lidar_config_.device_type == "LD06");

    if (!setup_device()) {
      RCLCPP_ERROR(get_logger(), "Failed to setup LIDAR device");
      return;
    }
    start_data_acquisition();

    if (enable_motion_correction_) {
      motion_corrector_ = std::make_unique<MotionCorrector>(motion_config_);
      odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        motion_config_.odom_topic, 50,
        std::bind(&SingleLidarNode::odom_callback, this, std::placeholders::_1));
    }

    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(10),
      std::bind(&SingleLidarNode::diagnostics_callback, this));

    RCLCPP_INFO(get_logger(), "Single LIDAR node initialized (%s -> %s)", lidar_config_.device_path.c_str(), lidar_config_.topic_name.c_str());
  }

  SingleLidarNode::~SingleLidarNode() { stop_data_acquisition(); }

  void SingleLidarNode::declare_parameters() {
    declare_parameter<std::string>("device_path", "/dev/lidar_front_center");
    declare_parameter<std::string>("frame_id", "lidar_frame_top_lidar");
    declare_parameter<std::string>("topic_name", "/scan");
    declare_parameter<std::string>("device_type", "LD06");
    declare_parameter<int>("serial_baudrate", 230400);

    declare_parameter<double>("angle_min", 0.0);
    declare_parameter<double>("angle_max", 2.0 * M_PI);
    declare_parameter<double>("angle_increment", M_PI / 180.0 * 0.8);
    declare_parameter<double>("range_min", 0.02);
    declare_parameter<double>("range_max", 15.0);

    declare_parameter<bool>("enable_motion_correction", false);
    declare_parameter<double>("max_correction_time_s", 0.2);
    declare_parameter<std::string>("odom_topic", "/sigyn/odom");
  }

  void SingleLidarNode::load_configuration() {
    lidar_config_.device_path = get_parameter("device_path").as_string();
    lidar_config_.frame_id = get_parameter("frame_id").as_string();
    lidar_config_.topic_name = get_parameter("topic_name").as_string();
    lidar_config_.device_type = get_parameter("device_type").as_string();
    lidar_config_.serial_baudrate = get_parameter("serial_baudrate").as_int();

    angle_min_ = get_parameter("angle_min").as_double();
    angle_max_ = get_parameter("angle_max").as_double();
    angle_increment_ = get_parameter("angle_increment").as_double();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();

    enable_motion_correction_ = get_parameter("enable_motion_correction").as_bool();
    motion_config_.enable_correction = enable_motion_correction_;
    motion_config_.max_correction_time_s = get_parameter("max_correction_time_s").as_double();
    motion_config_.odom_topic = get_parameter("odom_topic").as_string();
  }

  bool SingleLidarNode::setup_device() {
    device_ = std::make_shared<LidarDevice>();
    device_->config = lidar_config_;

    device_->publisher = create_publisher<sensor_msgs::msg::LaserScan>(lidar_config_.topic_name, 10);

    device_->driver = LidarDriverFactory::create_driver(lidar_config_.device_type);
    if (!device_->driver) {
      RCLCPP_ERROR(get_logger(), "Failed to create driver for %s", lidar_config_.device_type.c_str());
      return false;
    }
    if (!device_->driver->configure(lidar_config_)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure driver");
      return false;
    }
    device_->driver->set_scan_callback([this](const LidarScan& scan) { this->on_scan_received(scan); });
    return true;
  }

  void SingleLidarNode::start_data_acquisition() {
    device_->serial_fd = open(lidar_config_.device_path.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
    if (device_->serial_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", lidar_config_.device_path.c_str());
      return;
    }
    configure_serial_port(device_->serial_fd, lidar_config_.serial_baudrate);
    if (!device_->driver->start()) {
      RCLCPP_ERROR(get_logger(), "Failed to start driver");
      close_serial_port(device_->serial_fd);
      return;
    }
    device_->should_stop = false;
    device_->read_thread = std::thread(&SingleLidarNode::device_read_thread, this);
  }

  void SingleLidarNode::stop_data_acquisition() {
    if (!device_) return;
    device_->should_stop = true;
    if (device_->driver) device_->driver->stop();
    if (device_->read_thread.joinable()) device_->read_thread.join();
    close_serial_port(device_->serial_fd);
  }

  void SingleLidarNode::configure_serial_port(int fd, int baudrate) {
    struct termios options; tcgetattr(fd, &options);
    speed_t speed; switch (baudrate) { case 230400: speed = B230400; break; case 115200: speed = B115200; break; case 57600: speed = B57600; break; case 38400: speed = B38400; break; case 19200: speed = B19200; break; case 9600: speed = B9600; break; default: speed = B230400; }
                                                  cfsetispeed(&options, speed); cfsetospeed(&options, speed);
                                                  options.c_cflag |= (CLOCAL | CREAD); options.c_cflag &= ~PARENB; options.c_cflag &= ~CSTOPB; options.c_cflag &= ~CSIZE; options.c_cflag |= CS8;
                                                  options.c_iflag &= ~(IXON | IXOFF | IXANY); options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); options.c_oflag &= ~OPOST;
                                                  tcsetattr(fd, TCSANOW, &options);
  }

  void SingleLidarNode::close_serial_port(int fd) { if (fd >= 0) close(fd); }

  void SingleLidarNode::device_read_thread() {
    constexpr size_t BUFFER_SIZE = 1024; uint8_t buffer[BUFFER_SIZE];
    while (!device_->should_stop) {
      ssize_t bytes_read = read(device_->serial_fd, buffer, BUFFER_SIZE);
      if (bytes_read > 0) {
        device_->driver->process_data(buffer, bytes_read);
      }
      else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(get_logger(), "Read error on %s: %s", lidar_config_.device_path.c_str(), strerror(errno));
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  void SingleLidarNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(motion_mutex_);
    if (motion_corrector_) {
      uint64_t timestamp_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
      motion_corrector_->add_odom(*msg, timestamp_ns);
    }
  }

  void SingleLidarNode::on_scan_received(const LidarScan& scan) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    stats_.scans_received++;
    LidarScan proc_scan = scan;
    if (enable_motion_correction_ && motion_corrector_) {
      std::lock_guard<std::mutex> mlock(motion_mutex_);
      proc_scan = motion_corrector_->correct_scan(scan);
    }
    auto msg = lidar_scan_to_ros_msg(proc_scan);
    device_->publisher->publish(msg);
    stats_.scans_published++;
  }

  uint64_t SingleLidarNode::get_current_timestamp_ns() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  }

  sensor_msgs::msg::LaserScan SingleLidarNode::lidar_scan_to_ros_msg(const LidarScan& scan) const {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp.sec = scan.scan_start_ns / 1000000000ULL;
    msg.header.stamp.nanosec = scan.scan_start_ns % 1000000000ULL;
    msg.header.frame_id = scan.frame_id;
    msg.angle_min = angle_min_;
    msg.angle_max = angle_max_;
    msg.angle_increment = angle_increment_;
    size_t beam_size = static_cast<size_t>(std::round((msg.angle_max - msg.angle_min) / msg.angle_increment));
    if (beam_size == 0) beam_size = 450;
    msg.range_min = range_min_;
    msg.range_max = range_max_;
    msg.ranges.assign(beam_size, std::numeric_limits<float>::infinity());
    msg.intensities.assign(beam_size, 0.0f);

    std::vector<LidarPoint> sorted = scan.points;
    std::sort(sorted.begin(), sorted.end(), [](const LidarPoint& a, const LidarPoint& b) { return a.angle_deg < b.angle_deg; });
    for (const auto& p : sorted) {
      double ang = p.angle_deg * M_PI / 180.0;
      while (ang < msg.angle_min) ang += 2 * M_PI;
      while (ang >= msg.angle_max) ang -= 2 * M_PI;
      int idx = static_cast<int>(std::round((ang - msg.angle_min) / msg.angle_increment));
      if (idx >= 0 && idx < static_cast<int>(beam_size)) {
        float r = p.distance_mm / 1000.0f;
        if (r >= msg.range_min && r <= msg.range_max) {
          if (!std::isfinite(msg.ranges[idx]) || r < msg.ranges[idx]) {
            msg.ranges[idx] = r;
            msg.intensities[idx] = p.intensity;
          }
        }
      }
    }
    msg.scan_time = 1.0; // unknown; can refine
    msg.time_increment = beam_size ? msg.scan_time / static_cast<double>(beam_size) : 0.0;

    if (all_ld06_) {
      sensor_msgs::msg::LaserScan tmp = msg;
      size_t n = tmp.ranges.size();
      for (size_t i = 0; i < n; ++i) {
        msg.ranges[i] = tmp.ranges[n - 1 - i];
        msg.intensities[i] = tmp.intensities[n - 1 - i];
      }
    }
    return msg;
  }

  void SingleLidarNode::diagnostics_callback() {
    RCLCPP_INFO(get_logger(), "LIDAR stats: received=%zu published=%zu", stats_.scans_received, stats_.scans_published);
    if (motion_corrector_) {
      RCLCPP_INFO(get_logger(), "%s", motion_corrector_->get_correction_stats().c_str());
    }
  }

} // namespace sigyn_lidar_v2
