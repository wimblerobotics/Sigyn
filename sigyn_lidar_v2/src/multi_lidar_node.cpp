#include "sigyn_lidar_v2/multi_lidar_node.hpp"
#include "sigyn_lidar_v2/lidar_base.hpp"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <algorithm>
#include <limits>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace sigyn_lidar_v2 {

MultiLidarNode::MultiLidarNode(const rclcpp::NodeOptions& options)
  : Node("multi_lidar_node", options), fusion_frequency_hz_(10.0), enable_individual_topics_(true) {

  declare_parameters();
  load_configuration();

  if (!setup_devices()) {
    RCLCPP_ERROR(get_logger(), "Failed to setup LIDAR devices");
    return;
  }

  if (!setup_publishers_and_subscribers()) {
    RCLCPP_ERROR(get_logger(), "Failed to setup ROS publishers and subscribers");
    return;
  }

  start_data_acquisition();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(get_logger(), "Multi-LIDAR node initialized successfully");
}

MultiLidarNode::~MultiLidarNode() {
  stop_data_acquisition();
}

void MultiLidarNode::declare_parameters() {
  // Device configuration
  declare_parameter<std::vector<std::string>>("device_paths", std::vector<std::string>{"/dev/lidar_front_center"});
  declare_parameter<std::vector<std::string>>("frame_ids", std::vector<std::string>{"lidar_frame_top_lidar"});
  declare_parameter<std::vector<std::string>>("topic_names", std::vector<std::string>{"/scan/lidar_front_center"});
  declare_parameter<std::vector<std::string>>("device_types", std::vector<std::string>{"LD06"});
  declare_parameter<std::vector<int>>("serial_baudrates", std::vector<int>{230400});

  // Fusion configuration
  declare_parameter<std::string>("fused_frame_id", "base_link");
  declare_parameter<std::string>("fused_topic_name", "/scan");
  declare_parameter<double>("fusion_frequency_hz", 10.0);
  declare_parameter<bool>("enable_individual_topics", true);
  declare_parameter<double>("angle_min", 0.0);
  declare_parameter<double>("angle_max", 2.0 * M_PI);
  declare_parameter<double>("angle_increment", M_PI / 180.0 * 0.8); // ~0.8 degrees
  declare_parameter<double>("range_min", 0.02);
  declare_parameter<double>("range_max", 15.0);

  // Motion correction configuration
  declare_parameter<bool>("enable_motion_correction", false);
  declare_parameter<double>("max_correction_time_s", 0.2);
  declare_parameter<bool>("use_cmd_vel", true);
  declare_parameter<bool>("use_wheel_odom", true);
  declare_parameter<bool>("use_imu", false);
  declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  declare_parameter<std::string>("wheel_odom_topic", "/sigyn/wheel_odom");
  declare_parameter<std::string>("imu_topic", "/imu/data");
}

void MultiLidarNode::load_configuration() {
  // Load device configurations
  auto device_paths = get_parameter("device_paths").as_string_array();
  auto frame_ids = get_parameter("frame_ids").as_string_array();
  auto topic_names = get_parameter("topic_names").as_string_array();
  auto device_types = get_parameter("device_types").as_string_array();
  auto baudrates = get_parameter("serial_baudrates").as_integer_array();

  size_t num_devices = device_paths.size();
  lidar_configs_.resize(num_devices);

  for (size_t i = 0; i < num_devices; ++i) {
    lidar_configs_[i].device_path = device_paths[i];
    lidar_configs_[i].frame_id = (i < frame_ids.size()) ? frame_ids[i] : ("lidar_frame_" + std::to_string(i));
    lidar_configs_[i].topic_name = (i < topic_names.size()) ? topic_names[i] : ("/lidar_" + std::to_string(i));
    lidar_configs_[i].device_type = (i < device_types.size()) ? device_types[i] : "LD06";
    lidar_configs_[i].serial_baudrate = (i < baudrates.size()) ? baudrates[i] : 230400;
    lidar_configs_[i].enable_motion_correction = get_parameter("enable_motion_correction").as_bool();
  }

  // Load fusion configuration
  fusion_config_.fused_frame_id = get_parameter("fused_frame_id").as_string();
  fusion_config_.fused_topic_name = get_parameter("fused_topic_name").as_string();
  fusion_config_.angle_min = get_parameter("angle_min").as_double();
  fusion_config_.angle_max = get_parameter("angle_max").as_double();
  fusion_config_.angle_increment = get_parameter("angle_increment").as_double();
  fusion_config_.range_min = get_parameter("range_min").as_double();
  fusion_config_.range_max = get_parameter("range_max").as_double();
  fusion_config_.expected_scan_count = num_devices;

  fusion_frequency_hz_ = get_parameter("fusion_frequency_hz").as_double();
  enable_individual_topics_ = get_parameter("enable_individual_topics").as_bool();

  // Load motion correction configuration
  motion_config_.enable_correction = get_parameter("enable_motion_correction").as_bool();
  motion_config_.max_correction_time_s = get_parameter("max_correction_time_s").as_double();
  motion_config_.use_cmd_vel = get_parameter("use_cmd_vel").as_bool();
  motion_config_.use_wheel_odom = get_parameter("use_wheel_odom").as_bool();
  motion_config_.use_imu = get_parameter("use_imu").as_bool();
  motion_config_.cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
  motion_config_.wheel_odom_topic = get_parameter("wheel_odom_topic").as_string();
  motion_config_.imu_topic = get_parameter("imu_topic").as_string();
}

bool MultiLidarNode::setup_devices() {
  devices_.clear();
  devices_.reserve(lidar_configs_.size());

  for (size_t i = 0; i < lidar_configs_.size(); ++i) {
    const auto& config = lidar_configs_[i];

    auto device = std::make_shared<LidarDevice>();
    device->config = config;

    // Create driver
    device->driver = LidarDriverFactory::create_driver(config.device_type);
    if (!device->driver) {
      RCLCPP_ERROR(get_logger(), "Failed to create driver for device type: %s", config.device_type.c_str());
      continue;
    }

    // Configure driver
    if (!device->driver->configure(config)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure driver for device: %s", config.device_path.c_str());
      continue;
    }

    // Set up scan callback
    device->driver->set_scan_callback([this, device_name = config.device_path](const LidarScan& scan) {
      this->on_scan_received(scan, device_name);
      });

    devices_.push_back(device);

    RCLCPP_INFO(get_logger(), "Configured device %zu: %s (%s) -> %s",
      i, config.device_path.c_str(), config.device_type.c_str(), config.topic_name.c_str());
  }

  if (devices_.empty()) {
    RCLCPP_ERROR(get_logger(), "No LIDAR devices were successfully configured");
    return false;
  }

  return true;
}

bool MultiLidarNode::setup_publishers_and_subscribers() {
  // Create individual LIDAR publishers
  if (enable_individual_topics_) {
    for (auto& device : devices_) {
      device->publisher = create_publisher<sensor_msgs::msg::LaserScan>(
        device->config.topic_name, 10);
    }
  }

  // Create fused scan publisher
  fused_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
    fusion_config_.fused_topic_name, 10);

  // Create motion subscribers if motion correction is enabled
  if (motion_config_.enable_correction) {
    if (motion_config_.use_cmd_vel) {
      cmd_vel_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        motion_config_.cmd_vel_topic, 10,
        std::bind(&MultiLidarNode::cmd_vel_callback, this, std::placeholders::_1));
    }

    if (motion_config_.use_wheel_odom) {
      wheel_odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        motion_config_.wheel_odom_topic, 10,
        std::bind(&MultiLidarNode::wheel_odom_callback, this, std::placeholders::_1));
    }

    if (motion_config_.use_imu) {
      imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>(
        motion_config_.imu_topic, 10,
        std::bind(&MultiLidarNode::imu_callback, this, std::placeholders::_1));
    }
  }

  // Create processing components
  fusion_processor_ = std::make_unique<LidarFusion>(fusion_config_);
  motion_corrector_ = std::make_unique<MotionCorrector>(motion_config_);

  // Create fusion timer
  auto fusion_period = std::chrono::milliseconds(static_cast<int>(1000.0 / fusion_frequency_hz_));
  fusion_timer_ = create_wall_timer(fusion_period,
    std::bind(&MultiLidarNode::fusion_timer_callback, this));

  // Create diagnostics timer
  auto diagnostics_period = std::chrono::seconds(5);
  diagnostics_timer_ = create_wall_timer(diagnostics_period,
    std::bind(&MultiLidarNode::diagnostics_callback, this));

  return true;
}

void MultiLidarNode::start_data_acquisition() {
  for (auto& device : devices_) {
    // Open serial port
    device->serial_fd = open(device->config.device_path.c_str(), O_RDONLY | O_NOCTTY | O_NDELAY);
    if (device->serial_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", device->config.device_path.c_str());
      continue;
    }

    // Configure serial port
    configure_serial_port(device->serial_fd, device->config.serial_baudrate);

    // Start driver
    if (!device->driver->start()) {
      RCLCPP_ERROR(get_logger(), "Failed to start driver for: %s", device->config.device_path.c_str());
      close_serial_port(device->serial_fd);
      continue;
    }

    // Start read thread
    device->should_stop = false;
    device->read_thread = std::thread(&MultiLidarNode::device_read_thread, this, device);

    RCLCPP_INFO(get_logger(), "Started data acquisition for: %s", device->config.device_path.c_str());
  }
}

void MultiLidarNode::stop_data_acquisition() {
  for (auto& device : devices_) {
    // Signal thread to stop
    device->should_stop = true;

    // Stop driver
    if (device->driver) {
      device->driver->stop();
    }

    // Wait for read thread to finish
    if (device->read_thread.joinable()) {
      device->read_thread.join();
    }

    // Close serial port
    close_serial_port(device->serial_fd);
  }
}

void MultiLidarNode::configure_serial_port(int fd, int baudrate) {
  struct termios options;
  tcgetattr(fd, &options);

  // Set baud rate
  speed_t speed;
  switch (baudrate) {
  case 230400: speed = B230400; break;
  case 115200: speed = B115200; break;
  case 57600:  speed = B57600;  break;
  case 38400:  speed = B38400;  break;
  case 19200:  speed = B19200;  break;
  case 9600:   speed = B9600;   break;
  default:     speed = B230400; break;
  }

  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  // Configure 8N1
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Raw input
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  tcsetattr(fd, TCSANOW, &options);
}

void MultiLidarNode::close_serial_port(int fd) {
  if (fd >= 0) {
    close(fd);
  }
}

void MultiLidarNode::device_read_thread(std::shared_ptr<LidarDevice> device) {
  constexpr size_t BUFFER_SIZE = 1024;
  uint8_t buffer[BUFFER_SIZE];

  while (!device->should_stop) {
    ssize_t bytes_read = read(device->serial_fd, buffer, BUFFER_SIZE);

    if (bytes_read > 0) {
      device->driver->process_data(buffer, bytes_read);
    }
    else if (bytes_read < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_ERROR(get_logger(), "Read error on device %s: %s",
          device->config.device_path.c_str(), strerror(errno));
        break;
      }
    }

    // Small sleep to prevent excessive CPU usage
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

void MultiLidarNode::fusion_timer_callback() {
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if (fusion_processor_->is_ready_to_fuse()) {
    auto fused_scan = fusion_processor_->create_fused_scan(get_current_timestamp_ns());
    // Transform fused scan to configured fused_frame_id properly
    fused_scan = transform_scan_to_frame(fused_scan, fusion_config_.fused_frame_id);
    fused_scan_publisher_->publish(fused_scan);
    fusion_processor_->clear();
    stats_.fusion_cycles_completed++;
    stats_.total_scans_published++;
    stats_.last_fused_scan_time = now();
  }
}

void MultiLidarNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  motion_corrector_->add_cmd_vel(*msg, get_current_timestamp_ns());
}

void MultiLidarNode::wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  uint64_t timestamp_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
  motion_corrector_->add_wheel_odom(*msg, timestamp_ns);
}

void MultiLidarNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(motion_mutex_);
  uint64_t timestamp_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;
  motion_corrector_->add_imu_data(*msg, timestamp_ns);
}

void MultiLidarNode::on_scan_received(const LidarScan& scan, const std::string& device_name) {
  std::lock_guard<std::mutex> lock(scan_mutex_);

  stats_.total_scans_received++;

  // Apply motion correction if enabled
  LidarScan corrected_scan = scan;
  if (motion_config_.enable_correction) {
    std::lock_guard<std::mutex> motion_lock(motion_mutex_);
    corrected_scan = motion_corrector_->correct_scan(scan);
  }

  // Publish individual topic if enabled
  if (enable_individual_topics_) {
    for (auto& device : devices_) {
      if (device->config.device_path == device_name && device->publisher) {
        auto ros_msg = lidar_scan_to_ros_msg(corrected_scan);
        device->publisher->publish(ros_msg);
        break;
      }
    }
  }

  // Add to fusion processor
  fusion_processor_->add_scan(corrected_scan);
}

uint64_t MultiLidarNode::get_current_timestamp_ns() const {
  auto now = std::chrono::high_resolution_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

sensor_msgs::msg::LaserScan MultiLidarNode::lidar_scan_to_ros_msg(const LidarScan& scan) const {
  sensor_msgs::msg::LaserScan msg;

  // Set header
  msg.header.stamp.sec = scan.scan_start_ns / 1000000000ULL;
  msg.header.stamp.nanosec = scan.scan_start_ns % 1000000000ULL;
  msg.header.frame_id = scan.frame_id;

  // Set scan parameters (create 450-ray scan matching vendor expectation)
  msg.angle_min = 0.0;
  msg.angle_max = 2.0 * M_PI;
  // dynamic angle_increment like legacy driver: ANGLE_TO_RADIAN(mSpeed/4500.0)
  // legacy mSpeed corresponds to scan.motor_speed
  if (scan.motor_speed > 0) {
    msg.angle_increment = (scan.motor_speed / 4500.0f) * 3141.59f / 180000.0f; // ANGLE_TO_RADIAN(mSpeed/4500)
  }
  else {
    msg.angle_increment = (2.0 * M_PI) / 450.0; // fallback
  }
  size_t beam_size = static_cast<size_t>(std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment));
  if (beam_size < 450 || beam_size > 1000) {
    beam_size = 450; // constrain
    msg.angle_increment = (msg.angle_max - msg.angle_min) / static_cast<double>(beam_size);
  }
  msg.range_min = 0.0; // match legacy
  msg.range_max = 15.0;
  // Initialize arrays
  msg.ranges.assign(beam_size, std::numeric_limits<float>::infinity());
  msg.intensities.assign(beam_size, std::numeric_limits<float>::infinity());
  // Sort points by angle ascending like legacy driver before inversion mapping
  std::vector<LidarPoint> sorted_points = scan.points;
  std::sort(sorted_points.begin(), sorted_points.end(), [](const LidarPoint& a, const LidarPoint& b) {return a.angle_deg < b.angle_deg;});
  for (const auto& point : sorted_points) {
    float angle_radian = point.angle_deg * 3141.59f / 180000.0f; // ANGLE_TO_RADIAN
    int index = static_cast<int>((msg.angle_max - angle_radian) / msg.angle_increment);
    if (index >= 0 && index < static_cast<int>(beam_size)) {
      float range_m = point.distance_mm / 1000.0f;
      msg.ranges[index] = range_m;
      msg.intensities[index] = point.intensity;
    }
  }
  msg.scan_time = 0.0; // legacy
  msg.time_increment = 1.0 / 450.0; // legacy fixed value

  return msg;
}

sensor_msgs::msg::LaserScan MultiLidarNode::transform_scan_to_frame(const sensor_msgs::msg::LaserScan& in,
                                                    const std::string& target_frame) const {
  if (in.header.frame_id == target_frame) {
    return in; // no transform needed
  }
  sensor_msgs::msg::LaserScan out = in;
  try {
    auto tf = tf_buffer_->lookupTransform(target_frame, in.header.frame_id, tf2::TimePointZero, std::chrono::milliseconds(20));
    double tx = tf.transform.translation.x;
    double ty = tf.transform.translation.y;
    double qx = tf.transform.rotation.x;
    double qy = tf.transform.rotation.y;
    double qz = tf.transform.rotation.z;
    double qw = tf.transform.rotation.w;
    tf2::Quaternion q(qx,qy,qz,qw);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);

    const size_t n = in.ranges.size();
    // Prepare fresh arrays to avoid in-place overwrite distortions
    std::vector<float> new_ranges(n, std::numeric_limits<float>::infinity());
    std::vector<float> new_intensities(n, 0.0f);

    for (size_t i=0;i<n;i++) {
      float r = in.ranges[i];
      if (!std::isfinite(r)) continue;
      double angle = in.angle_min + static_cast<double>(i) * in.angle_increment;
      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      // rotate then translate
      double xr = x*std::cos(yaw) - y*std::sin(yaw) + tx;
      double yr = x*std::sin(yaw) + y*std::cos(yaw) + ty;
      double new_r = std::hypot(xr, yr);
      double new_theta = std::atan2(yr, xr);
      while (new_theta < in.angle_min) new_theta += 2*M_PI;
      while (new_theta >= in.angle_max) new_theta -= 2*M_PI;
      int new_index = static_cast<int>(std::round((new_theta - in.angle_min)/in.angle_increment));
      if (new_index >=0 && new_index < static_cast<int>(n)) {
        if (!std::isfinite(new_ranges[new_index]) || new_r < new_ranges[new_index]) {
          if (std::isfinite(new_ranges[new_index])) {
            // collision overwrite
            const_cast<MultiLidarNode*>(this)->stats_.transform_collisions++;
          }
          new_ranges[new_index] = static_cast<float>(new_r);
          new_intensities[new_index] = in.intensities[i];
        } else {
          // keep existing closer value
        }
      }
    }
    out.ranges.swap(new_ranges);
    out.intensities.swap(new_intensities);
    out.header.frame_id = target_frame;
  } catch (const tf2::TransformException& ex) {
    const_cast<MultiLidarNode*>(this)->stats_.tf_failures++;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "TF transform failed (%s->%s): %s", in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return in; // publish original frame if TF fails
  }
  return out;
}

void MultiLidarNode::diagnostics_callback() {
  RCLCPP_INFO(get_logger(), "=== Multi-LIDAR Node Diagnostics ===");
  RCLCPP_INFO(get_logger(), "Scans received: %zu, published: %zu, fusion cycles: %zu",
    stats_.total_scans_received, stats_.total_scans_published, stats_.fusion_cycles_completed);
  RCLCPP_INFO(get_logger(), "Transform collisions: %zu, TF failures: %zu", stats_.transform_collisions, stats_.tf_failures);

  for (size_t i = 0; i < devices_.size(); ++i) {
    const auto& device = devices_[i];
    RCLCPP_INFO(get_logger(), "Device %zu (%s): %s",
      i, device->config.device_path.c_str(), device->driver->get_status_string().c_str());
  }

  if (fusion_processor_) {
    RCLCPP_INFO(get_logger(), "%s", fusion_processor_->get_fusion_stats().c_str());
  }

  if (motion_corrector_) {
    RCLCPP_INFO(get_logger(), "%s", motion_corrector_->get_correction_stats().c_str());
  }
}

} // namespace sigyn_lidar_v2
