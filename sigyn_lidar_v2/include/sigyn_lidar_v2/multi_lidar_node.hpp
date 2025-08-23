#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "lidar_types.hpp"
#include "lidar_base.hpp"
#include "motion_corrector.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sigyn_lidar_v2 {

  struct LidarDevice {
    std::unique_ptr<LidarBase> driver;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    LidarConfig config;
    int serial_fd{ -1 };
    std::thread read_thread;
    std::atomic<bool> should_stop{ false };
  };

  class SingleLidarNode : public rclcpp::Node {
  public:
    explicit SingleLidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SingleLidarNode() override;

  private:
    void declare_parameters();
    void load_configuration();
    bool setup_device();
    void start_data_acquisition();
    void stop_data_acquisition();

    void configure_serial_port(int fd, int baudrate);
    void close_serial_port(int fd);
    void device_read_thread();

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_scan_received(const LidarScan& scan);

    uint64_t get_current_timestamp_ns() const;
    sensor_msgs::msg::LaserScan lidar_scan_to_ros_msg(const LidarScan& scan) const;

    // Config
    LidarConfig lidar_config_;
    bool all_ld06_{ false };
    double angle_min_{ 0.0 };
    double angle_max_{ 2.0 * M_PI };
    double angle_increment_{ M_PI / 180.0 * 0.8 };
    double range_min_{ 0.02 };
    double range_max_{ 15.0 };
    bool enable_motion_correction_{ false };
    MotionCorrectionConfig motion_config_;

    // Device
    std::shared_ptr<LidarDevice> device_;

    // Motion correction
    std::unique_ptr<MotionCorrector> motion_corrector_;
    std::mutex motion_mutex_;

    // Thread safety
    std::mutex scan_mutex_;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // Statistics
    struct Stats { size_t scans_received{ 0 }; size_t scans_published{ 0 }; } stats_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    void diagnostics_callback();
  };

} // namespace sigyn_lidar_v2
