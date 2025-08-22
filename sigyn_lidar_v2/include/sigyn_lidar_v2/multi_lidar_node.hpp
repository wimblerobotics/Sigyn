#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lidar_types.hpp"
#include "lidar_base.hpp"
#include "lidar_fusion.hpp"
#include "motion_corrector.hpp"

#include <memory>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sigyn_lidar_v2 {

  // Individual LIDAR device management
  struct LidarDevice {
    std::unique_ptr<LidarBase> driver;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
    LidarConfig config;
    int serial_fd;
    std::thread read_thread;
    std::atomic<bool> should_stop;

    LidarDevice() : serial_fd(-1), should_stop(false) {}
  };

  // Main multi-LIDAR node
  class MultiLidarNode : public rclcpp::Node {
  public:
    explicit MultiLidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MultiLidarNode() override;

  private:
    // Node lifecycle
    void declare_parameters();
    void load_configuration();
    bool setup_devices();
    bool setup_publishers_and_subscribers();
    void start_data_acquisition();
    void stop_data_acquisition();

    // Serial port management
    bool open_serial_port(const std::string& device_path, int baudrate);
    void close_serial_port(int fd);
    void configure_serial_port(int fd, int baudrate);

    // Data processing threads
    void device_read_thread(std::shared_ptr<LidarDevice> device);
    void fusion_timer_callback();

    // ROS callbacks
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // LIDAR data callbacks
    void on_scan_received(const LidarScan& scan, const std::string& device_name);

    // Utility functions
    uint64_t get_current_timestamp_ns() const;
    sensor_msgs::msg::LaserScan lidar_scan_to_ros_msg(const LidarScan& scan) const;
    sensor_msgs::msg::LaserScan transform_scan_to_frame(const sensor_msgs::msg::LaserScan& in,
                                                        const std::string& target_frame) const;

    // Configuration
    std::vector<LidarConfig> lidar_configs_;
    FusionConfig fusion_config_;
    MotionCorrectionConfig motion_config_;
    double fusion_frequency_hz_;
    bool enable_individual_topics_;

    // LIDAR devices
    std::vector<std::shared_ptr<LidarDevice>> devices_;

    // ROS publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    // Processing components
    std::unique_ptr<LidarFusion> fusion_processor_;
    std::unique_ptr<MotionCorrector> motion_corrector_;

    // Timers
    rclcpp::TimerBase::SharedPtr fusion_timer_;

    // Thread safety
    std::mutex scan_mutex_;
    std::mutex motion_mutex_;

    // Statistics
    struct NodeStats {
      size_t total_scans_received;
      size_t total_scans_published;
      size_t fusion_cycles_completed;
      rclcpp::Time last_fused_scan_time;

      NodeStats() : total_scans_received(0), total_scans_published(0),
        fusion_cycles_completed(0), last_fused_scan_time(0, 0, RCL_ROS_TIME) {
      }
    } stats_;

    // Diagnostics timer
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    void diagnostics_callback();

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  };

} // namespace sigyn_lidar_v2
