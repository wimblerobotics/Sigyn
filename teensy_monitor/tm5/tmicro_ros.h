#pragma once

#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/twist.h>
#include <micro_ros_arduino.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/string.h>


#include <stdio.h>
// #include <tf2_ros/transform_broadcaster.h>

#include "tmodule.h"

class TMicroRos : TModule {
 public:
  // Check if ROS time appears to be correct and, if not, fix it.
  // Returns a reasonable ROS time.
  static int64_t FixedTime(const char* caller);

  // Called by Temperature module handler to publish a reading.
  static void PublishBattery(const char* frame_id, float voltage);

  // Publish a diagnistoc message.
  static void PublishDiagnostic(const char* msg);

  // Publish the odom transform and topic.
  static void PublishOdometry(double x, double y, double x_velocity,
                              double y_velocity, double z_velocity,
                              float* quaternion);

  // Singleton constructor.
  static TMicroRos& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule
  const char* name() { return "uRos"; }

  // From TModule
  void setup();

 private:
  enum State {
    kWaitingAgent,
    kAgentAvailable,
    kAgentConnected,
    kAgentDisconnected
  } state_;

  // Private constructor.
  TMicroRos();

  bool CreateEntities();

  void DestroyEntities();


  // Sync ROS time.
  static void SyncTime(const char* caller, uint32_t fixed_time_call_count);

  // Regular maintenance, publish stats, etc.
  static void TimerCallback(rcl_timer_t* timer, int64_t last_call_time);

  // Handler for cmd_vel messages.
  static void TwistCallback(const void* twist_msg);

  // Block motor handling until time is synchronized again.
  // This is because performing a time sync function can take a
  // quarter of a second or more. Setting this to true will
  // prevent the cmd_vel listener from acting on any velocity
  // command except one to stop the motors. This will prevent,
  // say, the navigation system from moving the robot while
  // the sensors are reporting invalid times, which would cause
  // the sensors to be ignored.
  bool await_time_sync_;

  // For checking for a reasonable ROS time.
  static volatile int64_t ros_sync_time_;

  // Micro-ROS variables
  rcl_allocator_t allocator_;
  rcl_subscription_t cmd_vel_subscriber_;
  rclc_executor_t executor_;
  bool micro_ros_init_successful_;
  rcl_node_t node_;
  rclc_support_t support_;
  rcl_timer_t timer_;

  // ROS publishers.
  rcl_publisher_t battery_publisher_;
  rcl_publisher_t diagnostics_publisher_;
  rcl_publisher_t odom_publisher_;
  rcl_publisher_t roboclaw_status_publisher_;
  rcl_publisher_t teensy_stats_publisher_;

  // ROS messages, allocated once.
  sensor_msgs__msg__BatteryState battery_msg_;
  std_msgs__msg__String string_msg_;
  geometry_msgs__msg__Twist twist_msg_;
  nav_msgs__msg__Odometry odom_msg_;

  // Motor driver configuration values.
  int32_t accel_quad_pulses_per_second_;
  double max_angular_velocity_;
  double max_linear_velocity_;
  double max_seconds_uncommanded_travel_;
  int32_t quad_pulses_per_meter_;
  double wheel_radius_;
  double wheel_separation_;

  // Singleton instance.
  static TMicroRos* g_singleton_;
};
