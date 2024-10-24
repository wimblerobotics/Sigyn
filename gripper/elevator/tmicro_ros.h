/*
  ros2 topic pub -1 /extender_cmd std_msgs/msg/Float32 data:\ 0.2\
  ros2 topic pub -1 /elevator_cmd std_msgs/msg/Float32 data:\ 0.2\
*/

#pragma once

#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>
#include <stdio.h>

#include "sigyn_interfaces/action/move_elevator.h"
#include "tmodule.h"

class TMotorClass;

class TMicroRos : TModule {
public:
  // Check if ROS time appears to be correct and, if not, fix it.
  // Returns a reasonable ROS time.
  static int64_t FixedTime(const char *caller);

  // Publish a diagnistoc message.
  static void PublishDiagnostic(const char *msg);

  // Singleton constructor.
  static TMicroRos &singleton();

protected:
  enum Direction { kUp, kDown };

  // From TModule.
  void loop();

  // From TModule
  const char *name() { return "uRos"; }

  // From TModule
  void setup();

static TMotorClass* foo;
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
  static void SyncTime(const char *caller, uint32_t fixed_time_call_count);

  // Regular maintenance, publish stats, etc.
  static void TimerCallback(rcl_timer_t *timer, int64_t last_call_time);

  // For checking for a reasonable ROS time.
  static volatile int64_t ros_sync_time_;

  // Micro-ROS variables
  rcl_allocator_t allocator_;
  rcl_subscription_t elevator_command_subscriber_;
  rcl_subscription_t extender_command_subscriber_;
  rclc_executor_t executor_;
  rclc_action_server_t gripper_action_server_;
  bool micro_ros_init_successful_;
  rcl_node_t node_;
  rclc_support_t support_;
  rcl_timer_t timer_;

  // ROS publishers.
  rcl_publisher_t diagnostics_publisher_;
  rcl_publisher_t stats_publisher_;

  // ROS messages, allocated once.
  std_msgs__msg__Float32 elevator_command_msg_;
  std_msgs__msg__Float32 extender_command_msg_;
  std_msgs__msg__String string_msg_;

  static TMotorClass *elevator_;
  static TMotorClass *extender_;

  // Singleton instance.
  static TMicroRos *g_singleton_;
};
