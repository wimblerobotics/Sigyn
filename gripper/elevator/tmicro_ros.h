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

#include "tmodule.h"

class TMicroRos : TModule {
 public:
  // Check if ROS time appears to be correct and, if not, fix it.
  // Returns a reasonable ROS time.
  static int64_t FixedTime(const char* caller);

  // Publish a diagnistoc message.
  static void PublishDiagnostic(const char* msg);

  // Singleton constructor.
  static TMicroRos& singleton();

 protected:
  enum Direction { kUp, kDown };

  // Check if the elevator is at the bottom limit.
  static bool AtBottomLimit();

  // Check if the elevator is at the top limit.
  static bool AtTopLimit();

  // From TModule.
  void loop();

  // From TModule
  const char* name() { return "uRos"; }

  // From TModule
  void setup();

  // Step one pulse.
  static void StepPulse(Direction upwdirectionrds);

 private:
  enum State {
    kWaitingAgent,
    kAgentAvailable,
    kAgentConnected,
    kAgentDisconnected
  } state_;

  enum {
    kPinEcho0 = 35,
    kPinTrigger0 = 34,
    kPinEcho1 = 37,
    kPinTrigger1 = 36,
    kPinEcho2 = 41,
    kPinTrigger2 = 40,
    kPinEcho3 = 15,
    kPinTrigger3 = 14
  };

  // Private constructor.
  TMicroRos();

  bool CreateEntities();

  void DestroyEntities();

  // Sync ROS time.
  static void SyncTime(const char* caller, uint32_t fixed_time_call_count);

  // Regular maintenance, publish stats, etc.
  static void TimerCallback(rcl_timer_t* timer, int64_t last_call_time);

  // Handler for cmd_vel messages.
  static void CommandCallback(const void* msg);

  // For checking for a reasonable ROS time.
  static volatile int64_t ros_sync_time_;

  // Micro-ROS variables
  rcl_allocator_t allocator_;
  rcl_subscription_t command_subscriber_;
  rclc_executor_t executor_;
  bool micro_ros_init_successful_;
  rcl_node_t node_;
  rclc_support_t support_;
  rcl_timer_t timer_;

  // ROS publishers.
  rcl_publisher_t diagnostics_publisher_;
  rcl_publisher_t stats_publisher_;

  // ROS messages, allocated once.
  std_msgs__msg__Float32 command_msg_;
  std_msgs__msg__String string_msg_;

  // Elevator position;
  static float current_position_; // In meters.

  // Mm per pulse.
  static const float kMmPerPulse_;  

  // Singleton instance.
  static TMicroRos* g_singleton_;
};
