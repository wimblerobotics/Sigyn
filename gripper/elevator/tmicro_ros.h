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

class TMicroRos : TModule
{
public:
  // Check if ROS time appears to be correct and, if not, fix it.
  // Returns a reasonable ROS time.
  static int64_t FixedTime(const char* caller);

  // Publish a diagnistoc message.
  static void PublishDiagnostic(const char* msg);

  // Singleton constructor.
  static TMicroRos& singleton();

protected:
  enum Direction
  {
    kUp,
    kDown
  };

  // Check if the elevator is at the bottom limit.
  static bool ElevatorAtBottomLimit();

  // Check if the elevator is at the top limit.
  static bool ElevatorAtTopLimit();

  // Check if the extender is at the most extended limit.
  static bool ExtenderAtOutLimit();

  // Check if the extender is at the least extended limit.
  static bool ExtenderAtInLimit();

  // From TModule.
  void loop();

  // From TModule
  const char* name()
  {
    return "uRos";
  }

  // From TModule
  void setup();

  // Step one pulse.
  static void ElevatorStepPulse(Direction upwdirectionrds);
  static void ExtenderStepPulse(Direction upwdirectionrds);

private:
  enum State
  {
    kWaitingAgent,
    kAgentAvailable,
    kAgentConnected,
    kAgentDisconnected
  } state_;

  enum
  {
    // For limit switches, 0 => interrupted, 1 => not interrupted.
    kExtenderInLimitSwitchPin = 35,      // Echo 3
    kExtenderOutLimitSwitchPin = 34,     // Trigger 3
    kExtenderStepPulsePin = 37,          // Echo 2
    kExtenderStepDirectionPin = 36,      // Trigger 2
    kElevatorBottomLimitSwitchPin = 41,  // Echo 1
    kElevatorTopLimitSwitchPin = 40,     // Trigger 1

    // kElevatorTopLimitSwitchPin = 41,     // Echo 1
    // kElevatorBottomLimitSwitchPin = 40,  // Trigger 1

    kElevatorStepPulsePin = 15,     // Echo 0
    kElevatorStepDirectionPin = 14  // Trigger 0
  };

  // Private constructor.
  TMicroRos();

  bool CreateEntities();

  void DestroyEntities();

  static void doElevatorCommand();
  static void doExtenderCommand();

  // Gripper action handler.
  // static rcl_ret_t HandleGripperGoal(rclc_action_goal_handle_t* goal_handle, void* context);
  // static bool HandleGripperCancel(rclc_action_goal_handle_t* goal_handle, void* context);

      // Sync ROS time.
      static void SyncTime(const char* caller, uint32_t fixed_time_call_count);

  // Regular maintenance, publish stats, etc.
  static void TimerCallback(rcl_timer_t* timer, int64_t last_call_time);

  // Handler for elevator_cmd messages.
  static void ElevatorCommandCallback(const void* msg);

  // Handler for elevator_cmd messages.
  static void ExtenderCommandCallback(const void* msg);

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

  // Elevator position;
  static float current_elevator_position_;  // In meters.
  static float current_extender_position_;  // In meters.

  // Mm per pulse.
  static const float kElevatorMmPerPulse_;
  static const float kExtenderMmPerPulse_;

  static int32_t elevator_remaining_pulses_;
  static int32_t extender_remaining_pulses_;
  static bool elevator_has_command_;
  static bool extender_has_command_;

  // Singleton instance.
  static TMicroRos* g_singleton_;
};
