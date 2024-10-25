#pragma once

#include "tmicro_ros.h"
#include <cstdint>
#include <micro_ros_arduino.h>

class TMotorClass {
public:
  enum {
    // For limit switches, 0 => interrupted, 1 => not interrupted.
    kExtenderInLimitSwitchPin = 35,     // Echo 3
    kExtenderOutLimitSwitchPin = 34,    // Trigger 3
    kExtenderStepPulsePin = 37,         // Echo 2
    kExtenderStepDirectionPin = 36,     // Trigger 2
    kElevatorBottomLimitSwitchPin = 41, // Echo 1
    kElevatorTopLimitSwitchPin = 40,    // Trigger 1
    kElevatorStepPulsePin = 15,         // Echo 0
    kElevatorStepDirectionPin = 14      // Trigger 0
  };

  TMotorClass(int8_t pin_down_limit_switch, uint8_t pin_step_direction,
              uint8_t pin_step_pulse, uint8_t pin_up_limit_switch,
              float position_max_up, float position_min_down,
              float travel_mm_per_pulse, bool reverse_travel);

  // Execute the movement request.
  void DoMovementRequest();

  // Cancel the goal.
  static bool HandleActionCancel(rclc_action_goal_handle_t *ros_cancel_request,
                                 void *context);

  // Handle an action request.
  static rcl_ret_t HandleActionRequest(rclc_action_goal_handle_t *goal_handle,
                                       void *context);

  // Handle a move topic callback.
  static void HandleMoveTopicCallback(const void *msg, void *context);

  // Home the motor.
  void Home();

protected:
  enum Direction { kUp, kDown };

  // Is the motor at the down limit?
  bool AtDownLimit();

  // Is the motor at the up limit?
  bool AtUpLimit();

  // Send action feedback.
  void SendFeedback();

  // Send action success.
  void sendSuccess();

  // Step one pulse.
  void StepPulse(Direction direction);

  rclc_action_goal_handle_t *action_goal_handle_; // Goal handle.

  float current_position_;              // Current position in meters.
  bool pending_action_;                 // Is there a pending action?
  bool pending_movement_command_;       // Is there a pending movement command?
  const uint8_t pin_down_limit_switch_; // Pin for the down limit switch.
  const uint8_t pin_step_direction_;    // Pin for the step direction.
  const uint8_t pin_step_pulse_;        // Pin for the step pulse.
  const uint8_t pin_up_limit_switch_;   // Pin for the up limit switch.
  const float position_max_up_;         // Maximum upward position in meters.
  const float position_min_down_;       // Minimum downward position in meters.
  int32_t remaining_pulses_;            // Remaining pulses to move.
  bool reverse_travel_;                 // Reverse travel direction.
  const float travel_mm_per_pulse_;     // Travel in mm per pulse.
};