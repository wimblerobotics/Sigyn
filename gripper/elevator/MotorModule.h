#pragma once

#include "SerialManager.h"
#include "TModule.h"

class MotorModule : TModule {
 public:
  static MotorModule& singleton();

  enum MotorType { Elevator, Extender };

  void handleTwistMessage(const String& data);

  bool IsUnsafe() override;
  void ResetSafetyFlags() override;

  // // Execute the movement request.
  // void continueOutstandingMovementRequests();

  //   // Cancel the goal.
  //   static bool HandleActionCancel(rclc_action_goal_handle_t
  //   *ros_cancel_request,
  //                                  void *context);

  //   // Handle an action request.
  //   static rcl_ret_t HandleActionRequest(rclc_action_goal_handle_t
  //   *goal_handle,
  //                                        void *context);

  // Home the motor.
  // void home();

  // float getCurrentPosition() const { return current_position_; }
  // void setTargetPosition(float target_position) {
  //   if (current_position_ != target_position) {
  //     remaining_pulses_ = (target_position - current_position_) /
  //     travel_mm_per_pulse_; pending_movement_command_ = true;
  //   }
  // }

  // void moveByDelta(float delta) {
  //   float target = current_position_ + delta;
  //   if (target > position_max_up_) target = position_max_up_;
  //   if (target < position_min_down_) target = position_min_down_;
  //   remaining_pulses_ = (target - current_position_) / travel_mm_per_pulse_;
  //   pending_movement_command_ = (remaining_pulses_ != 0);
  //   char diagnostic_message[256];
  //   snprintf(diagnostic_message, sizeof(diagnostic_message),
  //            "INFO [MotorModule::MoveByDelta] target: %4.3f, "
  //            "current_position: %4.3f, remaining_pulses: %ld",
  //            target, current_position_, remaining_pulses_);
  //   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  // }

 private:
  MotorModule();

  class Motor {
   public:
    enum Direction { kUp, kDown };

    enum {
      // For limit switches, 0 => interrupted, 1 => not interrupted.
      kExtenderInLimitSwitchPin = 35,      // Echo 3
      kExtenderOutLimitSwitchPin = 34,     // Trigger 3
      kExtenderStepPulsePin = 37,          // Echo 2
      kExtenderStepDirectionPin = 36,      // Trigger 2
      kElevatorBottomLimitSwitchPin = 41,  // Echo 1
      kElevatorTopLimitSwitchPin = 40,     // Trigger 1
      kElevatorStepPulsePin = 15,          // Echo 0
      kElevatorStepDirectionPin = 14       // Trigger 0
    };

    Motor(int8_t pin_down_limit_switch, uint8_t pin_step_direction,
          uint8_t pin_step_pulse, uint8_t pin_up_limit_switch,
          float position_max_up, float position_min_down,
          float travel_mm_per_pulse, bool reverse_travel);

    ~Motor() {}

    // Public methods to access private members
    bool atDownLimit();

    bool atUpLimit();

    // // Execute the movement request.
    void continueOutstandingMovementRequests();

    float getCurrentPosition() const { return current_position_; }

    void SetTargetPosition(float target_position) {
      if (current_position_ != target_position) {
        remaining_pulses_ =
            (target_position - current_position_) / travel_mm_per_pulse_;
        pending_movement_command_ = true;
      }
    }

    void MoveByDelta(float delta) {
      float target = current_position_ + delta;
      if (target > position_max_up_) target = position_max_up_;
      if (target < position_min_down_) target = position_min_down_;
      remaining_pulses_ = (target - current_position_) / travel_mm_per_pulse_;
      // pending_movement_command_ = (remaining_pulses_ != 0);
      // char diagnostic_message[256];
      // snprintf(diagnostic_message, sizeof(diagnostic_message),
      //          "INFO [Motor::MoveByDelta] target: %4.3f, "
      //          "current_position: %4.3f, remaining_pulses: %ld",
      //          target, current_position_, remaining_pulses_);
      // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
    }

    void ContinueOutstandingMovementRequests();

    void Home() {
      while (!atDownLimit()) {
        stepPulse(kDown);
      }
    }

    void stepPulse(Direction direction) {
      const int pd = 500;  // Pulse Delay period
      // 50 takes about 23 sec to travel 0.4m
      // 20 takes about 21
      // 10 takes about 22
      if ((direction == kUp) && atUpLimit()) {
        return;
      }

      if ((direction == kDown) && atDownLimit()) {
        return;
      }

      digitalWriteFast(pin_step_direction_,
                       direction == reverse_travel_ ? kUp : kDown);
      digitalWriteFast(pin_step_pulse_, HIGH);
      delayMicroseconds(pd);
      digitalWriteFast(pin_step_pulse_, LOW);
      delayMicroseconds(pd);
      if (direction == kUp) {
        current_position_ += travel_mm_per_pulse_;
      } else {
        current_position_ -= travel_mm_per_pulse_;
      }
    }

   private:
    float current_position_;         // Current position in meters.
    bool pending_action_;            // Is there a pending action?
    bool pending_movement_command_;  // Is there a pending movement command?
    const uint8_t pin_down_limit_switch_;  // Pin for the down limit switch.
    const uint8_t pin_step_direction_;     // Pin for the step direction.
    const uint8_t pin_step_pulse_;         // Pin for the step pulse.
    const uint8_t pin_up_limit_switch_;    // Pin for the up limit switch.
    const float position_max_up_;          // Maximum upward position in meters.
    const float position_min_down_;    // Minimum downward position in meters.
    int32_t remaining_pulses_;         // Remaining pulses to move.
    bool reverse_travel_;              // Reverse travel direction.
    const float travel_mm_per_pulse_;  // Travel in mm per pulse.

  };  // Motor class for handling motor operations.

  // Create a motor.
  Motor* createMotor(MotorType motor_type);

  void loop() override;
  const char* name() override { return "MBAT"; }
  void setup() override;

  // rclc_action_goal_handle_t *action_goal_handle_; // Goal handle.

  Motor* elevator_;  // Left motor instance.
  Motor* extender_;  // Right motor instance.
  // int calculatePercentage(float voltage);
  static MotorModule* g_instance_;
};