#include "tmotorClass.h"
#include "sigyn_interfaces/action/move_elevator.h"
#include "std_msgs/msg/float32.h"
#include "tmicro_ros.h"
#include <Wire.h>
#include <stdio.h>
#include <geometry_msgs/msg/twist.h>

TMotorClass *TMotorClass::CreateMotor(TMotorType motor_type) {
  if (motor_type == Elevator) {
    return new TMotorClass(TMotorClass::kElevatorBottomLimitSwitchPin,
                           TMotorClass::kElevatorStepDirectionPin,
                           TMotorClass::kElevatorStepPulsePin,
                           TMotorClass::kElevatorTopLimitSwitchPin, 0.9, 0.0,
                           0.000180369, false); // 5544 pules per meter.
  } else {
    return new TMotorClass(TMotorClass::kExtenderInLimitSwitchPin,
                           TMotorClass::kExtenderStepDirectionPin,
                           TMotorClass::kExtenderStepPulsePin,
                           TMotorClass::kExtenderOutLimitSwitchPin, 0.342, 0.0,
                           0.000149626, true); // 6683 pulses per meter.
  }
}

TMotorClass::TMotorClass(int8_t pin_down_limit_switch,
                         uint8_t pin_step_direction, uint8_t pin_step_pulse,
                         uint8_t pin_up_limit_switch, float position_max_up,
                         float position_min_down, float travel_mm_per_pulse,
                         bool reverse_travel)
    : action_goal_handle_(nullptr), current_position_(0),
      pending_action_(false), pending_movement_command_(false),
      pin_down_limit_switch_(pin_down_limit_switch),
      pin_step_direction_(pin_step_direction), pin_step_pulse_(pin_step_pulse),
      pin_up_limit_switch_(pin_up_limit_switch),
      position_max_up_(position_max_up), position_min_down_(position_min_down),
      remaining_pulses_(0), reverse_travel_(reverse_travel),
      travel_mm_per_pulse_(travel_mm_per_pulse) {
  pinMode(pin_down_limit_switch, INPUT);
  pinMode(pin_up_limit_switch, INPUT);
  pinMode(pin_step_direction, OUTPUT);
  pinMode(pin_step_pulse, OUTPUT);
};

bool TMotorClass::AtDownLimit() {
  bool atBottom = digitalRead(pin_down_limit_switch_);
  if (reverse_travel_) {
    atBottom = !atBottom;
  }

  if (atBottom) {
    current_position_ = position_min_down_;
    char diagnostic_message[256];
    snprintf(
        diagnostic_message, sizeof(diagnostic_message),
        "INFO [TMotorClass::AtDownLimit] At bottom, current_position_: %4.3f",
        current_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return atBottom;
}

bool TMotorClass::AtUpLimit() {
  bool at_top = digitalRead(pin_up_limit_switch_);
  if (reverse_travel_) {
    at_top = !at_top;
  }

  if (at_top) {
    current_position_ = position_max_up_;
    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMotorClass::AtUpLimit] At top, current_position_: %4.3f",
             current_position_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  return at_top;
}

void TMotorClass::ContinueOutstandingMovementRequests() {
  if (pending_movement_command_) {
    if (pending_action_ && action_goal_handle_->goal_cancelled) {
      // HandleActionCancel();
      pending_action_ = false;
      return;
    }

    Direction direction_to_travel = remaining_pulses_ > 0 ? kUp : kDown;
    char diagnostic_message[256];

    if ((direction_to_travel == kUp) && AtUpLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMotorClass::doMovementRequest] Already at up limit, "
               "not moving further up, current_position: %7.6f",
               current_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      pending_movement_command_ = false;
      return;
    }

    if ((direction_to_travel == kDown) && AtDownLimit()) {
      snprintf(diagnostic_message, sizeof(diagnostic_message),
               "INFO [TMotorClass::doMovementRequest] Already at down limit, "
               "not moving further down. current_position: %7.6f",
               current_position_);
      TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      pending_movement_command_ = false;
      return;
    }

    StepPulse(direction_to_travel);
    if ((direction_to_travel == kUp) && (remaining_pulses_ > 0)) {
      remaining_pulses_--;
    } else if ((direction_to_travel == kDown) && (remaining_pulses_ < 0)) {
      remaining_pulses_++;
    }

    if (remaining_pulses_ == 0) {
      pending_movement_command_ = false;
    }

    SendFeedback();
    // snprintf(diagnostic_message, sizeof(diagnostic_message),
    //          "INFO [TMotorClass::doMovementRequest] remaining_pulses: %ld, "
    //          "current_position: %7.6f",
    //          remaining_pulses_, current_position_);
    // TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  } else {
    if (pending_action_) {
      sendSuccess();
      pending_action_ = false;
    }
  }
}

bool TMotorClass::HandleActionCancel(
    rclc_action_goal_handle_t *ros_cancel_request, void *context) {
  TMotorClass *motor = (TMotorClass *)context;
  sigyn_interfaces__action__MoveElevator_GetResult_Response result = {0};
  result.result.final_position = motor->current_position_;
  // ### Note: Possibly repeat after small delay until return code ==
  // RCL_RET_OK. Also other send places.
  rcl_ret_t rclc_result = rclc_action_send_result(motor->action_goal_handle_,
                                                  GOAL_STATE_CANCELED, &result);

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMotorClass::CancelGoal] "
           "goal canceled, rclc_action_send_result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  return true;
}

rcl_ret_t
TMotorClass::HandleActionRequest(rclc_action_goal_handle_t *goal_handle,
                                 void *context) {
  char diagnostic_message[256];
  TMotorClass *motor = (TMotorClass *)context;

  sigyn_interfaces__action__MoveElevator_SendGoal_Request *req =
      (sigyn_interfaces__action__MoveElevator_SendGoal_Request *)
          goal_handle->ros_goal_request;

  float goal_position = req->goal.goal_position;
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMotorClass::HandleActionRequest] goal_position: %4.3f",
           goal_position);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);

  if (motor->current_position_ != goal_position) {
    motor->remaining_pulses_ = (goal_position - motor->current_position_) /
                               motor->travel_mm_per_pulse_;
    motor->pending_movement_command_ = true;

    char diagnostic_message[256];
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TMotorClass::HandleActionRequestl] goal_position: %7.6f, "
             "current_position: %7.6f, remaining_pulses: %ld",
             goal_position, motor->current_position_, motor->remaining_pulses_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);

    motor->action_goal_handle_ = goal_handle;
    motor->pending_action_ = true;
  }

  return RCL_RET_ACTION_GOAL_ACCEPTED;
}

void TMotorClass::Home() {
  while (!AtDownLimit()) {
    StepPulse(kDown);
  }
}

void TMotorClass::SendFeedback() {
  if (pending_action_) {
    sigyn_interfaces__action__MoveElevator_FeedbackMessage feedback;
    feedback.goal_id = action_goal_handle_->goal_id;
    feedback.feedback.current_position = current_position_;
    rclc_action_publish_feedback(action_goal_handle_, &feedback);
  }
}

void TMotorClass::sendSuccess() {
  sigyn_interfaces__action__MoveElevator_GetResult_Response result = {0};
  result.result.final_position = current_position_;
  rcl_ret_t rclc_result = rclc_action_send_result(
      action_goal_handle_, GOAL_STATE_SUCCEEDED, &result);

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMotorClass::sendSuccess] rclc_action_send_result: %ld",
           rclc_result);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
}

void TMotorClass::StepPulse(Direction direction) {
  const int pd = 500; // Pulse Delay period
  // 50 takes about 23 sec to travel 0.4m
  // 20 takes about 21
  // 10 takes about 22

  // static unsigned long last_call_time = 0;
  // static float avg_rate_hz = 0.0f;
  // unsigned long now = micros();
  // if (last_call_time != 0) {
  //   float dt = (now - last_call_time) / 1e6f;
  //   float rate = (dt > 0) ? (1.0f / dt) : 0.0f;
  //   // Simple moving average for smoother output
  //   avg_rate_hz = 0.9f * avg_rate_hz + 0.1f * rate;
  //   char diagnostic_message[128];
  //   snprintf(diagnostic_message, sizeof(diagnostic_message),
  //            "StepPulse rate: %.2f Hz (instant: %.2f Hz)", avg_rate_hz, rate);
  //   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  // }
  // last_call_time = now;

  if ((direction == kUp) && AtUpLimit()) {
    return;
  }

  if ((direction == kDown) && AtDownLimit()) {
    return;
  }

  digitalWriteFast(pin_step_direction_, direction == reverse_travel_ ? kUp : kDown);
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

