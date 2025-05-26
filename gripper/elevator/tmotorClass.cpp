#include "tmotorClass.h"
#include "sigyn_interfaces/action/move_elevator.h"
#include "std_msgs/msg/float32.h"
#include "tmicro_ros.h"
#include <Wire.h>
#include <stdio.h>

TMotorClass *TMotorClass::CreateMotor(TMotorType motor_type) {
  if (motor_type == Elevator) {
    return new TMotorClass(TMotorClass::kElevatorBottomLimitSwitchPin,
                           TMotorClass::kElevatorStepDirectionPin,
                           TMotorClass::kElevatorStepPulsePin,
                           TMotorClass::kElevatorTopLimitSwitchPin, 0.9, 0.0,
                           0.000180369, false);
  } else {
    return new TMotorClass(TMotorClass::kExtenderInLimitSwitchPin,
                           TMotorClass::kExtenderStepDirectionPin,
                           TMotorClass::kExtenderStepPulsePin,
                           TMotorClass::kExtenderOutLimitSwitchPin, 0.342, 0.0,
                           0.000149626, true);
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

void TMotorClass::HandleGripperServiceCallback(const void *request_msg,
                                               void *response_msg,
                                               void *context) {
  // Cast messages to expected types
  TMotorClass *motor = (TMotorClass *)context;
  sigyn_interfaces__srv__GripperPosition_Response *res_in =
      (sigyn_interfaces__srv__GripperPosition_Response *)response_msg;

  // Handle request message and set the response message values
  res_in->position = motor->current_position_;
}

void TMotorClass::HandleMoveTopicCallback(const void *msg, void *context) {
  TMotorClass *motor = (TMotorClass *)context;
  const std_msgs__msg__Float32 *command = (const std_msgs__msg__Float32 *)msg;

  char diagnostic_message[256];
  snprintf(diagnostic_message, sizeof(diagnostic_message),
           "INFO [TMotorClass::HandleMoveTopicCallback] command: %4.3f",
           command->data);
  TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  float target_position = command->data;
  if (motor->current_position_ != target_position) {
    motor->remaining_pulses_ = (target_position - motor->current_position_) /
                               motor->travel_mm_per_pulse_;
    motor->pending_movement_command_ = true;

    char diagnostic_message[256];
    snprintf(
        diagnostic_message, sizeof(diagnostic_message),
        "INFO [TMotorClass::HandleMoveTopicCallback] target_position: %7.6f, "
        "current_position: %7.6f, remaining_pulses: %ld",
        target_position, motor->current_position_, motor->remaining_pulses_);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }
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

  if ((direction == kUp) && AtUpLimit()) {
    return;
  }

  if ((direction == kDown) && AtDownLimit()) {
    return;
  }

  digitalWrite(pin_step_direction_, direction == reverse_travel_ ? kUp : kDown);
  digitalWrite(pin_step_pulse_, HIGH);
  delayMicroseconds(pd);
  digitalWrite(pin_step_pulse_, LOW);
  delayMicroseconds(pd);
  if (direction == kUp) {
    current_position_ += travel_mm_per_pulse_;
  } else {
    current_position_ -= travel_mm_per_pulse_;
  }
}

void TMotorClass::MoveByDelta(float delta) {
  float target = current_position_ + delta;
  if (target > position_max_up_) target = position_max_up_;
  if (target < position_min_down_) target = position_min_down_;
  remaining_pulses_ = (target - current_position_) / travel_mm_per_pulse_;
  pending_movement_command_ = (remaining_pulses_ != 0);
}