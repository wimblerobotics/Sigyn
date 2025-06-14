#include "MotorModule.h"
// #include "sigyn_interfaces/action/move_elevator.h"
// #include "std_msgs/msg/float32.h"
// #include "tmicro_ros.h"
#include <Wire.h>
#include <stdio.h>

#include "SerialManager.h"
// #include <geometry_msgs/msg/twist.h>

MotorModule::Motor* MotorModule::createMotor(MotorType motor_type) {
  if (motor_type == Elevator) {
    return new Motor(
        Motor::kElevatorBottomLimitSwitchPin, Motor::kElevatorStepDirectionPin,
        Motor::kElevatorStepPulsePin, Motor::kElevatorTopLimitSwitchPin, 0.9,
        0.0, 0.000180369, false);  // 5544 pules per meter.
  } else {
    return new Motor(
        Motor::kExtenderInLimitSwitchPin, Motor::kExtenderStepDirectionPin,
        Motor::kExtenderStepPulsePin, Motor::kExtenderOutLimitSwitchPin, 0.342,
        0.0, 0.000149626, true);  // 6683 pulses per meter.
  }
}

void MotorModule::handleTwistMessage(const String& data) {
  // Parse twist data format: "linear_x,angular_z"
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {
    float linear_x_ = 0.0f;
    float angular_z_ = 0.0f;
    linear_x_ = data.substring(0, commaIndex).toFloat();
    angular_z_ = data.substring(commaIndex + 1).toFloat();
    // char diagnostic_message[256];
    // snprintf(diagnostic_message, sizeof(diagnostic_message),
    //          "INFO [MotorModule::HandleTwistMessage] linear_x: %4.3f, "
    //          "angular_z: %4.3f",
    //          linear_x_, angular_z_);
    // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);

    if (linear_x_ > 0.0f && elevator_) {
      elevator_->SetTargetPosition(elevator_->getCurrentPosition() + 0.001f);
    } else if (linear_x_ < 0.0f && elevator_) {
      elevator_->SetTargetPosition(elevator_->getCurrentPosition() - 0.001f);
    }

    if (angular_z_ > 0.0f && extender_) {
      extender_->SetTargetPosition(extender_->getCurrentPosition() + 0.001f);
    } else if (angular_z_ < 0.0f && extender_) {
      extender_->SetTargetPosition(extender_->getCurrentPosition() - 0.001f);
    }
  }
}

void MotorModule::Motor::continueOutstandingMovementRequests() {
  if (pending_movement_command_) {
    Direction direction_to_travel = remaining_pulses_ > 0 ? kUp : kDown;
    char diagnostic_message[256];

    if ((direction_to_travel == kUp) && atUpLimit()) {
      // snprintf(diagnostic_message, sizeof(diagnostic_message),
      //          "INFO [MotorModule::doMovementRequest] Already at up limit, "
      //          "not moving further up, current_position: %7.6f",
      //          current_position_);
      // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
      pending_movement_command_ = false;
      return;
    }

    if ((direction_to_travel == kDown) && atDownLimit()) {
      // snprintf(diagnostic_message, sizeof(diagnostic_message),
      //          "INFO [MotorModule::doMovementRequest] Already at down limit, "
      //          "not moving further down, current_position: %7.6f",
      //          current_position_);
      // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
      pending_movement_command_ = false;
      return;
    }

    stepPulse(direction_to_travel);
    if ((direction_to_travel == kUp) && (remaining_pulses_ > 0)) {
      remaining_pulses_--;
    } else if ((direction_to_travel == kDown) && (remaining_pulses_ < 0)) {
      remaining_pulses_++;
    }

    if (remaining_pulses_ == 0) {
      pending_movement_command_ = false;
    }

    // snprintf(diagnostic_message, sizeof(diagnostic_message),
    //          "INFO [MotorModule::doMovementRequest] remaining_pulses: %ld, "
    //          "current_position: %7.6f",
    //          remaining_pulses_, current_position_);
    // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
  } else {
    if (pending_action_) {
      pending_action_ = false;
    }
  }
}

// bool MotorModule::HandleActionCancel(
//     rclc_action_goal_handle_t *ros_cancel_request, void *context) {
//   MotorModule *motor = (MotorModule *)context;
//   sigyn_interfaces__action__MoveElevator_GetResult_Response result = {0};
//   result.result.final_position = motor->current_position_;
//   // ### Note: Possibly repeat after small delay until return code ==
//   // RCL_RET_OK. Also other send places.
//   rcl_ret_t rclc_result = rclc_action_send_result(motor->action_goal_handle_,
//                                                   GOAL_STATE_CANCELED,
//                                                   &result);

//   char diagnostic_message[256];
//   snprintf(diagnostic_message, sizeof(diagnostic_message),
//            "INFO [MotorModule::CancelGoal] "
//            "goal canceled, rclc_action_send_result: %ld",
//            rclc_result);
//   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
//   return true;
// }

// rcl_ret_t
// MotorModule::HandleActionRequest(rclc_action_goal_handle_t *goal_handle,
//                                  void *context) {
//   char diagnostic_message[256];
//   MotorModule *motor = (MotorModule *)context;

//   sigyn_interfaces__action__MoveElevator_SendGoal_Request *req =
//       (sigyn_interfaces__action__MoveElevator_SendGoal_Request *)
//           goal_handle->ros_goal_request;

//   float goal_position = req->goal.goal_position;
//   snprintf(diagnostic_message, sizeof(diagnostic_message),
//            "INFO [MotorModule::HandleActionRequest] goal_position: %4.3f",
//            goal_position);
//   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);

//   if (motor->current_position_ != goal_position) {
//     motor->remaining_pulses_ = (goal_position - motor->current_position_) /
//                                motor->travel_mm_per_pulse_;
//     motor->pending_movement_command_ = true;

//     char diagnostic_message[256];
//     snprintf(diagnostic_message, sizeof(diagnostic_message),
//              "INFO [MotorModule::HandleActionRequestl] goal_position: %7.6f,
//              " "current_position: %7.6f, remaining_pulses: %ld",
//              goal_position, motor->current_position_,
//              motor->remaining_pulses_);
//     TMicroRos::singleton().PublishDiagnostic(diagnostic_message);

//     motor->action_goal_handle_ = goal_handle;
//     motor->pending_action_ = true;
//   }

//   return RCL_RET_ACTION_GOAL_ACCEPTED;
// }

// void MotorModule::home() {
//   while (!AtDownLimit()) {
//     StepPulse(kDown);
//   }
// }

// void MotorModule::SendFeedback() {
//   if (pending_action_) {
//     sigyn_interfaces__action__MoveElevator_FeedbackMessage feedback;
//     feedback.goal_id = action_goal_handle_->goal_id;
//     feedback.feedback.current_position = current_position_;
//     rclc_action_publish_feedback(action_goal_handle_, &feedback);
//   }
// }

// void MotorModule::sendSuccess() {
//   sigyn_interfaces__action__MoveElevator_GetResult_Response result = {0};
//   result.result.final_position = current_position_;
//   rcl_ret_t rclc_result = rclc_action_send_result(
//       action_goal_handle_, GOAL_STATE_SUCCEEDED, &result);

//   char diagnostic_message[256];
//   snprintf(diagnostic_message, sizeof(diagnostic_message),
//            "INFO [MotorModule::sendSuccess] rclc_action_send_result: %ld",
//            rclc_result);
//   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
// }

MotorModule::MotorModule() : TModule() {
  // Initialize the motors.
  elevator_ = createMotor(Elevator);
  extender_ = createMotor(Extender);
}

MotorModule& MotorModule::singleton() {
  if (!g_instance_) {
    g_instance_ = new MotorModule();
  }
  return *g_instance_;
}

MotorModule* MotorModule::g_instance_ = nullptr;

MotorModule::Motor::Motor(int8_t pin_down_limit_switch,
                          uint8_t pin_step_direction, uint8_t pin_step_pulse,
                          uint8_t pin_up_limit_switch, float position_max_up,
                          float position_min_down, float travel_mm_per_pulse,
                          bool reverse_travel)
    : current_position_(0),
      pending_action_(false),
      pending_movement_command_(false),
      pin_down_limit_switch_(pin_down_limit_switch),
      pin_step_direction_(pin_step_direction),
      pin_step_pulse_(pin_step_pulse),
      pin_up_limit_switch_(pin_up_limit_switch),
      position_max_up_(position_max_up),
      position_min_down_(position_min_down),
      remaining_pulses_(0),
      reverse_travel_(reverse_travel),
      travel_mm_per_pulse_(travel_mm_per_pulse) {
  pinMode(pin_down_limit_switch, INPUT);
  pinMode(pin_up_limit_switch, INPUT);
  pinMode(pin_step_direction, OUTPUT);
  pinMode(pin_step_pulse, OUTPUT);
}

// Motor class method implementations
bool MotorModule::Motor::atDownLimit() {
  bool atBottom = digitalRead(pin_down_limit_switch_);
  if (reverse_travel_) {
    atBottom = !atBottom;
  }

  if (atBottom) {
    // current_position_ = position_min_down_;
    // char diagnostic_message[256];
    // snprintf(diagnostic_message, sizeof(diagnostic_message),
    //          "INFO [Motor::AtDownLimit] At bottom, current_position_: %4.3f",
    //          current_position_);
    // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
  }

  return atBottom;
}

bool MotorModule::Motor::atUpLimit() {
  bool at_top = digitalRead(pin_up_limit_switch_);
  if (reverse_travel_) {
    at_top = !at_top;
  }

  if (at_top) {
    // current_position_ = position_max_up_;
    // char diagnostic_message[256];
    // snprintf(diagnostic_message, sizeof(diagnostic_message),
    //          "INFO [Motor::AtUpLimit] At top, current_position_: %4.3f",
    //          current_position_);
    // SerialManager::singleton().SendDiagnosticMessage(diagnostic_message);
  }

  return at_top;
}

// MotorModule method implementations
void MotorModule::setup() {
  // Initialize the motors
}

void MotorModule::loop() {
  static bool has_homed = false;
  if (!has_homed) {
    // Home the motors on first loop
    elevator_->Home();
    extender_->Home();
    has_homed = true;
  }

  elevator_->continueOutstandingMovementRequests();
  extender_->continueOutstandingMovementRequests();
}

bool MotorModule::IsUnsafe() {
  return false;  // TODO: Implement safety checks
}

void MotorModule::ResetSafetyFlags() {
  // TODO: Implement safety flag reset
}
