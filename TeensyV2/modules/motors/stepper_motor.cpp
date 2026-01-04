// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "stepper_motor.h"

#include <cstdlib>
#include <cstring>

namespace sigyn_teensy {

  static StepperMotor* g_instance = nullptr;

  StepperMotor& StepperMotor::getInstance() {
    if (!g_instance) {
      g_instance = new StepperMotor();
    }
    return *g_instance;
  }

  StepperMotor::StepperMotor() : Module(), serial_(SerialManager::getInstance()) {
    // Construct motors now
    elevator_ = createMotor(true);
    extender_ = createMotor(false);
  }

  void StepperMotor::setup() {
    // Nothing blocking here for now
    serial_.sendDiagnosticMessage("INFO", name(), "Setup complete");
  }

  void StepperMotor::loop() {
    static bool homed = false;
    if (!homed) {
      if (elevator_) elevator_->home();
      if (extender_) extender_->home();
      homed = true;
    }

    // Check for TWIST command "linear_x,angular_z"
    if (serial_.hasNewTwistCommand()) {
      const char* twist_data = SerialManager::getInstance().getLatestTwistCommand();
      handleTwistMessage(twist_data);
    }

    if (elevator_) elevator_->continueOutstandingMovementRequests();
    if (extender_) extender_->continueOutstandingMovementRequests();
  }

  void StepperMotor::handleTwistMessage(const char* data) {
    // Parse twist message: "linear_x:<value>,angular_z:<value>"
    float linear_x = 0.0f;
    float angular_z = 0.0f;

    if (data) {
      const char* linear_start = strstr(data, "linear_x:");
      if (linear_start) {
        linear_start += strlen("linear_x:");
        linear_x = strtof(linear_start, nullptr);
      }

      const char* angular_start = strstr(data, "angular_z:");
      if (angular_start) {
        angular_start += strlen("angular_z:");
        angular_z = strtof(angular_start, nullptr);
      }
    }

    // Move in very small increments per command
    const float kStep = 0.001f; // meters
    if (linear_x > 0.0f && elevator_) {
      elevator_->setTargetPosition(elevator_->getCurrentPosition() + kStep);
    }
    else if (linear_x < 0.0f && elevator_) {
      elevator_->setTargetPosition(elevator_->getCurrentPosition() - kStep);
    }

    if (angular_z > 0.0f && extender_) {
      extender_->setTargetPosition(extender_->getCurrentPosition() + kStep);
    }
    else if (angular_z < 0.0f && extender_) {
      extender_->setTargetPosition(extender_->getCurrentPosition() - kStep);
    }
  }

  // ---- Motor implementation ----

  StepperMotor::Motor::Motor(int8_t pin_down_limit_switch,
    uint8_t pin_step_direction,
    uint8_t pin_step_pulse,
    uint8_t pin_up_limit_switch,
    float position_max_up,
    float position_min_down,
    float travel_m_per_pulse,
    bool reverse_travel)
    : pin_down_limit_switch_(pin_down_limit_switch),
    pin_step_direction_(pin_step_direction),
    pin_step_pulse_(pin_step_pulse),
    pin_up_limit_switch_(pin_up_limit_switch),
    position_max_up_m_(position_max_up),
    position_min_down_m_(position_min_down),
    reverse_travel_(reverse_travel),
    travel_m_per_pulse_(travel_m_per_pulse) {
    pinMode(pin_down_limit_switch, INPUT);
    pinMode(pin_up_limit_switch, INPUT);
    pinMode(pin_step_direction, OUTPUT);
    pinMode(pin_step_pulse, OUTPUT);
  }

  bool StepperMotor::Motor::atDownLimit() {
    bool at_bottom = digitalRead(pin_down_limit_switch_);
    if (reverse_travel_) at_bottom = !at_bottom;
    return at_bottom;
  }

  bool StepperMotor::Motor::atUpLimit() {
    bool at_top = digitalRead(pin_up_limit_switch_);
    if (reverse_travel_) at_top = !at_top;
    return at_top;
  }

  void StepperMotor::Motor::stepPulse(Direction direction) {
    const int pd = 500;  // microseconds
    if ((direction == kUp) && atUpLimit()) return;
    if ((direction == kDown) && atDownLimit()) return;

    digitalWriteFast(pin_step_direction_, direction == reverse_travel_ ? kUp : kDown);
    digitalWriteFast(pin_step_pulse_, HIGH);
    delayMicroseconds(pd);
    digitalWriteFast(pin_step_pulse_, LOW);
    delayMicroseconds(pd);
    if (direction == kUp) {
      current_position_m_ += travel_m_per_pulse_;
    }
    else {
      current_position_m_ -= travel_m_per_pulse_;
    }
  }

  void StepperMotor::Motor::home() {
    // Simple blocking home: move down until limit
    uint32_t guard = 0;
    while (!atDownLimit() && guard < 200000) {  // ~100ms max at 500us pulses
      stepPulse(kDown);
      guard++;
    }
    current_position_m_ = position_min_down_m_;
  }

  void StepperMotor::Motor::setTargetPosition(float target_position_m) {
    if (target_position_m > position_max_up_m_) target_position_m = position_max_up_m_;
    if (target_position_m < position_min_down_m_) target_position_m = position_min_down_m_;
    if (fabsf(current_position_m_ - target_position_m) < 1e-6f) return;
    remaining_pulses_ = (int32_t)((target_position_m - current_position_m_) / travel_m_per_pulse_);
    pending_movement_command_ = (remaining_pulses_ != 0);
  }

  void StepperMotor::Motor::moveByDelta(float delta_m) {
    setTargetPosition(current_position_m_ + delta_m);
  }

  void StepperMotor::Motor::continueOutstandingMovementRequests() {
    if (!pending_movement_command_) return;
    Direction dir = remaining_pulses_ > 0 ? kUp : kDown;
    if ((dir == kUp && atUpLimit()) || (dir == kDown && atDownLimit())) {
      pending_movement_command_ = false;
      return;
    }
    stepPulse(dir);
    if (dir == kUp && remaining_pulses_ > 0) {
      remaining_pulses_--;
    }
    else if (dir == kDown && remaining_pulses_ < 0) {
      remaining_pulses_++;
    }
    if (remaining_pulses_ == 0) pending_movement_command_ = false;
  }

  StepperMotor::Motor* StepperMotor::createMotor(bool elevator) {
    if (elevator) {
      // 5544 pulses per meter (from legacy); 1/5544 m per pulse
      return new Motor(Motor::kElevatorBottomLimitSwitchPin,
        Motor::kElevatorStepDirectionPin,
        Motor::kElevatorStepPulsePin,
        Motor::kElevatorTopLimitSwitchPin,
        /*max_up*/ 0.90f, /*min_down*/ 0.0f,
        /*travel_m_per_pulse*/ (1.0f / 5544.0f),
        /*reverse*/ false);
    }
    else {
      // 6683 pulses per meter
      return new Motor(Motor::kExtenderInLimitSwitchPin,
        Motor::kExtenderStepDirectionPin,
        Motor::kExtenderStepPulsePin,
        Motor::kExtenderOutLimitSwitchPin,
        /*max_out*/ 0.342f, /*min_in*/ 0.0f,
        /*travel_m_per_pulse*/ (1.0f / 6683.0f),
        /*reverse*/ true);
    }
  }

}  // namespace sigyn_teensy

