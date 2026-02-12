// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include "stepper_motor.h"

#include <cstdlib>
#include <cstring>

namespace sigyn_teensy {

  StepperMotor& StepperMotor::getInstance() {
    static StepperMotor instance;
    return instance;
  }

  StepperMotor::StepperMotor()
      : Module(),
        serial_(SerialManager::getInstance()),
        elevator_(Motor::kElevatorBottomLimitSwitchPin,
                  Motor::kElevatorStepDirectionPin,
                  Motor::kElevatorStepPulsePin,
                  Motor::kElevatorTopLimitSwitchPin,
                  /*max_up*/ 0.8999f,
                  /*min_down*/ 0.0f,
                  /*travel_m_per_pulse*/ (1.0f / 5544.0f),
                  /*reverse*/ false),
        extender_(Motor::kExtenderInLimitSwitchPin,
                  Motor::kExtenderStepDirectionPin,
                  Motor::kExtenderStepPulsePin,
                  Motor::kExtenderOutLimitSwitchPin,
                  /*max_out*/ 0.3418f,
                  /*min_in*/ 0.0f,
                  /*travel_m_per_pulse*/ (1.0f / 6683.0f),
                  /*reverse*/ true) {
  }

  void StepperMotor::setup() {
    // Nothing blocking here for now
    serial_.sendDiagnosticMessage("INFO", name(), "Setup complete");
  }

  void StepperMotor::loop() {
    static bool initial_homing_done = false;
    static bool initial_homing_started = false;
    static bool elevator_homing_started = false;  // Track if elevator homing was initiated
    
    // Start initial homing on first loop
    if (!initial_homing_started) {
      extender_.startHoming();  // Retract first (safer)
      initial_homing_started = true;
    }
    
    // Always continue homing for any motor that's homing (initial or user-requested)
    if (extender_.isHoming() || elevator_.isHoming()) {
      bool extender_done = extender_.continueHoming();
      bool elevator_done = elevator_.continueHoming();
      
      // For initial homing: start elevator after extender is done
      if (!initial_homing_done && extender_done && !elevator_homing_started) {
        elevator_.startHoming();
        elevator_homing_started = true;
      }
      
      if (!initial_homing_done && extender_done && elevator_done && elevator_homing_started) {
        initial_homing_done = true;
        serial_.sendDiagnosticMessage("INFO", name(), "Initial homing complete");
      }
    }

    // Check for STEPHOME command
    if (serial_.hasNewStepHomeCommand()) {
      const char* stephome_data = SerialManager::getInstance().getLatestStepHomeCommand();
      handleStepHomeMessage(stephome_data);
    }
    
    // Check for STEPSTATUS command (always allow)
    if (serial_.hasNewStepStatusCommand()) {
      const char* stepstatus_data = SerialManager::getInstance().getLatestStepStatusCommand();
      handleStepStatusMessage(stepstatus_data);
    }

    // Only process movement commands if not homing
    bool homing_active = elevator_.isHoming() || extender_.isHoming();
    if (!homing_active) {
      // Check for TWIST command "linear_x,angular_z"
      if (serial_.hasNewTwistCommand()) {
        const char* twist_data = SerialManager::getInstance().getLatestTwistCommand();
        handleTwistMessage(twist_data);
      }

      // Check for STEPPOS command "elevator:X,extender:Y"
      if (serial_.hasNewStepPosCommand()) {
        const char* steppos_data = SerialManager::getInstance().getLatestStepPosCommand();
        handleStepPosMessage(steppos_data);
      }

      elevator_.continueOutstandingMovementRequests();
      extender_.continueOutstandingMovementRequests();
    } else {
      // Drain command queues while homing to prevent buildup
      if (serial_.hasNewTwistCommand()) {
        SerialManager::getInstance().getLatestTwistCommand();  // Discard
      }
      if (serial_.hasNewStepPosCommand()) {
        SerialManager::getInstance().getLatestStepPosCommand();  // Discard
      }
    }
    
    // Auto-publish status at 10 Hz (always, even during homing)
    static uint32_t last_status_publish_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_status_publish_ms >= 100) {
      sendStatusMessage();
      last_status_publish_ms = now_ms;
    }
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
    if (linear_x > 0.0f) {
      elevator_.setTargetPosition(elevator_.getCurrentPosition() + kStep);
    }
    else if (linear_x < 0.0f) {
      elevator_.setTargetPosition(elevator_.getCurrentPosition() - kStep);
    }

    if (angular_z > 0.0f) {
      extender_.setTargetPosition(extender_.getCurrentPosition() + kStep);
    }
    else if (angular_z < 0.0f) {
      extender_.setTargetPosition(extender_.getCurrentPosition() - kStep);
    }
  }

  void StepperMotor::handleStepPosMessage(const char* data) {
    // Parse position message: "elevator:<value>,extender:<value>"
    float elevator_pos = -1.0f;
    float extender_pos = -1.0f;
    bool has_elevator = false;
    bool has_extender = false;

    if (data) {
      const char* elevator_start = strstr(data, "elevator:");
      if (elevator_start) {
        elevator_start += strlen("elevator:");
        elevator_pos = strtof(elevator_start, nullptr);
        has_elevator = true;
      }

      const char* extender_start = strstr(data, "extender:");
      if (extender_start) {
        extender_start += strlen("extender:");
        extender_pos = strtof(extender_start, nullptr);
        has_extender = true;
      }
    }

    // Set target positions if valid
    if (has_elevator) {
      elevator_.setTargetPosition(elevator_pos);
    }
    if (has_extender) {
      extender_.setTargetPosition(extender_pos);
    }

    // Send diagnostic confirmation
    char msg[128];
    snprintf(msg, sizeof(msg), "STEPPOS set: elev=%s%.3f ext=%s%.3f",
      has_elevator ? "" : "unchanged,", has_elevator ? elevator_pos : 0.0f,
      has_extender ? "" : "unchanged,", has_extender ? extender_pos : 0.0f);
    serial_.sendDiagnosticMessage("INFO", name(), msg);
  }

  void StepperMotor::handleStepHomeMessage(const char* data) {
    (void)data;  // Unused parameter
    
    // Start non-blocking homing: retract first (safer), then lower
    extender_.startHoming();  // Retract gripper to in position
    elevator_.startHoming();  // Lower elevator to bottom (will wait for extender)
    
    // Send diagnostic confirmation that homing started
    serial_.sendDiagnosticMessage("INFO", name(), "Homing initiated");
  }

  void StepperMotor::handleStepStatusMessage(const char* data) {
    (void)data;  // Unused parameter
    sendStatusMessage();
  }

  void StepperMotor::sendStatusMessage() {
    // Get current positions and limit states
    float elev_pos = elevator_.getCurrentPosition();
    float ext_pos = extender_.getCurrentPosition();
    bool elev_at_bottom = elevator_.atDownLimit();
    bool elev_at_top = elevator_.atUpLimit();
    bool ext_at_in = extender_.atDownLimit();
    bool ext_at_out = extender_.atUpLimit();

    // Determine limit state strings
    const char* elev_lim = elev_at_top ? "top" : (elev_at_bottom ? "bottom" : "none");
    const char* ext_lim = ext_at_out ? "out" : (ext_at_in ? "in" : "none");

    // Format status message as JSON
    // Max positions measured from hardware: elev=0.8999m, ext=0.3418m
    char payload[256];
    snprintf(payload, sizeof(payload),
      "{\"elev_pos\":%.4f,\"ext_pos\":%.4f,\"elev_lim\":\"%s\",\"ext_lim\":\"%s\",\"elev_max\":%.4f,\"ext_max\":%.4f}",
      elev_pos, ext_pos, elev_lim, ext_lim, 0.8999f, 0.3418f);

    serial_.sendMessage("STEPPERSTAT", payload);
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
    
    // Check limit switches and correct position if at limit
    if ((direction == kUp) && atUpLimit()) {
      current_position_m_ = position_max_up_m_;  // Correct position to max
      return;
    }
    if ((direction == kDown) && atDownLimit()) {
      current_position_m_ = position_min_down_m_;  // Correct position to min (0.0)
      return;
    }

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

  void StepperMotor::Motor::startHoming() {
    homing_in_progress_ = true;
    pending_movement_command_ = false;  // Cancel any pending moves
  }

  bool StepperMotor::Motor::continueHoming() {
    if (!homing_in_progress_) return true;  // Already homed
    
    if (atDownLimit()) {
      // Homing complete
      current_position_m_ = position_min_down_m_;
      homing_in_progress_ = false;
      return true;
    }
    
    // Take one step toward home
    stepPulse(kDown);
    return false;  // Still homing
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
    
    // Rate limit: ensure at least 1000us (1ms) between step pulses
    // stepPulse internally takes 1000us (500us high + 500us low), but
    // we need to gate calls to this function to prevent rapid-fire execution
    uint32_t now = micros();
    if (now - last_step_time_us_ < 1000) {
      return;  // Too soon, skip this iteration
    }
    
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

}  // namespace sigyn_teensy

