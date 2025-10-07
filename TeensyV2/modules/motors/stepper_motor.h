// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once

#include <Arduino.h>

#include "../../common/core/module.h"
#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

  /**
   * StepperMotor module controls two steppers: Elevator and Extender.
   * Ported and simplified from legacy MotorModule to fit Module framework.
   */
  class StepperMotor : public Module {
  public:
    // Access singleton
    static StepperMotor& getInstance();

    // Module interface
    void setup() override;
    void loop() override;
    const char* name() const override { return "StepperMotor"; }

    // Safety hooks
    bool isUnsafe() override { return false; }
    void resetSafetyFlags() override {}

    // Command handling
    void handleTwistMessage(const String& data);

  private:
    StepperMotor();

    // Inner motor class
    class Motor {
    public:
      enum Direction { kUp, kDown };

      // Pin map (matches legacy wiring)
      enum Pins : uint8_t {
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
        float travel_m_per_pulse, bool reverse_travel);

      // operations
      void home();
      void setTargetPosition(float target_position_m);
      void moveByDelta(float delta_m);
      void continueOutstandingMovementRequests();

      // helpers
      bool atDownLimit();
      bool atUpLimit();

      float getCurrentPosition() const { return current_position_m_; }

    private:
      void stepPulse(Direction direction);

      float current_position_m_ = 0.0f;
      bool pending_action_ = false;
      bool pending_movement_command_ = false;
      const uint8_t pin_down_limit_switch_;
      const uint8_t pin_step_direction_;
      const uint8_t pin_step_pulse_;
      const uint8_t pin_up_limit_switch_;
      const float position_max_up_m_;
      const float position_min_down_m_;
      int32_t remaining_pulses_ = 0;
      bool reverse_travel_ = false;
      const float travel_m_per_pulse_;
    };

    // Factory
    Motor* createMotor(bool elevator);

    // Instances
    Motor* elevator_ = nullptr;
    Motor* extender_ = nullptr;

    SerialManager& serial_;
  };

}  // namespace sigyn_teensy

