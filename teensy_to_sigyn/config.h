#pragma once
#include <stdint.h>

// Motor specs, deviced from the manufacturer's datasheet and the hardware specs of the robot.
typedef struct MotorSpecs {
  int16_t max_rpm;                // Motor's max RPM.
  float motor_operating_voltage;  // Motor's operating voltage (used to calculate max RPM).
  float motor_power_max_voltage;  // Max voltage of the motor's power source(used to calculate max
                                  // RPM).
  uint32_t quad_pulses_per_revolution;
} MotorSpecs;

typedef struct RobotSpecs {
  float wheel_diameter;     // Wheel's diameter in meters.
  float wheels_separation;  // Distance between left and right wheels in meters.
} RobotSpecs;

static const MotorSpecs SuperDroid_1831 = {
    .max_rpm = 122, .motor_operating_voltage = 24, .motor_power_max_voltage = 24, .quad_pulses_per_revolution = 1000};

static const RobotSpecs Sigyn_specs = {.wheel_diameter = 0.102224144529039, .wheels_separation = 0.3906};
