#pragma once

#include <Adafruit_INA260.h>
#include <stdint.h>

#include "tmodule.h"

class TMotorCurrent : TModule {
 public:
  // Which motor.
  typedef enum MOTOR { LEFT, RIGHT, NUMBER_MOTORS } MOTOR;

  // Get the average motor current value.
  int getValueMa(MOTOR index);

  // From TModule.‰
  void loop();

  // From TModule.
  const char* name() { return "TMotorCurrent"; }

  // From TModule.
  void setup();

  // Singleton constructor.
  static TMotorCurrent& singleton();

 private:
  // GPIO pins for current sensors.
  typedef enum {
    LEFT_MOTOR__ADDRESS = 0x40,
    RIGHT_MOTOR_ADDRESS = 0x41
  } MOTOR_PINS;

  // Private constructor.
  TMotorCurrent();

  // Singleton instance.
  static TMotorCurrent* g_singleton;

  // Number of samples to go into an average motor current.
  static const uint8_t AVERAGE_COUNT = 20;

  // Motor current sensor instances.
  static Adafruit_INA260 g_leftINA260;
  static Adafruit_INA260 g_rightINA260;

  // Indicate if motor current sensor was detected.
  static bool g_haveLeftMotorSensor;
  static bool g_haveRightMotorSensor;

  // Reported value for motor current sensors.
  static float g_leftMotorCurrentMa;
  static float g_rightMotorCurrentMa;

  // Index for insertion of next motor current sample value.
  static uint8_t g_nextMotorCurrentIndex;

  // Rolling window of motor current sensor values by sensor.
  static float g_leftMotorCurrentMaReadings[AVERAGE_COUNT];
  static float g_rightMotorCurrentMaReadings[AVERAGE_COUNT];
};