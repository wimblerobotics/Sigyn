#pragma once

#include <VL53L0X.h>
#include <stdint.h>

#include "tmodule.h"

class TTimeOfFlight : TModule {
 public:
  typedef enum TIMEOFFLIGHT {
    UPPER_LEFT_FORWARDS,
    UPPER_RIGHT_FORWARDS,
    UPPER_LEFT_SIDEWAYS,
    UPPER_RIGHT_SIDEWAYS,
    LEFT_LEFT_SIDEWAYS,
    LOWER_RIGHT_SIDEWAYS,
    LOWER_LEFT_BACKWARDS,
    LOWER_RIGHT_BACKWARDS,
    NUMBER_TIME_OF_FLIGHT  // Number of time-of-flight devices.
  } TIMEOFFLIGHT;

  // Get the sensed distance for the device. A value of < 0 => no sensor at that
  // index.
  int getValueMm(TIMEOFFLIGHT device);

  // From TModule.‰
  void loop();

  // From TModule.‰
  const char* name() { return "TTimeOfFlight"; }

  // From TModule.‰
  void setup();

  // Singleton constructor.
  static TTimeOfFlight& singleton();

 private:

  // Should motors be put in e-stop if collision is imminent?
  static const bool doStopMotorsOnCollisionThreat = false;

  // Private constructor.
  TTimeOfFlight();

  // Select a time-of-flight device through the multiplexer.
  void selectTimeOfFlightSensor(TIMEOFFLIGHT device);

  // Address of I2C multiplexer for time-of-flight devices.
  static const uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;

  // Last sensed distance for each time-of-flight device.
  static int g_cachedValue[NUMBER_TIME_OF_FLIGHT];

  // Device hardware handle for each time-of-flight device.
  static VL53L0X* g_sensor[NUMBER_TIME_OF_FLIGHT];

  // Singleton instance.
  static TTimeOfFlight* g_singleton;

  // Minimum detection distance before an alert is raised
  static const int ALERT_DISTANCE_MM = 3 * 25.4;
};
