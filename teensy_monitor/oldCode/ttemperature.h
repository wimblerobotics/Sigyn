#pragma once

#include <stdint.h>

#include "tmodule.h"

class TTemperature : TModule {
 public:
  // Which temperature sensor.
  typedef enum TEMPERATURE {
    LEFT,
    RIGHT,
    NUMBER_TEMPERATURES  // Number of temperature sensors.
  } TEMPERATURE;

  int16_t getValueTenthsC(TEMPERATURE device);

  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "TTemperature"; }

  // From TModule.
  void setup();

  // Singleton constructor.
  static TTemperature& singleton();

 private:
  // GPIO addresses of temperature sensors.
  enum { ANALOG_0_PIN = 26, ANALOG_1_PIN = 27 };

  // Private constructor.
  TTemperature();

  // Last temperature sensor readings.
  static int16_t g_analog0Value;
  static int16_t g_analog1Value;

  // Singleton instance.
  static TTemperature* g_singleton;
};