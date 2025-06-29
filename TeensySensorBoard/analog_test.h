#pragma once
#include <Arduino.h>

#include "test_module.h"

class TAnalogTest : public TTestModule {
 public:
  static TAnalogTest& singleton();

  const char* deviceDescription(uint8_t deviceIndex);

  const char* deviceName(uint8_t deviceIndex);

  const int numberOfDevices();

  const int numberOfTestsPerDevice();

  bool runTest(uint8_t testDevice, uint8_t testIndex);

  const char* testName(uint8_t testIndex);

  int16_t value(uint8_t deviceIndex);

 protected:
  TAnalogTest();
  static TAnalogTest* g_singleton;

  void setup();

 private:
  // GPIO addresses of analog sensors.
  enum {
    ANALOG_0_PIN = 24,
    ANALOG_1_PIN = 25,
    ANALOG_2_PIN = 26,
    ANALOG_3_PIN = 27,
    ANALOG_4_PIN = 38,
    ANALOG_5_PIN = 39,
    ANALOG_6_PIN = 40,
    ANALOG_7_PIN = 41
  };
};