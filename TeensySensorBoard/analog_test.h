#pragma once
#include <Arduino.h>
#include "test_module.h"

class TAnalogTest : TTestModule{
 public:
  static TAnalogTest& singleton();

  void setup() override {}
//   void loop();

  int16_t getValueTenthsC(uint8_t device);

 private:
  TAnalogTest();
  static TAnalogTest* g_singleton;

  static int16_t g_analog0Value;
  static int16_t g_analog1Value;
};