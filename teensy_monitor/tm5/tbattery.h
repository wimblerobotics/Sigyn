#pragma once

#include <stdint.h>

#include "tmodule.h"

class TBattery : TModule {
 public:

  // Singleton constructor.
  static TBattery& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "Batt"; }

  // From TModule.
  void setup();

 private:
  // GPIO addresses of temperature sensors.
  enum { kAnalog0Pin = 26, kAnalog1Pin = 27 };

  // Private constructor.
  TBattery();

  // Last battery readings.

  static const uint8_t kNumberReadingsToAverage_ = 50;
  static float g_averages_[1][kNumberReadingsToAverage_];
  static size_t g_next_average_index_;

  // Singleton instance.
  static TBattery* g_singleton_;
};