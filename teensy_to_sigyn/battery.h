#ifndef BATTERY_H
#define BATTERY_H

#include "tmodule.h"
#include <Arduino.h>

class BatteryModule : TModule {
public:
  // Singleton constructor.
  static BatteryModule &singleton();

protected:
  // From TModule.
  void loop();

  // From TModule.
  const char *name() { return "Batt"; }

  // From TModule.
  void setup();

private:
  // GPIO addresses of the analog pins.
  enum { kAnalog0Pin = 26, kAnalog1Pin = 27 };

  BatteryModule();
  void sendBatteryState();
  void readBatteryValues();
  unsigned long last_battery_send_time;
  unsigned long battery_send_interval; // milliseconds
  static const uint8_t kNumberReadingsToAverage_ = 50;
  static float g_averages_[1][kNumberReadingsToAverage_];
  static size_t g_next_average_index_;

  // Singleton instance.
  static BatteryModule *g_singleton_;
};

#endif
