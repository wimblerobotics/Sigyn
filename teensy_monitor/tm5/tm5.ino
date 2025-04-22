#include <micro_ros_arduino.h>
#include <stdint.h>

#include "tbattery.h"
#include "tconfiguration.h"
// #include "Watchdog_t4.h"
#include "tmicro_ros.h"
#include "tmodule.h"
#include "trelay.h"
#include "troboclaw.h"
#if USE_TSD
#include "tsd.h"
#endif

// Initialize all TModule instances in any required order.
TMicroRos& micro_ros = TMicroRos::singleton();
// TProximityPanel& proximityPanel = TProximityPanel::singleton();
TRelay& relay = TRelay::singleton();
TRoboClaw& roboclaw = TRoboClaw::singleton();
TBattery& battery = TBattery::singleton();

// WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  // Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  //  WDT_timings_t config;
  //  config.window = 1;       // Minimum time (ms) betwee//n watchdog feed()
  //  calls. config.timeout = 20000;  // Maximum time (ms) between watchdog
  //  feed() calls. config.callback = watchdogTimeout;

#if USE_TSD
  TSd& sd = TSd::singleton();
  (void)sd;  // Suppress unused variable warning.
#endif
  TModule::DoSetup();
  //  wdt.begin(config);
}

void loop() {
  TModule::DoLoop();
  //  wdt.feed();
}
