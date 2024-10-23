#include <micro_ros_arduino.h>

#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdint.h>

#include "tconfiguration.h"
// #include "Watchdog_t4.h"
#include "tmicro_ros.h"
#include "tmodule.h"
#if USE_TSD
#include "tsd.h"
#endif


// Initialize all TModule instances in any required order.
#if USE_TSD
// TSd& sd = TSd::singleton();
#endif

TMicroRos& micro_ros = TMicroRos::singleton();

// WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  // Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  pinMode(13, OUTPUT);
  // Serial.begin(9600);
  // while (!Serial) {  }
  // Serial.println("setup()");

  //  WDT_timings_t config;
  //  config.window = 1;       // Minimum time (ms) betwee//n watchdog feed()
  //  calls. config.timeout = 20000;  // Maximum time (ms) between watchdog
  //  feed() calls. config.callback = watchdogTimeout;

  TSd::singleton().log("elevator setup() complete");
  TModule::DoSetup();

  //  wdt.begin(config);
}

void loop() {
  TModule::DoLoop();
  //  wdt.feed();
}