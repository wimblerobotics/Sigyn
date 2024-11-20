#include <Wire.h>
#include <stdint.h>

#include "Watchdog_t4.h"
#include "talarm.h"
#include "talert.h"
#include "tmodule.h"
#include "tmotor_current.h"
#include "tpanel_selector.h"
#include "tpower_panel.h"
#include "tproximity_panel.h"
#include "trelay.h"
#include "troboclaw.h"
// #include "tros_client.h"
#include "tsd.h"
#include "tserver.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

// Initialize all TModule instances in any required order.
TAlarm& alarm = TAlarm::singleton();
TAlert& alert = TAlert::singleton();
TMotorCurrent& motorCurrent = TMotorCurrent::singleton();
TPanelSelector& panelSelector = TPanelSelector::singleton();
TRelay& relay = TRelay::singleton();
TRoboClaw& roboclaw = TRoboClaw::singleton();
// TRosClient& rosClient = TRosClient::singleton();
// ###TSd& sd = TSd::singleton();
TServer& server = TServer::singleton();
TSonar& sonar = TSonar::singleton();
TTemperature& temperature = TTemperature::singleton();
TTimeOfFlight& timeOfFlight = TTimeOfFlight::singleton();

WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  Serial.begin(38400);
  while (!Serial && (millis() <= 1000))
    ;

  WDT_timings_t config;
  config.window = 1;       // Minimum time (ms) between watchdog feed() calls.
  config.timeout = 20000;  // Maximum time (ms) between watchdog feed() calls.
  config.callback = watchdogTimeout;
  TModule::doSetup();
  wdt.begin(config);
}

void loop() {
  static int counter = 0;
  static const int STAT_LOOPS = 1'000;
  static uint32_t start = micros();
  TModule::doLoop();
  wdt.feed();
  counter++;
  if ((counter % STAT_LOOPS) == 0) {
    float durationMs = ((micros() * 1.0) - start) / 1000.0;
    float avgDurationMs = durationMs / STAT_LOOPS;
    Serial.print("MONITOR avg loop duration: ");
    Serial.print(avgDurationMs);
    Serial.print(" ms, fps: ");
    Serial.println(1000 / avgDurationMs);
    start = micros();
    counter = 0;
  }
}
