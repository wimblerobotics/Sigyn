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
TMicroRos& micro_ros = TMicroRos::singleton();
#if USE_TSD
TSd& sd = TSd::singleton();
#endif

// WDT_T4<WDT3> wdt;

// Method to handle watchdog timeout.
void watchdogTimeout() {
  // Serial.println("WATCHDOG TIMEOUT, 255 CYCLES TILL RESET...");
}

void setup() {
  Wire.begin();
  pinMode(13, OUTPUT);
  

  //  WDT_timings_t config;
  //  config.window = 1;       // Minimum time (ms) betwee//n watchdog feed()
  //  calls. config.timeout = 20000;  // Maximum time (ms) between watchdog
  //  feed() calls. config.callback = watchdogTimeout;

  TModule::DoSetup();

  //  wdt.begin(config);
}

void loop() {
  TModule::DoLoop();
  //  wdt.feed();
}



//   enum {
//     kPinEcho0 = 35,
//     kPinTrigger0 = 34,
//     kPinEcho1 = 37,
//     kPinTrigger1 = 36,
//     kPinEcho2 = 41,
//     kPinTrigger2 = 40,
//     kPinEcho3 = 15,
//     kPinTrigger3 = 14
//   };

// void setup() {
//   pinMode(kPinEcho2, INPUT);
//   pinMode(kPinTrigger2, INPUT);
//   Serial.begin(9600);
// }

// void loop() {
//   bool senseTop = digitalRead(kPinEcho2);
//   bool senseBottom = digitalRead(kPinTrigger2);
//   Serial.print("Sense: ^");
//   Serial.print(senseTop);
//   Serial.print(" v");
//   Serial.println(senseBottom);
//   delay(500);
// }

// boolean setdir = LOW; // Set Direction
// int pd = 500;       // Pulse Delay period

  
// void setup() {
//   pinMode(kPinEcho3, OUTPUT);
//   pinMode(kPinTrigger3, OUTPUT);
// }

// void loop() {
//   static int counter = 0;
//   if (counter++ > 1000) {
//     setdir = !setdir;
//     counter = 0;
//   }

//   digitalWrite(kPinTrigger3,setdir);
//   digitalWrite(kPinEcho3,HIGH);
//   delayMicroseconds(pd);
//   digitalWrite(kPinEcho3,LOW);
//   delayMicroseconds(pd);
// }