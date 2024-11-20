#include "tmodule.h"

#include <Arduino.h>
#include <stdint.h>

#include "tsd.h"

#define DO_TIMING true

TModule::TModule() {
  if (g_nextModuleNumber < NUMBER_MODULES) {
    g_allModules[g_nextModuleNumber++] = this;
  }
}

void TModule::doLoop() {
  uint32_t start;
  static uint32_t statTimingStart = micros();
  for (int i = 0; i < g_nextModuleNumber; i++) {
    if (DO_TIMING) {
      start = micros();
    }

    g_allModules[i]->loop();

    if (DO_TIMING) {
      float duration = ((micros() * 1.0) - start) / 1000.0;
      g_readings[i][SUM] += duration;
      if (duration < g_readings[i][MIN]) {
        g_readings[i][MIN] = duration;
      }

      if (duration > g_readings[i][MAX]) {
        g_readings[i][MAX] = duration;
      }

      // Serial.print("fn: ");
      // Serial.print(g_allModules[i]->name());
      // Serial.print(", duration (ms): ");
      // Serial.println(duration);
    }
  }

  if (DO_TIMING) {
    g_nextReadingNumber++;
    if (g_nextReadingNumber >= NUMBER_READINGS) {
      Serial.print("--- --- --- duration for ");
      Serial.print(NUMBER_READINGS);
      Serial.print(" readings: ");
      Serial.print(((micros() * 1.0) - statTimingStart) / 1000.0);
      Serial.println(" ms");

      for (int i = 0; i < g_nextModuleNumber; i++) {
        Serial.print(g_allModules[i]->name());
        Serial.print(" min: ");
        Serial.print(g_readings[i][MIN]);
        Serial.print(", max: ");
        Serial.print(g_readings[i][MAX]);
        Serial.print(" avg: ");
        Serial.print(g_readings[i][SUM] / g_nextReadingNumber);
        Serial.println();
        String logMessage;
        logMessage += g_allModules[i]->name();
        logMessage += "\tmin\t";
        logMessage += g_readings[i][MIN];
        logMessage += "\tmax\t";
        logMessage += g_readings[i][MAX];
        logMessage += "\tavg\t";
        logMessage += g_readings[i][SUM] / g_nextReadingNumber;
        TSd::singleton().log(logMessage.c_str());
      }

      resetReadings();
      g_nextReadingNumber = 0;
      statTimingStart = micros();
    }
  }
}

void TModule::resetReadings() {
  for (int i = 0; i < NUMBER_MODULES; i++) {
    g_readings[i][MIN] = 10'000'000;
    g_readings[i][MAX] = -10'000'000;
    g_readings[i][SUM] = 0;
  }
}

void TModule::doSetup() {
  for (int i = 0; i < g_nextModuleNumber; i++) {
    g_allModules[i]->setup();
  }
}

// TModule& TModule::singleton() {
//   if (!g_singleton) {
//     g_singleton = new TModule();
//     resetReadings();
//   }

//   return *g_singleton;
// }

TModule* TModule::g_allModules[TModule::NUMBER_MODULES + 1];
uint8_t TModule::g_nextModuleNumber = 0;
int TModule::g_nextReadingNumber = 0;
// TModule* TModule::g_singleton = nullptr;
float TModule::g_readings[TModule::NUMBER_MODULES][TModule::NUMBER_SLOTS];
