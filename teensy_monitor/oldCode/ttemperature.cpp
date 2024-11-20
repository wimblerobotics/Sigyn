#include "ttemperature.h"

#include <stdint.h>

#include "Arduino.h"

int16_t TTemperature::getValueTenthsC(TEMPERATURE device) {
  switch (device) {
    case LEFT:
      return g_analog0Value;

    case RIGHT:
      return g_analog1Value;

    default:
      return -1;
  }
}

void TTemperature::loop() {
  int raw = analogRead(ANALOG_0_PIN);
  float tempMv = (raw * 3250 / 1024.0) - 55.0;  // TMP36 temperature conversion.
  g_analog0Value = (tempMv - 500);

  raw = analogRead(ANALOG_1_PIN);
  tempMv = (raw * 3250 / 1024.0) - 55.0;
  g_analog1Value = (tempMv - 500);
}

void TTemperature::setup() { analogReadResolution(10); }

TTemperature::TTemperature() {}

TTemperature& TTemperature::singleton() {
  if (!g_singleton) {
    g_singleton = new TTemperature();
  }

  return *g_singleton;
}

int16_t TTemperature::g_analog0Value = 0;
int16_t TTemperature::g_analog1Value = 0;

TTemperature* TTemperature::g_singleton = nullptr;