#include "tbattery.h"

#include <stdint.h>

#include "Arduino.h"
#include "tmicro_ros.h"

void TBattery::loop() {
  static uint32_t start_time_ms = millis();

  float raw = analogRead(kAnalog1Pin);
  float battery_v = raw * 0.08845 * 0.91;
  g_averages_[0][g_next_average_index_] = battery_v;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage_) {
    g_next_average_index_ = 0;
  }

  uint32_t now_ms = millis();
  if ((now_ms - start_time_ms) > 500) {
    float voltage_sum = 0;
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      voltage_sum += g_averages_[0][reading];
    }

    TMicroRos::singleton().PublishBattery(
        "main_battery", voltage_sum / kNumberReadingsToAverage_);
    start_time_ms = now_ms;
  }
}

void TBattery::setup() { analogReadResolution(10); }

TBattery::TBattery() : TModule(TModule::kBattery) {
  // Code throughout originally handled more than one sensor, so you'll see
  // g_averages_ is a 2D array.
  for (size_t device = 0; device < 1; device++) {
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      g_averages_[device][reading] = 0.0;
    }
  }

  g_next_average_index_ = 0;
}

TBattery& TBattery::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TBattery();
  }

  return *g_singleton_;
}

float TBattery::g_averages_[1][kNumberReadingsToAverage_];
size_t TBattery::g_next_average_index_ = 0;

TBattery* TBattery::g_singleton_ = nullptr;