#include "battery.h"

BatteryModule::BatteryModule() : TModule(TModule::kBattery) {
  last_battery_send_time = 0;
  battery_send_interval = 1000; // Send battery state every 1000ms
  for (size_t device = 0; device < 1; device++) {
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      g_averages_[device][reading] = 0.0;
    }
  }

  g_next_average_index_ = 0;
}

void BatteryModule::setup() {
  // Serial.println("Battery module initialized");
}

void BatteryModule::loop() {
  unsigned long current_time = millis();

  // Send battery state at configured rate
  if (current_time - last_battery_send_time >= battery_send_interval) {
    readBatteryValues();
    sendBatteryState();
    last_battery_send_time = current_time;
  }
}

void BatteryModule::readBatteryValues() {
  float raw = analogRead(kAnalog1Pin);
  float battery_v = raw * 0.08845 * 0.91;
  g_averages_[0][g_next_average_index_] = battery_v;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage_) {
    g_next_average_index_ = 0;
  }
}

void BatteryModule::sendBatteryState() {
  static uint32_t start_time_ms = millis();
  uint32_t now_ms = millis();

  if ((now_ms - start_time_ms) > 500) {
    float voltage_sum = 0;
    float voltage = 0.0;
    float current = 0.0;
    float charge = 0.0;
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      voltage_sum += g_averages_[0][reading];
    }

    Serial.print("BATTERY:");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print(current);
    Serial.print(",");
    Serial.println(charge);
  }
}

BatteryModule &BatteryModule::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new BatteryModule();
  }
  return *g_singleton_;
}

float BatteryModule::g_averages_[1][kNumberReadingsToAverage_];
size_t BatteryModule::g_next_average_index_ = 0;
BatteryModule *BatteryModule::g_singleton_ = nullptr;