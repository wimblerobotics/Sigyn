#include "battery.h"

BatteryModule::BatteryModule()
    : TModule(TModule::kBattery),
      last_battery_send_time(0),
      battery_send_interval_(1000),  // Default to sending every 1000ms
      total_battery_readings_(0) {
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

  static unsigned long last_battery_read_time = millis();
  if (current_time - last_battery_read_time >= 100) {
    // Read battery values every second
    readBatteryValues();
    last_battery_read_time = current_time;
  }

  // Send battery state at configured rate
  if (current_time - last_battery_send_time >= battery_send_interval_) {
    sendBatteryState();
    last_battery_send_time = current_time;
  }
}

void BatteryModule::readBatteryValues() {
  float raw = analogRead(kAnalog1Pin);
  float battery_v = raw * 0.08845 * 0.91;
  g_averages_[0][g_next_average_index_] = battery_v;
  total_battery_readings_++;

  g_next_average_index_ += 1;
  if (g_next_average_index_ >= kNumberReadingsToAverage_) {
    g_next_average_index_ = 0;
  }
}

void BatteryModule::sendBatteryState() {
  float voltage_sum = 0;
  float voltage = 0.0;
  float current = 0.0;
  float charge = 0.0;
  size_t number_readings_to_averae =
      total_battery_readings_ < kNumberReadingsToAverage_
          ? total_battery_readings_
          : kNumberReadingsToAverage_;
  for (size_t reading = 0; reading < number_readings_to_averae; reading++) {
    voltage_sum += g_averages_[0][reading];
  }

  Serial.print("BATTERY:");
  Serial.print(number_readings_to_averae > 0 ? voltage_sum / number_readings_to_averae : 0);  
  Serial.print(",");
  Serial.print(current);
  Serial.print(",");
  Serial.println(charge);
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