#pragma once

#include "config.h"
#include "module.h"

class BatteryModule : public Module {
 public:
  static BatteryModule& singleton();

  float getVoltage() const;

 protected:
  void loop() override;
  const char* name() override { return "MBAT"; }
  void setup() override;

 private:
  BatteryModule();
  float calculatePercentage(float voltage);
  void sendBatteryState();

  bool isUnsafe() override;
  void resetSafetyFlags() override;

  static const uint8_t kNumberReadingsToAverage_ = 50;
  static const uint8_t kNumberOfBatteries =
      1;  // Currently only one battery supported
  static float g_averages_[kNumberOfBatteries][kNumberReadingsToAverage_];
  static size_t g_next_average_index_;
  unsigned long
      total_battery_readings_;  // Total number of battery readings taken.

  // int calculatePercentage(float voltage);
  static BatteryModule* g_instance_;
};
