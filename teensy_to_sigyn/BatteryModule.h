#pragma once

#include "TModule.h"
#include "config.h"

class BatteryModule : public TModule {
 public:
  static BatteryModule& singleton();

  bool IsUnsafe() override;
  void ResetSafetyFlags() override;

  float getVoltage() const;

 protected:
  void loop() override;
  const char* name() override { return "MBAT"; }
  void setup() override;

 private:
  BatteryModule();
  float calculatePercentage(float voltage);
  void sendBatteryState();

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
