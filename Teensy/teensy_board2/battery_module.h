#pragma once

#include <stdint.h>

#include "INA226.h"  // For INA226 battery monitoring.
#include "config.h"
#include "module.h"

class BatteryModule : public Module {
 public:
  static BatteryModule& singleton();

  float getVoltage() const;
  float getCurrent(uint8_t device_index);

 protected:
  void loop() override;
  const char* name() override { return "MBAT"; }
  void setup() override;

 private:
  BatteryModule();
  float calculatePercentage(float voltage);
  void sendBatteryState(uint8_t device_index);

  bool isUnsafe() override;
  void resetSafetyFlags() override;

  void selectSensor(uint8_t sensor_index);
  bool testI2CMultiplexer();

  static const uint8_t kNumberReadingsToAverage_ = 50;
  static const uint8_t kNumberOfBatteries =
      1;  // Currently only one battery supported
  static float g_averages_[kNumberOfBatteries][kNumberReadingsToAverage_];
  static size_t g_next_average_index_;

  static INA226
      g_ina226_[kNumberOfBatteries];  // INA226 instance for battery monitoring

  static uint8_t gINA226_DeviceIndexes_[kNumberOfBatteries];  // Device index in
                                                              // multiplexer

  static const uint8_t INA226_ADDRESS = 0x40;  // Default INA226 I2C address

  // int calculatePercentage(float voltage);
  static BatteryModule* g_instance_;

  bool multiplexer_available_;  // Multiplexer availability
  bool setup_completed_;        // Setup status
  unsigned long
      total_battery_readings_;  // Total number of battery readings taken.
};
