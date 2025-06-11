#include "BatteryModule.h"

#include "Arduino.h"        // For analogRead, millis
#include "SerialManager.h"  // For sending diagnostic messages

BatteryModule::BatteryModule()
    : TModule(),
      total_battery_readings_(0)
//   current_voltage_(0.0f),
//   current_percentage_(0),
//   low_battery_warning_(false),
//   critical_battery_warning_(false),
//   last_read_time_ms_(0) {}
{
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      g_averages_[device][reading] = 0.0;
    }
  }
}

void BatteryModule::setup() { pinMode(MAIN_BATTERY_PIN, INPUT); }

void BatteryModule::loop() {
  static uint32_t last_message_send_time_ms_ = millis();
  unsigned long current_time_ms = millis();
  static uint32_t last_battery_read_time_ms_ = millis();
  if ((current_time_ms - last_battery_read_time_ms_) >
      MAIN_BATTERY_READ_INTERVAL_MS) {
    float raw = analogRead(MAIN_BATTERY_PIN);
    float battery_v = raw * 0.08845 * 0.9168;
    g_averages_[0][g_next_average_index_] = battery_v;
    total_battery_readings_++;
    g_next_average_index_ += 1;
    if (g_next_average_index_ >= kNumberReadingsToAverage_) {
      g_next_average_index_ = 0;
    }
    last_battery_read_time_ms_ = current_time_ms;
  }

  // Read battery status periodically, not every loop, to save processing
  // For battery, reading can be less frequent, e.g., every 500ms
  if ((current_time_ms - last_message_send_time_ms_) >
      MAIN_BATTERY_REPORT_INTERVAL_MS) {
    sendBatteryState();
    last_message_send_time_ms_ = current_time_ms;
  }
}

// void BatteryModule::updateBatteryStatus() {
//     int adc_value = analogRead(MAIN_BATTERY_PIN);
//     current_voltage_ = (adc_value / MAIN_BATTERY_ADC_MAX_VALUE) *
//     MAIN_BATTERY_ADC_REF_VOLTAGE * MAIN_BATTERY_VOLTAGE_DIVIDER_RATIO;
//     current_percentage_ = calculatePercentage(current_voltage_);

//     if (current_voltage_ < MAIN_BATTERY_CRITICAL_VOLTAGE_THRESHOLD) {
//         if (!critical_battery_warning_) {
//              serial_manager.SendDiagnosticMessage("CRITICAL LOW BATTERY -
//              E-Stop imminent!");
//         }
//         critical_battery_warning_ = true;
//         low_battery_warning_ = true; // Critical implies low
//     } else if (current_voltage_ < MAIN_BATTERY_LOW_VOLTAGE_THRESHOLD) {
//         if (!low_battery_warning_) {
//             serial_manager.SendDiagnosticMessage("Warning: Low Battery.");
//         }
//         low_battery_warning_ = true;
//         critical_battery_warning_ = false; // Not critical yet
//     } else {
//         low_battery_warning_ = false;
//         critical_battery_warning_ = false;
//     }
// }

bool BatteryModule::IsUnsafe() {
  return getVoltage() < MAIN_BATTERY_CRITICAL_VOLTAGE_THRESHOLD;
}

void BatteryModule::ResetSafetyFlags() {
  // Safety flags are reset based on voltage readings in
}

void BatteryModule::sendBatteryState() {
  SerialManager::singleton().SendBatteryStatus(
      getVoltage(), calculatePercentage(getVoltage()));
}

float BatteryModule::getVoltage() const {
  float voltage_sum = 0;
  size_t number_readings_to_average =
      total_battery_readings_ < kNumberReadingsToAverage_
          ? total_battery_readings_
          : kNumberReadingsToAverage_;
  for (size_t reading = 0; reading < number_readings_to_average; reading++) {
    voltage_sum += g_averages_[0][reading];
  }

  float average_voltage = number_readings_to_average > 0
                              ? voltage_sum / number_readings_to_average
                              : 0.0f;
  return average_voltage;
}

// int BatteryModule::GetPercentage() const {
//     return current_percentage_;
// }

// Simple linear interpolation for battery percentage
float BatteryModule::calculatePercentage(float voltage) {
  if (voltage >= MAIN_BATTERY_MAX_VOLTAGE) return 100;
  if (voltage <= MAIN_BATTERY_MIN_VOLTAGE) return 0;

  return (voltage - MAIN_BATTERY_MIN_VOLTAGE) / MAIN_BATTERY_LIPO_CELLS * 100.0f;
}

BatteryModule& BatteryModule::singleton() {
  if (!g_instance_) {
    g_instance_ = new BatteryModule();
  }
  return *g_instance_;
}

BatteryModule* BatteryModule::g_instance_ = nullptr;
float BatteryModule::g_averages_[BatteryModule::kNumberOfBatteries]
                                [BatteryModule::kNumberReadingsToAverage_] = {};
size_t BatteryModule::g_next_average_index_ = 0;