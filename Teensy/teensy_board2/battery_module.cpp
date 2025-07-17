#include "battery_module.h"

#include "Arduino.h"         // For analogRead, millis
#include "INA226.h"          // For INA226 battery monitoring (if used)
#include "config.h"          // For battery configuration constants
#include "serial_manager.h"  // For sending diagnostic messages

// This module is specific to Teensy board 2 and handles battery monitoring
// using not only analog readings but an INA226 for current readings.
BatteryModule::BatteryModule()
    : Module(),
      multiplexer_available_(false),
      setup_completed_(false),
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

  SerialManager::singleton().SendDiagnosticMessage(
      String("[BatteryModule::BatteryModule] Configured for ") +
      String(kNumberOfBatteries) + String(" sensors"));
}

void BatteryModule::setup() {
  if (setup_completed_) {
    return;  // Already completed setup
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "[BatteryModule::setup] Starting sensor initialization...");

  pinMode(MAIN_BATTERY_PIN, INPUT);
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // Set I2C to 400kHz for faster communication

  // Test I2C multiplexer connectivity
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::setup] I2C multiplexer not available, cannot "
        "proceed.");
    return;
  }

  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    selectSensor(gINA226_DeviceIndexes_[device]);
    g_ina226_[device].begin();
    g_ina226_[device].setMaxCurrentShunt(20, 0.002);
    // g_ina226_[device].setAverage(INA226_AVERAGE_16);
    // g_ina226_[device].setMode(INA226_MODE_CONTINUOUS);
  }

  setup_completed_ = true;
}

void BatteryModule::loop() {
  static uint32_t last_message_send_time_ms_ = millis();
  unsigned long current_time_ms = millis();
  static uint32_t last_battery_read_time_ms_ = millis();
  if ((current_time_ms - last_battery_read_time_ms_) >
      MAIN_BATTERY_READ_INTERVAL_MS) {
    float raw = analogRead(MAIN_BATTERY_PIN);
   float battery_v = raw * 0.087393162;
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
    for (size_t device_index = 0; device_index < kNumberOfBatteries;
         device_index++) {
      sendBatteryState(device_index);
    }

    last_message_send_time_ms_ = current_time_ms;
  }
}

float BatteryModule::getCurrent(uint8_t device_index) {
  // Return the current reading from the INA226
  selectSensor(gINA226_DeviceIndexes_[device_index]);
  if (device_index < kNumberOfBatteries) {
    return g_ina226_[device_index].getCurrent();
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::getCurrent] Invalid device index: " +
        String(device_index));
    return 0.0f;  // Return 0 if device is not connected
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

bool BatteryModule::isUnsafe() {
  return getVoltage() < MAIN_BATTERY_CRITICAL_VOLTAGE_THRESHOLD;
}

void BatteryModule::resetSafetyFlags() {
  // Safety flags are reset based on voltage readings in
}

void BatteryModule::sendBatteryState(uint8_t device_index) {
  if (device_index >= kNumberOfBatteries) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::sendBatteryState] Invalid device index: " +
        String(device_index));
    return;
  }

  char message[64];

  selectSensor(gINA226_DeviceIndexes_[device_index]);
  snprintf(message, sizeof(message), "BATTERY:%d,%.2fV%.2f%%,%.2fA",
           device_index, getVoltage(), calculatePercentage(getVoltage()),
           getCurrent(device_index));
  SerialManager::singleton().SendDiagnosticMessage(message);
}

void BatteryModule::selectSensor(uint8_t sensor_index) {
  // Select the appropriate channel on the I2C multiplexer
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << sensor_index);
  Wire.endTransmission();

  // Small delay to allow multiplexer to switch
  delayMicroseconds(100);
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

  return (voltage - MAIN_BATTERY_MIN_VOLTAGE) / MAIN_BATTERY_LIPO_CELLS *
         100.0f;
}

bool BatteryModule::testI2CMultiplexer() {
  // Test if the I2C multiplexer is present and responsive
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer found at address "
        "0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX));
    return true;
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer NOT found at "
        "address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX) + " (error: " + String(error) +
        ")");
    return false;
  }
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
uint8_t BatteryModule::gINA226_DeviceIndexes_[kNumberOfBatteries] = {
    2};  // Device index in multiplexer
INA226 BatteryModule::g_ina226_[kNumberOfBatteries] = {INA226(
    BatteryModule::INA226_ADDRESS)};  // INA226 instance for battery monitoring