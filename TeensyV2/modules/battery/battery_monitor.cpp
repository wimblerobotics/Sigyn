#include "battery_monitor.h"

#include <Arduino.h>
#include <Wire.h>
#include <stddef.h>
#include <stdint.h>

#include <cmath>

#include "INA226.h"
#include "serial_manager.h"

namespace sigyn_teensy {

BatteryMonitor::BatteryMonitor()
    : setup_completed_(false), multiplexer_available_(false) {
  for (size_t i = 0; i < kNumberOfBatteries; i++) {
    voltage_ema_[i] = 0.0f;
    current_ema_[i] = 0.0f;
    state_[i] = BatteryState::UNKNOWN;
    total_readings_[i] = 0;
    ina226_[i] = new INA226(0x40);  // Default address, adjust as needed
  }
}

void BatteryMonitor::setup() {
  pinMode(MAIN_BATTERY_PIN, INPUT);
  pinMode(kI2CMultiplexorEnablePin, OUTPUT);
  digitalWrite(kI2CMultiplexorEnablePin, HIGH);
  Wire.begin();
  Wire.setClock(400000);
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::GetInstance().SendMessage(
        "FATAL", "BatteryMonitor setup failed: I2C multiplexer not available");
    return;
  }

  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    selectSensor(device);
    if (!ina226_[device]->begin()) {
      String message =
          "INA226 sensor initialization failed for device " + String(device);
      SerialManager::GetInstance().SendMessage("FATAL", message.c_str());
      return;
    }
    ina226_[device]->setMaxCurrentShunt(20, 0.002);  // 20A max, 2mΩ shunt
    // if (!ina226_[device]->setMode(INA226_MODE_CONTINUOUS)) {
    //   SerialManager::GetInstance().SendMessage(
    //       "FATAL", "INA226 sensor mode setting failed for device " +
    //                String(device));
    //   return;
    // }

    // Optional: Set averaging and continuous mode for better accuracy
    // ina226_[device]->setAverage(INA226_AVERAGE_16);
    // ina226_[device]->setMode(INA226_MODE_CONTINUOUS);
  }
  setup_completed_ = true;
}

void BatteryMonitor::loop() {
  static uint32_t last_message_send_time_ms_ = millis();
  static uint32_t last_read_time = millis();
  unsigned long now_ms = millis();

  if ((now_ms - last_read_time) > 100) {
    for (size_t battery_idx = 0; battery_idx < kNumberOfBatteries;
         battery_idx++) {
      float voltage = 0.0f;
      float current = 0.0f;
      if (battery_idx == 0) {
        float raw = analogRead(MAIN_BATTERY_PIN);
        voltage = raw * k36VAnalogToVoltageConversion;
        selectSensor(battery_idx);
        current = ina226_[battery_idx]->getCurrent();
      }
      if (total_readings_[battery_idx] == 0) {
        voltage_ema_[battery_idx] = voltage;
        current_ema_[battery_idx] = current;
      } else {
        voltage_ema_[battery_idx] = updateExponentialAverage(
            voltage_ema_[battery_idx], voltage, kDefaultVoltageAlpha);
        current_ema_[battery_idx] = updateExponentialAverage(
            current_ema_[battery_idx], current, kDefaultCurrentAlpha);
      }

      UpdateBatteryState(battery_idx);
      total_readings_[battery_idx]++;
    }

    last_read_time = now_ms;
  }

  // Status reporting phase - send diagnostic messages
  if ((now_ms - last_message_send_time_ms_) > MAIN_BATTERY_REPORT_INTERVAL_MS) {
    for (size_t device_index = 0; device_index < kNumberOfBatteries;
         device_index++) {
      SendStatusMessage(device_index);
    }
    last_message_send_time_ms_ = now_ms;
  }
}

float BatteryMonitor::EstimateChargePercentage(float voltage) const {
  // Simple linear approximation for Li-ion battery pack
  // Adjust these values based on actual battery characteristics
  constexpr float empty_voltage = 32.0f;  // 0% charge
  constexpr float full_voltage = 42.0f;   // 100% charge

  if (voltage < empty_voltage) {
    return 0.0f;
  } else if (voltage > full_voltage) {
    return 1.0f;
  } else {
    return (voltage - empty_voltage) / (full_voltage - empty_voltage);
  }
}

float BatteryMonitor::getVoltage(size_t idx) const { return voltage_ema_[idx]; }

float BatteryMonitor::getCurrent(size_t idx) const { return current_ema_[idx]; }

void BatteryMonitor::SendStatusMessage(size_t idx) {
  // Create status message with all battery data
  String message = "id=" + String(idx);

  if (!isnan(getVoltage(idx))) {
    message += ",v=" + String(voltage_ema_[idx], 2);
  }

  if (!isnan(getCurrent(idx))) {
    message += ",c=" + String(current_ema_[idx], 3);
  }

  float charge_percentage = EstimateChargePercentage(getVoltage(idx));
  if (!isnan(charge_percentage)) {
    message += ",pct=" + String(charge_percentage, 2);
  }

  // Add state information
  message += ",state=";
  switch (state_[idx]) {
    case BatteryState::UNKNOWN:
      message += "UNKNOWN";
      break;
    case BatteryState::CHARGING:
      message += "CHARGING";
      break;
    case BatteryState::DISCHARGING:
      message += "DISCHARGING";
      break;
    case BatteryState::CRITICAL:
      message += "CRITICAL";
      break;
    case BatteryState::WARNING:
      message += "WARNING";
      break;
    case BatteryState::NORMAL:
      message += "NORMAL";
      break;
  }

  SerialManager::GetInstance().SendMessage("BATT", message.c_str());
}

float BatteryMonitor::updateExponentialAverage(float current_avg,
                                               float new_value, float alpha) {
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  return alpha * new_value + (1.0f - alpha) * current_avg;
}

void BatteryMonitor::UpdateBatteryState(size_t idx) {
  if (isnan(getVoltage(idx)) || isnan(getCurrent(idx))) {
    state_[idx] = BatteryState::UNKNOWN;
    return;
  }

  // Determine state based on voltage and current
  if (getVoltage(idx) < config_.critical_low_voltage) {
    state_[idx] = BatteryState::CRITICAL;
  } else if (getVoltage(idx) < config_.warning_low_voltage) {
    state_[idx] = BatteryState::WARNING;
  } else if (!isnan(getCurrent(idx))) {
    if (getCurrent(idx) < -0.1f) {  // Negative current indicates charging
      state_[idx] = BatteryState::CHARGING;
    } else if (getCurrent(idx) >
               0.1f) {  // Positive current indicates discharging
      state_[idx] = BatteryState::DISCHARGING;
    } else {
      state_[idx] = BatteryState::NORMAL;  // Near zero current
    }
  } else {
    state_[idx] = BatteryState::NORMAL;  // Default if current unavailable
  }
}

void BatteryMonitor::selectSensor(size_t battery_idx) const {
  Wire.beginTransmission(0x70);  // I2C_MULTIPLEXER_ADDRESS, adjust as needed
  Wire.write(1 << gINA226_DeviceIndexes_[battery_idx] );  // Select channel
  Wire.endTransmission();
  delayMicroseconds(100);
}

bool BatteryMonitor::testI2CMultiplexer() {
  Wire.beginTransmission(0x70);  // I2C_MULTIPLEXER_ADDRESS, adjust as needed
  uint8_t error = Wire.endTransmission();
  return (error == 0);
}

BatteryMonitor& BatteryMonitor::GetInstance() {
  static BatteryMonitor instance;
  return instance;
}

void BatteryMonitor::Configure(const BatteryConfig& config) {
  config_ = config;
}

const char* BatteryMonitor::name() const { return "BatteryMonitor"; }

// Hardware interface arrays
uint8_t BatteryMonitor::gINA226_DeviceIndexes_[kNumberOfBatteries] = {
    2  // Main battery on multiplexer channel 2
};

}  // namespace sigyn_teensy