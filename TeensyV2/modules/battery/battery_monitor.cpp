// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file battery_monitor.cpp
 * @brief Implementation of comprehensive battery monitoring system
 */

#include "battery_monitor.h"

#include <Arduino.h>
#ifndef UNIT_TEST
#include <Wire.h>
#endif
#include <stddef.h>
#include <stdint.h>
#include <cmath>

#include "sensors/ina226_sensor.h"
#include "common/core/serial_manager.h"
#include "modules/safety/safety_coordinator.h"

namespace sigyn_teensy {

  // Static member definitions
  BatteryConfig BatteryMonitor::g_battery_config_[kNumberOfBatteries] = {
      {32.0f, 34.0f, 2.375, 15.0f, 100, 0x40, A0, A1, 11.0f, 0.0f, 0.0f, 1.0f,
       1.0f, "36VLIPO"},
      {4.9f, 4.94f, 1.0f, 9.0f, 100, 0x40, A0, A1, 11.0f, 0.0f, 0.0f, 1.0f, 1.0f,
       "5VDCDC"},
      {11.8f, 11.9f, 1.0f, 19.0f, 100, 0x40, A0, A1, 11.0f, 0.0f, 0.0f, 1.0f,
       1.0f, "12VDCDC"},
      {23.0f, 23.5f, 1.0f, 9.0f, 100, 0x40, A0, A1, 11.0f, 0.0f, 0.0f, 1.0f, 1.0f,
       "24VDCDC"},
      {3.1f, 3.2f, 1.0f, 9.0f, 100, 0x40, A0, A1, 11.0f, 0.0f, 0.0f, 1.0f, 1.0f,
       "3.3VDCDC"},
  };

  uint8_t BatteryMonitor::gINA226_DeviceIndexes_[kNumberOfBatteries] = { 2, 3, 4,
                                                                        5, 6 };

  BatteryMonitor::BatteryMonitor()
    : multiplexer_available_(false), setup_completed_(false) {
    config_ = g_battery_config_[0];

    // Initialize EMA arrays and sensors
    for (size_t i = 0; i < kNumberOfBatteries; i++) {
      voltage_ema_[i] = 0.0f;
      current_ema_[i] = 0.0f;
      state_[i] = BatteryState::UNKNOWN;
      total_readings_[i] = 0;
      sensors_[i] = nullptr;
      sensor_owned_[i] = false;
    }
  }

  void BatteryMonitor::registerSensor(size_t index, IPowerSensor* sensor) {
    if (index < kNumberOfBatteries) {
      // If we previously owned a sensor here, we should delete it (though typically registration happens before setup)
      // For safety on embedded, we assume registration happens once at startup.
      sensors_[index] = sensor;
      sensor_owned_[index] = false; 
    }
  }

  float BatteryMonitor::estimateChargePercentage(float voltage) const {
    // Simple linear approximation for Li-ion battery pack
    constexpr float empty_voltage = 32.0f;  // 0% charge
    constexpr float full_voltage = 42.0f;   // 100% charge

    if (voltage < empty_voltage) {
      return 0.0f;
    }
    else if (voltage > full_voltage) {
      return 1.0f;
    }
    else {
      return (voltage - empty_voltage) / (full_voltage - empty_voltage);
    }
  }

  float BatteryMonitor::getCurrent(size_t idx) const { return current_ema_[idx]; }

  BatteryMonitor& BatteryMonitor::getInstance() {
    static BatteryMonitor instance;
    return instance;
  }

  float BatteryMonitor::getVoltage(size_t idx) const {
    return voltage_ema_[idx] * g_battery_config_[idx].voltage_multiplier;
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
        
        // Sensor failure check
        if (state_[battery_idx] == BatteryState::DEGRADED) {
            // Try to re-init periodically? Or just skip? 
            // For now, we skip reading but maybe we should try to recover.
            if (!sensors_[battery_idx]->isConnected()) {
                 continue; // Still broken
            } else {
                 // Recovered?
                 state_[battery_idx] = BatteryState::UNKNOWN; // Reset to unknown to re-evaluate
            }
        }

        if (sensors_[battery_idx]->isConnected()) {
          voltage = sensors_[battery_idx]->readBusVoltage();
          current = sensors_[battery_idx]->readCurrent();
          
          if (total_readings_[battery_idx] == 0) {
            voltage_ema_[battery_idx] = voltage;
            current_ema_[battery_idx] = current;
          }
          else {
            voltage_ema_[battery_idx] = updateExponentialAverage(
              voltage_ema_[battery_idx], voltage, kDefaultVoltageAlpha);
            current_ema_[battery_idx] = updateExponentialAverage(
              current_ema_[battery_idx], current, kDefaultCurrentAlpha);
          }
  
          updateBatteryState(battery_idx);
          total_readings_[battery_idx]++;
        } else {
            // Mark as DEGRADED if connection lost during operation
            if (state_[battery_idx] != BatteryState::DEGRADED) {
                state_[battery_idx] = BatteryState::DEGRADED;
                
                // Report sensor failure to SafetyCoordinator
                SafetyCoordinator& safety = SafetyCoordinator::getInstance();
                char fault_name[64];
                char description[128];
                snprintf(fault_name, sizeof(fault_name), "%s_Battery%zu", name(), battery_idx);
                snprintf(description, sizeof(description), 
                  "Battery %zu (%s) sensor lost connection",
                  battery_idx, g_battery_config_[battery_idx].battery_name);
                  
                safety.activateFault(FaultSeverity::DEGRADED, fault_name, description);
                SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), description);
            }
        }
      }

      last_read_time = now_ms;
    }

    // Status reporting phase - send diagnostic messages
    if ((now_ms - last_message_send_time_ms_) > MAIN_BATTERY_REPORT_INTERVAL_MS) {
      for (size_t device_index = 0; device_index < kNumberOfBatteries;
        device_index++) {
        sendStatusMessage(device_index);
      }
      last_message_send_time_ms_ = now_ms;
    }
  }

  const char* BatteryMonitor::name() const { return "BatteryMonitor"; }

  // Check if any battery is in critical state
  bool BatteryMonitor::isUnsafe() {
      for (size_t i = 0; i < kNumberOfBatteries; i++) {
          if (state_[i] == BatteryState::CRITICAL) {
              return true;
          }
      }
      return false;
  }

  void BatteryMonitor::selectSensor(size_t battery_idx) const {
      // Deprecated: sensors now handle their own selection
      // Kept if needed for manual mux testing
      Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
      Wire.write(1 << gINA226_DeviceIndexes_[battery_idx]);
      Wire.endTransmission();
  }

  void BatteryMonitor::sendStatusMessage(size_t idx) {
    // Create a JSON string for the battery status
    char message[256];
    snprintf(message, sizeof(message),
             "{\"idx\":%u,\"V\":%.2f,\"A\":%.2f,\"charge\":%.0f,\"state\":\"%s\",\"location\":\"%s\"}",
             static_cast<unsigned>(idx), static_cast<double>(getVoltage(idx)), static_cast<double>(getCurrent(idx)),
             static_cast<double>(estimateChargePercentage(getVoltage(idx))), batteryStateToString(state_[idx]),
             g_battery_config_[idx].battery_name);

    SerialManager::getInstance().sendMessage("BATT", message);
  }

  void BatteryMonitor::setup() {
    pinMode(kI2CMultiplexorEnablePin, OUTPUT);
    digitalWrite(kI2CMultiplexorEnablePin, HIGH);
    Wire.begin();
    Wire.setClock(400000);
    
    // We still test the mux because it is a shared resource
    multiplexer_available_ = testI2cMultiplexer();
    if (!multiplexer_available_) {
      SerialManager::getInstance().sendDiagnosticMessage(
        "FATAL", name(),
        "BatteryMonitor setup failed: I2C multiplexer not available");
      // Don't return, allow generic fallback or partial operation?
      // If mux is dead, all sensors behind it are dead.
    }

    for (size_t device = 0; device < kNumberOfBatteries; device++) {
      config_ = g_battery_config_[device]; 

      // If no sensor injected, create default
      if (sensors_[device] == nullptr) {
          // Provide the mux channel to the sensor wrapper so it can self-select
          sensors_[device] = new INA226Sensor(0x40, gINA226_DeviceIndexes_[device], I2C_MULTIPLEXER_ADDRESS);
          sensor_owned_[device] = true;
      }

      // Initialize
      if (!sensors_[device]->init()) {
        char message[128];
        snprintf(message, sizeof(message), "Sensor initialization failed for device %u (%s)",
                 static_cast<unsigned>(device), g_battery_config_[device].battery_name);
        SerialManager::getInstance().sendDiagnosticMessage("ERROR", name(), message);
        state_[device] = BatteryState::DEGRADED; // Mark as degraded initially
        
        // Report initialization failure to SafetyCoordinator
        SafetyCoordinator& safety = SafetyCoordinator::getInstance();
        char fault_name[64];
        snprintf(fault_name, sizeof(fault_name), "%s_Battery%zu", name(), device);
        safety.activateFault(FaultSeverity::DEGRADED, fault_name, message);
      } else {
          char debug_msg[128];
          snprintf(debug_msg, sizeof(debug_msg), "Successfully initialized device %zu", device);
          SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), debug_msg);
      }
    }
    setup_completed_ = true;
  }

  bool BatteryMonitor::testI2cMultiplexer() {
    Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
    uint8_t error = Wire.endTransmission();
    char message[128];
    if (error == 0) {
      snprintf(message, sizeof(message),
        "[BatteryMonitor::testI2cMultiplexer] I2C multiplexer found at "
        "address 0x%02X",
        I2C_MULTIPLEXER_ADDRESS);
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), message);
      return true;
    }
    else {
      snprintf(
        message, sizeof(message),
        "[BatteryMonitor::testI2cMultiplexer] I2C multiplexer NOT found at "
        "address 0x%02X (error: %d)",
        I2C_MULTIPLEXER_ADDRESS, error);
      SerialManager::getInstance().sendDiagnosticMessage("FATAL", name(),
        message);
      return false;
    }
  }

  void BatteryMonitor::updateBatteryState(size_t idx) {
    char msg[256];
    float voltage = getVoltage(idx);
    float current = getCurrent(idx);

    // Determine desired state and severity
    BatteryState new_state = BatteryState::NORMAL;
    FaultSeverity desired_severity = FaultSeverity::NORMAL;
    char desired_description[128] = {0};

    if (voltage < g_battery_config_[idx].critical_low_voltage ||
      abs(current) > g_battery_config_[idx].critical_high_current) {
      new_state = BatteryState::CRITICAL;
      desired_severity = FaultSeverity::EMERGENCY_STOP;
      snprintf(desired_description, sizeof(desired_description),
        "Battery %zu (%s) CRITICAL: V=%.2f A=%.2f",
        idx, g_battery_config_[idx].battery_name, voltage, current);
    }
    else if (voltage < g_battery_config_[idx].warning_low_voltage) {
      new_state = BatteryState::WARNING;
      desired_severity = FaultSeverity::WARNING;
      snprintf(desired_description, sizeof(desired_description),
        "Battery %zu (%s) WARNING: V=%.2f A=%.2f",
        idx, g_battery_config_[idx].battery_name, voltage, current);
    }
    else if ((idx == 0) && (current < 0.0f)) {
      new_state = BatteryState::CHARGING;
    }
    else {
      new_state = BatteryState::NORMAL;
    }

    // Update state and report to SafetyCoordinator if changed
    bool state_changed = (state_[idx] != new_state);
    state_[idx] = new_state;

    // Report to SafetyCoordinator (avoid spamming)
    SafetyCoordinator& safety = SafetyCoordinator::getInstance();
    
    // Build fault name with battery index
    char fault_name[64];
    snprintf(fault_name, sizeof(fault_name), "%s_Battery%zu", name(), idx);
    
    const Fault& current_fault = safety.getFault(fault_name);
    
    if (desired_severity == FaultSeverity::NORMAL) {
      if (current_fault.active) {
        safety.deactivateFault(fault_name);
      }
    } else {
      // Report if new fault or severity/description changed
      if (!current_fault.active || 
          current_fault.severity != desired_severity || 
          strcmp(current_fault.description, desired_description) != 0) {
        safety.activateFault(desired_severity, fault_name, desired_description);
        
        // Also log to serial
        if (state_changed) {
          const char* severity_str = (desired_severity == FaultSeverity::EMERGENCY_STOP) ? "CRITICAL" : "WARNING";
          SerialManager::getInstance().sendDiagnosticMessage(severity_str, name(), desired_description);
        }
      }
    }
  }

  float BatteryMonitor::updateExponentialAverage(float current_avg,
    float new_value, float alpha) {
    return alpha * new_value + (1.0f - alpha) * current_avg;
  }

  const char* BatteryMonitor::batteryStateToString(BatteryState state) const {
    switch (state) {
    case BatteryState::UNKNOWN:
      return "UNKNOWN";
    case BatteryState::CHARGING:
      return "CHARGING";
    case BatteryState::DISCHARGING:
      return "DISCHARGING";
    case BatteryState::CRITICAL:
      return "CRITICAL";
    case BatteryState::WARNING:
      return "WARNING";
    case BatteryState::NORMAL:
      return "NORMAL";
    case BatteryState::DEGRADED:
      return "DEGRADED";
    default:
      return "INVALID";
    }
  }

}  // namespace sigyn_teensy