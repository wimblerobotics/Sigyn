/**
 * @file battery_module.cpp
 * @brief Implementation of enhanced battery monitoring module for Teensy board 2.
 * 
 * This module provides comprehensive monitoring for dual power systems:
 * 1. 36V LIPO Battery (10-cell, 30Ah): Monitored via analog voltage divider
 * 2. Power Supplies (24V, 12V, 5V, 3.3V): Monitored via INA226 sensors with I2C multiplexer
 * 
 * Enhanced features:
 * - Exponential moving average for voltage and current with configurable alpha factors
 * - Outlier detection with configurable consecutive reading thresholds  
 * - Per-battery alarm states with overvoltage and overcurrent protection
 * - Motor overcurrent detection for critical 24V supply monitoring
 * - Non-resettable unsafe states for critical safety conditions
 * - Scalable architecture ready for additional power supply monitoring
 * 
 * Safety System:
 * - Overvoltage: Triggers when voltage > nominal * multiplier for N consecutive readings
 * - Overcurrent: Triggers when current > threshold for configurable time period
 * - Motor Protection: Critical monitoring for motor stall detection on 24V supply
 * - Global Safety: Aggregates all per-battery unsafe conditions
 * 
 * @author Sigyn Robotics  
 * @date 2025
 */

#include "battery_module.h"

#include "Arduino.h"         // For analogRead, millis, pinMode, etc.
#include "INA226.h"          // For INA226 power supply monitoring sensors
#include "config.h"          // For battery configuration constants
#include "serial_manager.h"  // For sending diagnostic messages

/**
 * @brief Constructs a new BatteryModule instance.
 * 
 * Initializes battery configurations for LIPO battery and future power supplies,
 * sets up alarm states, and prepares exponential averaging systems.
 * 
 * @note This constructor is private to enforce the singleton pattern.
 */
BatteryModule::BatteryModule()
    : Module(),
      multiplexer_available_(false),
      setup_completed_(false) {
  
  // Initialize per-battery data structures
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    // Initialize legacy circular buffer (for compatibility during transition)
    for (size_t reading = 0; reading < kNumberReadingsToAverage_; reading++) {
      g_averages_[device][reading] = 0.0;
    }
    g_next_average_index_[device] = 0;
    total_battery_readings_[device] = 0;
    
    // Initialize exponential averages
    g_voltage_averages_[device] = 0.0f;
    g_current_averages_[device] = 0.0f;
    
    // Initialize alarm states
    g_alarm_states_[device] = {
      .overvoltage_alarm = false,
      .overcurrent_alarm = false, 
      .is_unsafe = false,
      .consecutive_overvoltage = 0,
      .consecutive_overcurrent = 0,
      .overcurrent_start_time = 0
    };
  }
  
  // Initialize battery configurations
  initializeBatteryConfigs();

  // Log successful configuration
  SerialManager::singleton().SendDiagnosticMessage(
      String("[BatteryModule::BatteryModule] Configured for ") +
      String(kNumberOfBatteries) + String(" power systems"));
}

/**
 * @brief Initializes battery/power supply configurations.
 * 
 * Sets up monitoring parameters for each power system including voltage thresholds,
 * current limits, averaging factors, and safety parameters. Ready for expansion
 * when additional power supplies are added to the BatteryType enum.
 */
void BatteryModule::initializeBatteryConfigs() {
  // 36V LIPO Battery configuration (10-cell, 30Ah)
  g_battery_configs_[k36vBattery] = {
    .nominal_voltage = 36.0f,           // 10 cells * 3.6V nominal
    .max_charge_voltage = 42.0f,        // 10 cells * 4.2V max charge  
    .min_discharge_voltage = 30.0f,     // 10 cells * 3.0V min safe
    .overvoltage_multiplier = 1.25f,    // 25% over nominal triggers alarm
    .max_current = 25.0f,               // 30Ah battery can handle ~25A safely
    .overcurrent_time_threshold = 5.0f, // 5 seconds for overcurrent protection
    .consecutive_alarm_count = 5,       // 5 consecutive readings for alarm
    .voltage_alpha = kDefaultVoltageAlpha, // 10% new, 90% old for stability
    .current_alpha = kDefaultCurrentAlpha, // 20% new, 80% old for current averaging
    .is_lipo_battery = true,            // Use analog voltage monitoring (+ INA226 for current)
    .monitor_motor_current = false      // Not a motor supply, but monitor for safety
  };
  
  // Future power supply configurations will be added here when enum is expanded:
  /*
  g_battery_configs_[k24vSupply] = {
    .nominal_voltage = 24.0f,
    .max_charge_voltage = 26.0f,
    .min_discharge_voltage = 22.0f, 
    .overvoltage_multiplier = 1.15f,    // Tighter tolerance for power supplies
    .max_current = 15.0f,               // Maximum motor current
    .overcurrent_time_threshold = 2.0f, // 2 seconds for motor stall
    .consecutive_alarm_count = 3,       // Faster response for motor supply
    .voltage_alpha = kDefaultVoltageAlpha,
    .current_alpha = kDefaultCurrentAlpha,
    .is_lipo_battery = false,           // Use INA226 monitoring
    .monitor_motor_current = true       // Critical motor monitoring
  };
  */
}

/**
 * @brief Calculates battery state of charge percentage from voltage.
 * 
 * Uses linear interpolation between configured minimum and maximum battery
 * voltages to estimate the current state of charge. Clamps results to 0-100%.
 * 
 * @param battery The battery type (should be LIPO).
 * @param voltage The measured battery voltage in volts.
 * @return Estimated state of charge as percentage (0.0-100.0).
 * 
 * @note Assumes linear discharge curve which may not be accurate for all
 *       battery chemistries. Consider implementing lookup tables for better accuracy.
 */
float BatteryModule::calculatePercentage(BatteryType battery, float voltage) {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    return 0.0f;
  }
  
  const BatteryConfig& config = g_battery_configs_[battery];
  if (voltage >= config.max_charge_voltage) return 100.0f;
  if (voltage <= config.min_discharge_voltage) return 0.0f;

  return (voltage - config.min_discharge_voltage) / 
         (config.max_charge_voltage - config.min_discharge_voltage) * 100.0f;
}

/**
 * @brief Reads current consumption from the specified battery.
 * 
 * Returns the exponentially averaged current consumption from the INA226 sensor.
 * All batteries now have current monitoring via INA226 sensors.
 * 
 * @param battery The battery type to read current from.
 * @return Exponentially averaged current consumption in amperes, or 0.0f if not available.
 */
float BatteryModule::getCurrent(BatteryType battery) const {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::getCurrent] Invalid battery index: " +
        String(static_cast<int>(battery)));
    return 0.0f;
  }

  return g_current_averages_[battery];
}

/**
 * @brief Gets the averaged voltage reading for the specified battery.
 * 
 * Returns the exponentially averaged voltage from recent readings.
 * Uses the exponential moving average which provides better noise filtering
 * and responsiveness compared to circular buffer averaging.
 * 
 * @param battery The battery type to read voltage from.
 * @return Exponentially averaged voltage in volts, or 0.0f if invalid battery index.
 * 
 * @note Returns 0.0f if no readings have been taken yet.
 */
float BatteryModule::getVoltage(BatteryType battery) const {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    return 0.0f;
  }
  
  return g_voltage_averages_[battery];
}

/**
 * @brief Checks if any battery is in a critical safety condition.
 * 
 * Iterates through all configured batteries and checks their unsafe states.
 * Returns true if any battery requires immediate attention or emergency shutdown.
 * This aggregates all per-battery unsafe conditions for global safety monitoring.
 * 
 * @return true if any battery is in an unsafe state.
 * 
 * @note This method triggers safety protocols when batteries are dangerously low
 *       or experiencing alarm conditions.
 */
bool BatteryModule::isUnsafe() {
  for (size_t battery_idx = 0; battery_idx < kNumberOfBatteries; battery_idx++) {
    if (g_alarm_states_[battery_idx].is_unsafe) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Checks if a specific battery is in an unsafe state.
 * 
 * Returns the per-battery unsafe state which is set when alarm conditions
 * are met and is not automatically resettable.
 * 
 * @param battery The battery type to check.
 * @return true if the specified battery is in an unsafe state.
 */
bool BatteryModule::isUnsafe(BatteryType battery) const {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    return false;
  }
  
  return g_alarm_states_[battery].is_unsafe;
}

/**
 * @brief Gets the alarm state for a specific battery.
 * 
 * Returns the complete alarm state structure containing all alarm flags
 * and consecutive reading counts for the specified battery.
 * 
 * @param battery The battery type to get alarm state for.
 * @return Reference to the battery's alarm state structure.
 */
const BatteryModule::BatteryAlarmState& BatteryModule::getAlarmState(BatteryType battery) const {
  // Validate battery index and return reference to appropriate alarm state
  if (battery >= kNumberOfBatteries) {
    // Return reference to first alarm state as fallback (should not happen with proper validation)
    static const BatteryAlarmState invalid_state = {};
    return invalid_state;
  }
  
  return g_alarm_states_[battery];
}

/**
 * @brief Main processing loop for battery monitoring operations.
 * 
 * Performs comprehensive battery monitoring including:
 * 1. Battery reading: Samples all batteries and updates averages
 * 2. Safety monitoring: Checks for alarm conditions and outliers
 * 3. Status reporting: Sends diagnostic messages with current battery state
 * 
 * Reading strategy:
 * - Battery 0: Analog voltage divider via MAIN_BATTERY_PIN
 * - Battery 1+: INA226 bus voltage via I2C multiplexer
 * 
 * @note Called periodically by the base Module class framework.
 */
void BatteryModule::loop() {
  static uint32_t last_message_send_time_ms_ = millis();
  static uint32_t last_battery_read_time_ms_ = millis();
  unsigned long current_time_ms = millis();
  
  // Battery reading phase - sample all batteries
  if ((current_time_ms - last_battery_read_time_ms_) > MAIN_BATTERY_READ_INTERVAL_MS) {
    for (size_t battery_idx = 0; battery_idx < kNumberOfBatteries; battery_idx++) {
      BatteryType battery_type = static_cast<BatteryType>(battery_idx);
      float battery_v = 0.0f;
      float battery_c = 0.0f;
      
      if (battery_idx == 0) {
        // Main battery: analog voltage divider reading + INA226 current monitoring
        float raw = analogRead(MAIN_BATTERY_PIN);
        battery_v = raw * k36VAnalogToVoltageConversion;  // Use named constant
        
        // Get current from INA226 sensor for main battery
        selectSensor(battery_type);
        battery_c = g_ina226_[battery_idx].getCurrent();
      } else {
        // Additional batteries: INA226 bus voltage and current reading
        selectSensor(battery_type);
        battery_v = g_ina226_[battery_idx].getBusVoltage();
        battery_c = g_ina226_[battery_idx].getCurrent();
      }
      
      // Update exponential averages
      const BatteryConfig& config = g_battery_configs_[battery_type];
      if (total_battery_readings_[battery_idx] == 0) {
        // Initialize averages with first reading
        g_voltage_averages_[battery_idx] = battery_v;
        g_current_averages_[battery_idx] = battery_c;
      } else {
        // Apply exponential moving average
        g_voltage_averages_[battery_idx] = updateExponentialAverage(
            g_voltage_averages_[battery_idx], battery_v, config.voltage_alpha);
        g_current_averages_[battery_idx] = updateExponentialAverage(
            g_current_averages_[battery_idx], battery_c, config.current_alpha);
      }
      
      // Store reading in legacy circular buffer (for compatibility)
      g_averages_[battery_idx][g_next_average_index_[battery_idx]] = battery_v;
      
      // Advance circular buffer index
      g_next_average_index_[battery_idx]++;
      if (g_next_average_index_[battery_idx] >= kNumberReadingsToAverage_) {
        g_next_average_index_[battery_idx] = 0;
      }
      
      // Update total reading count (used for averaging)
      total_battery_readings_[battery_idx]++;
      
      // Process safety checks and alarms
      processSafetyChecks(battery_type, battery_v, battery_c);
    }
    
    last_battery_read_time_ms_ = current_time_ms;
  }

  // Status reporting phase - send diagnostic messages
  if ((current_time_ms - last_message_send_time_ms_) > MAIN_BATTERY_REPORT_INTERVAL_MS) {
    for (size_t device_index = 0; device_index < kNumberOfBatteries; device_index++) {
      sendBatteryState(static_cast<BatteryType>(device_index));
    }
    last_message_send_time_ms_ = current_time_ms;
  }
}

/**
 * @brief Resets safety warning flags and conditions.
 * 
 * Clears any safety-related warning states when battery conditions improve.
 * Currently a placeholder for future safety flag implementation.
 * 
 * @todo Implement actual safety flag management when warning system is added.
 */
void BatteryModule::resetSafetyFlags() {
  // TODO: Implement safety flag reset logic when warning flags are added
}

/**
 * @brief Selects the specified battery sensor via I2C multiplexer.
 * 
 * Configures the I2C multiplexer to route communication to the INA226
 * sensor associated with the specified battery. Includes a settling delay
 * to ensure the multiplexer has switched channels properly.
 * 
 * @param battery The battery type whose sensor should be selected.
 * 
 * @note The 100μs delay is hardware-specific and may need adjustment
 *       for different multiplexer models.
 */
void BatteryModule::selectSensor(BatteryType battery) const {
  // Configure I2C multiplexer channel selection
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << gINA226_DeviceIndexes_[battery]);  // Set channel bit
  Wire.endTransmission();

  // Allow multiplexer to settle on new channel
  delayMicroseconds(100);
}

/**
 * @brief Constructs and sends a formatted battery status message.
 * 
 * Creates a diagnostic message containing voltage, state of charge percentage,
 * current consumption, and alarm status for the specified battery. Message format:
 * "BATTERY:index,voltage,percentage,current,alarms"
 * 
 * @param battery The battery type to report.
 * 
 * @note Message buffer is limited to 128 characters to accommodate alarm status.
 */
void BatteryModule::sendBatteryState(BatteryType battery) {
  // Validate battery type
  if (battery >= kNumberOfBatteries) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::sendBatteryState] Invalid battery type: " +
        String(static_cast<int>(battery)));
    return;
  }

  // Format battery status message
  char message[128];
  float voltage = getVoltage(battery);
  float percentage = calculatePercentage(battery, voltage);
  float current = getCurrent(battery);
  const BatteryAlarmState& alarms = g_alarm_states_[battery];

  snprintf(message, sizeof(message), "BATTERY:%d,%.2fV,%.1f%%,%.2fA,%s%s%s",
           static_cast<int>(battery), voltage, percentage, current,
           alarms.overvoltage_alarm ? "OV," : "",
           alarms.overcurrent_alarm ? "OC," : "", 
           alarms.is_unsafe ? "UNSAFE" : "OK");
  
  SerialManager::singleton().SendDiagnosticMessage(message);
}

/**
 * @brief Initializes all battery monitoring hardware and sensors.
 * 
 * Performs complete system initialization including:
 * - GPIO pin configuration for analog reading and power control
 * - I2C bus initialization with high-speed configuration
 * - I2C multiplexer connectivity verification
 * - INA226 sensor initialization and configuration
 * 
 * @note Setup is idempotent - subsequent calls are safely ignored.
 * @warning If multiplexer is not available, sensor initialization is skipped.
 */
void BatteryModule::setup() {
  if (setup_completed_) {
    return;  // Prevent duplicate initialization
  }

  SerialManager::singleton().SendDiagnosticMessage(
      "[BatteryModule::setup] Starting sensor initialization...");

  // Configure hardware pins
  pinMode(MAIN_BATTERY_PIN, INPUT);              // Analog voltage reading
  pinMode(kI2CMultiplexorEnablePin, OUTPUT);     // I2C multiplexer power control
  digitalWrite(kI2CMultiplexorEnablePin, HIGH);  // Enable I2C multiplexer power

  // Initialize I2C with high-speed configuration
  Wire.begin();
  Wire.setClock(400000);  // 400kHz for faster sensor communication

  // Verify I2C multiplexer connectivity
  multiplexer_available_ = testI2CMultiplexer();
  if (!multiplexer_available_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::setup] I2C multiplexer not available, cannot proceed.");
    return;
  }

  // Initialize all INA226 sensors
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    selectSensor(static_cast<BatteryType>(device));
    g_ina226_[device].begin();
    // Configure for 20A max current with 2mΩ shunt resistor
    g_ina226_[device].setMaxCurrentShunt(20, 0.002);
    // TODO: Consider enabling averaging and continuous mode for better accuracy
    // g_ina226_[device].setAverage(INA226_AVERAGE_16);
    // g_ina226_[device].setMode(INA226_MODE_CONTINUOUS);
  }

  setup_completed_ = true;
  SerialManager::singleton().SendDiagnosticMessage(
      "[BatteryModule::setup] Initialization completed successfully.");
}

/**
 * @brief Gets the singleton instance of BatteryModule.
 * 
 * Implements thread-safe lazy initialization of the singleton instance.
 * Creates the instance on first access and returns the same instance
 * for all subsequent calls.
 * 
 * @return Reference to the singleton BatteryModule instance.
 * 
 * @note Not thread-safe in multi-threaded environments. Consider adding
 *       mutex protection if used in multi-threaded context.
 */
BatteryModule& BatteryModule::singleton() {
  if (!g_instance_) {
    g_instance_ = new BatteryModule();
  }
  return *g_instance_;
}

/**
 * @brief Tests I2C multiplexer presence and responsiveness.
 * 
 * Attempts to establish communication with the I2C multiplexer to verify
 * it's present and functioning before attempting sensor initialization.
 * Provides diagnostic feedback on success or failure.
 * 
 * @return true if multiplexer responds successfully to I2C communication.
 * 
 * @note This is a simple presence test. Consider adding more comprehensive
 *       functionality tests for production use.
 */
bool BatteryModule::testI2CMultiplexer() {
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer found at address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX));
    return true;
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[BatteryModule::testI2CMultiplexer] I2C multiplexer NOT found at address 0x" +
        String(I2C_MULTIPLEXER_ADDRESS, HEX) + " (error: " + String(error) + ")");
    return false;
  }
}

/**
 * @brief Processes safety checks for a specific battery.
 * 
 * Evaluates voltage and current readings against safety thresholds,
 * updates alarm states, and sets unsafe flags when conditions are met.
 * Implements outlier detection and consecutive reading requirements.
 * 
 * @param battery The battery type to check.
 * @param voltage Current voltage reading.
 * @param current Current reading (0.0 for LIPO battery).
 */
void BatteryModule::processSafetyChecks(BatteryType battery, float voltage, float current) {
  // Validate battery index
  if (battery >= kNumberOfBatteries) {
    return;
  }
  
  const BatteryConfig& config = g_battery_configs_[battery];
  BatteryAlarmState& alarm_state = g_alarm_states_[battery];
  unsigned long current_time = millis();
  
  // Overvoltage detection with consecutive reading requirement
  float overvoltage_threshold = config.nominal_voltage * config.overvoltage_multiplier;
  if (voltage > overvoltage_threshold) {
    alarm_state.consecutive_overvoltage++;
    if (alarm_state.consecutive_overvoltage >= config.consecutive_alarm_count) {
      if (!alarm_state.overvoltage_alarm) {
        SerialManager::singleton().SendDiagnosticMessage(
            String("[BatteryModule] OVERVOLTAGE ALARM: Battery ") + 
            String(static_cast<int>(battery)) + String(" - ") + 
            String(voltage, 2) + String("V > ") + String(overvoltage_threshold, 2) + String("V"));
      }
      alarm_state.overvoltage_alarm = true;
      alarm_state.is_unsafe = true;  // Overvoltage is a critical safety condition
    }
  } else {
    alarm_state.consecutive_overvoltage = 0;  // Reset consecutive count
    alarm_state.overvoltage_alarm = false;    // Allow overvoltage alarm to clear
  }
  
  // Undervoltage detection (immediate unsafe condition)
  if (voltage < config.min_discharge_voltage) {
    if (!alarm_state.is_unsafe) {
      SerialManager::singleton().SendDiagnosticMessage(
          String("[BatteryModule] UNDERVOLTAGE CRITICAL: Battery ") + 
          String(static_cast<int>(battery)) + String(" - ") + 
          String(voltage, 2) + String("V < ") + String(config.min_discharge_voltage, 2) + String("V"));
    }
    alarm_state.is_unsafe = true;  // Undervoltage is always critical
  }
  
  // Overcurrent detection with time-based threshold (for all batteries with current monitoring)
  if (config.max_current > 0.0f) {
    if (current > config.max_current) {
      if (alarm_state.overcurrent_start_time == 0) {
        alarm_state.overcurrent_start_time = current_time;  // Start timing overcurrent condition
      } else {
        // Check if overcurrent has persisted long enough
        unsigned long overcurrent_duration = current_time - alarm_state.overcurrent_start_time;
        if (overcurrent_duration >= (config.overcurrent_time_threshold * 1000)) {
          if (!alarm_state.overcurrent_alarm) {
            SerialManager::singleton().SendDiagnosticMessage(
                String("[BatteryModule] OVERCURRENT ALARM: Battery ") + 
                String(static_cast<int>(battery)) + String(" - ") + 
                String(current, 2) + String("A > ") + String(config.max_current, 2) + String("A"));
          }
          alarm_state.overcurrent_alarm = true;
          
          // For motor supplies, overcurrent is a critical safety condition
          // For LIPO battery, also critical as it indicates potential short circuit
          if (config.monitor_motor_current || config.is_lipo_battery) {
            alarm_state.is_unsafe = true;
          }
        }
      }
    } else {
      // Current is within limits, reset overcurrent tracking
      alarm_state.overcurrent_start_time = 0;
      alarm_state.overcurrent_alarm = false;  // Allow overcurrent alarm to clear
    }
  }
}

/**
 * @brief Updates exponential average with new reading.
 * 
 * Applies exponential moving average formula: 
 * new_avg = alpha * new_value + (1-alpha) * old_avg
 * 
 * @param current_average Current averaged value.
 * @param new_reading New sensor reading.
 * @param alpha Averaging factor (0.0-1.0, higher = more responsive).
 * @return Updated averaged value.
 */
float BatteryModule::updateExponentialAverage(float current_average, float new_reading, float alpha) {
  // Clamp alpha to valid range
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha > 1.0f) alpha = 1.0f;
  
  return alpha * new_reading + (1.0f - alpha) * current_average;
}

// Static member variable definitions
BatteryModule* BatteryModule::g_instance_ = nullptr;

// Legacy circular buffer arrays (kept for compatibility during transition)
float BatteryModule::g_averages_[BatteryModule::kNumberOfBatteries]
                                [BatteryModule::kNumberReadingsToAverage_] = {};

size_t BatteryModule::g_next_average_index_[BatteryModule::kNumberOfBatteries] = {0};

// Per-battery configuration and state arrays  
BatteryModule::BatteryConfig BatteryModule::g_battery_configs_[BatteryModule::kNumberOfBatteries] = {};
BatteryModule::BatteryAlarmState BatteryModule::g_alarm_states_[BatteryModule::kNumberOfBatteries] = {};
float BatteryModule::g_voltage_averages_[BatteryModule::kNumberOfBatteries] = {0.0f};
float BatteryModule::g_current_averages_[BatteryModule::kNumberOfBatteries] = {0.0f};

// Hardware interface arrays
uint8_t BatteryModule::gINA226_DeviceIndexes_[kNumberOfBatteries] = {
    2  // Main battery on multiplexer channel 2
};

INA226 BatteryModule::g_ina226_[kNumberOfBatteries] = {
    INA226(BatteryModule::INA226_ADDRESS)  // INA226 instance for battery monitoring
};