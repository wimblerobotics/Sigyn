/**
 * @file battery_monitor.cpp
 * @brief Implementation of comprehensive battery monitoring system
 * 
 * @author Sigyn Robotics
 * @date 2025
 */

#include "battery_monitor.h"
#include <cmath>

namespace sigyn_teensy {

// INA226 register addresses
constexpr uint8_t INA226_CONFIG_REG = 0x00;
constexpr uint8_t INA226_SHUNT_VOLTAGE_REG = 0x01;
constexpr uint8_t INA226_BUS_VOLTAGE_REG = 0x02;
constexpr uint8_t INA226_POWER_REG = 0x03;
constexpr uint8_t INA226_CURRENT_REG = 0x04;
constexpr uint8_t INA226_CALIBRATION_REG = 0x05;

// INA226 configuration values
constexpr uint16_t INA226_CONFIG_VALUE = 0x4527;  // 16 averages, 1.1ms conversion time
constexpr uint16_t INA226_CALIBRATION_VALUE = 0x0800;  // Calibration for 0.1 ohm shunt
constexpr float INA226_SHUNT_RESISTANCE = 0.1f;  // Shunt resistance in ohms
constexpr float INA226_CURRENT_LSB = 0.0001f;    // Current LSB in A/bit

BatteryMonitor& BatteryMonitor::GetInstance() {
  static BatteryMonitor instance;
  return instance;
}

BatteryMonitor::BatteryMonitor() 
    : Module(),
      voltage_(NAN),
      current_(NAN),
      power_(NAN),
      charge_percentage_(NAN),
      state_(BatteryState::UNKNOWN),
      ina226_available_(false),
      analog_available_(false),
      sensor_healthy_(false),
      last_sensor_update_(0),
      last_status_report_(0),
      voltage_critical_(false),
      current_critical_(false),
      power_critical_(false),
      safety_violation_start_(0),
      history_index_(0),
      history_count_(0),
      serial_manager_(&SerialManager::GetInstance()) {
  
  // Initialize history arrays
  for (size_t i = 0; i < kAverageWindow; ++i) {
    voltage_history_[i] = 0.0f;
    current_history_[i] = 0.0f;
  }
}

void BatteryMonitor::Configure(const BatteryConfig& config) {
  config_ = config;
}

void BatteryMonitor::setup() {
  // Initialize I2C communication
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed
  
  // Initialize sensors
  if (config_.enable_ina226) {
    ina226_available_ = InitializeINA226();
    if (ina226_available_) {
      Serial.println("BatteryMonitor: INA226 initialized successfully");
    } else {
      Serial.println("BatteryMonitor: INA226 initialization failed");
    }
  }
  
  if (config_.enable_analog_voltage) {
    // Configure analog pin
    pinMode(config_.analog_pin, INPUT);
    analog_available_ = true;
    Serial.println("BatteryMonitor: Analog voltage monitoring enabled");
  }
  
  sensor_healthy_ = ina226_available_ || analog_available_;
  
  if (!sensor_healthy_) {
    Serial.println("BatteryMonitor: ERROR - No functional sensors available!");
  }
  
  // Initialize state
  state_ = BatteryState::UNKNOWN;
  last_sensor_update_ = millis();
  last_status_report_ = millis();
  
  Serial.println("BatteryMonitor: Initialization complete");
}

void BatteryMonitor::loop() {
  uint32_t current_time = millis();
  
  // Update sensor readings
  if (current_time - last_sensor_update_ >= config_.update_period_ms) {
    last_sensor_update_ = current_time;
    
    // Read voltage from available sensors
    float new_voltage = NAN;
    if (ina226_available_) {
      new_voltage = ReadINA226Voltage();
    }
    if (isnan(new_voltage) && analog_available_) {
      new_voltage = ReadAnalogVoltage();
    }
    
    // Read current from INA226
    float new_current = NAN;
    if (ina226_available_) {
      new_current = ReadINA226Current();
    }
    
    // Update moving averages if we have valid readings
    if (!isnan(new_voltage)) {
      voltage_history_[history_index_] = new_voltage;
      if (history_count_ < kAverageWindow) {
        history_count_++;
      }
    }
    
    if (!isnan(new_current)) {
      current_history_[history_index_] = new_current;
    }
    
    history_index_ = (history_index_ + 1) % kAverageWindow;
    
    // Calculate averages
    if (history_count_ > 0) {
      float voltage_sum = 0.0f;
      float current_sum = 0.0f;
      
      for (size_t i = 0; i < history_count_; ++i) {
        voltage_sum += voltage_history_[i];
        current_sum += current_history_[i];
      }
      
      voltage_ = voltage_sum / history_count_;
      if (!isnan(new_current)) {
        current_ = current_sum / history_count_;
        power_ = voltage_ * current_;
      }
      
      charge_percentage_ = EstimateChargePercentage(voltage_);
    }
    
    // Update battery state and check safety
    UpdateBatteryState();
    CheckSafetyThresholds();
    
    // Update sensor health status
    sensor_healthy_ = !isnan(voltage_) || !isnan(current_);
  }
  
  // Send status updates
  if (current_time - last_status_report_ >= config_.report_period_ms) {
    last_status_report_ = current_time;
    SendStatusMessage();
  }
}

bool BatteryMonitor::IsUnsafe() {
  return voltage_critical_ || current_critical_ || power_critical_ || !sensor_healthy_;
}

void BatteryMonitor::ProcessMessage(const String& message) {
  // Handle configuration updates from ROS2
  if (message.startsWith("CONFIG:")) {
    String config_data = message.substring(7);
    
    // Parse key-value pairs for configuration updates
    int start = 0;
    while (start < config_data.length()) {
      int equals_pos = config_data.indexOf('=', start);
      int comma_pos = config_data.indexOf(',', start);
      
      if (equals_pos == -1) break;
      if (comma_pos == -1) comma_pos = config_data.length();
      
      String key = config_data.substring(start, equals_pos);
      String value = config_data.substring(equals_pos + 1, comma_pos);
      
      // Update configuration based on key
      if (key == "critical_low_v") {
        config_.critical_low_voltage = value.toFloat();
      } else if (key == "warning_low_v") {
        config_.warning_low_voltage = value.toFloat();
      } else if (key == "critical_high_c") {
        config_.critical_high_current = value.toFloat();
      } else if (key == "update_period") {
        config_.update_period_ms = value.toInt();
      }
      
      start = comma_pos + 1;
    }
    
    Serial.println("BatteryMonitor: Configuration updated");
  }
}

bool BatteryMonitor::InitializeINA226() {
  // Reset INA226
  Wire.beginTransmission(config_.ina226_address);
  Wire.write(INA226_CONFIG_REG);
  Wire.write(0x80);  // Reset bit
  Wire.write(0x00);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  delay(2);  // Wait for reset
  
  // Configure INA226
  Wire.beginTransmission(config_.ina226_address);
  Wire.write(INA226_CONFIG_REG);
  Wire.write(INA226_CONFIG_VALUE >> 8);
  Wire.write(INA226_CONFIG_VALUE & 0xFF);
  error = Wire.endTransmission();
  
  if (error != 0) {
    return false;
  }
  
  // Set calibration
  Wire.beginTransmission(config_.ina226_address);
  Wire.write(INA226_CALIBRATION_REG);
  Wire.write(INA226_CALIBRATION_VALUE >> 8);
  Wire.write(INA226_CALIBRATION_VALUE & 0xFF);
  error = Wire.endTransmission();
  
  return (error == 0);
}

float BatteryMonitor::ReadINA226Voltage() {
  Wire.beginTransmission(config_.ina226_address);
  Wire.write(INA226_BUS_VOLTAGE_REG);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return NAN;
  }
  
  Wire.requestFrom(config_.ina226_address, (uint8_t)2);
  if (Wire.available() < 2) {
    return NAN;
  }
  
  uint16_t raw_voltage = Wire.read() << 8;
  raw_voltage |= Wire.read();
  
  // Convert to voltage (LSB = 1.25mV)
  return (raw_voltage * 1.25e-3f);
}

float BatteryMonitor::ReadINA226Current() {
  Wire.beginTransmission(config_.ina226_address);
  Wire.write(INA226_CURRENT_REG);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    return NAN;
  }
  
  Wire.requestFrom(config_.ina226_address, (uint8_t)2);
  if (Wire.available() < 2) {
    return NAN;
  }
  
  int16_t raw_current = Wire.read() << 8;
  raw_current |= Wire.read();
  
  // Convert to current using calibration
  return (raw_current * INA226_CURRENT_LSB);
}

float BatteryMonitor::ReadAnalogVoltage() {
  int raw_reading = analogRead(config_.analog_pin);
  
  // Convert ADC reading to voltage
  float adc_voltage = (raw_reading / 1024.0f) * 3.3f;  // Assuming 3.3V reference
  
  // Apply voltage divider ratio
  return adc_voltage * config_.voltage_divider_ratio;
}

void BatteryMonitor::UpdateBatteryState() {
  if (isnan(voltage_)) {
    state_ = BatteryState::UNKNOWN;
    return;
  }
  
  // Determine state based on voltage and current
  if (voltage_ < config_.critical_low_voltage) {
    state_ = BatteryState::CRITICAL;
  } else if (voltage_ < config_.warning_low_voltage) {
    state_ = BatteryState::WARNING;
  } else if (!isnan(current_)) {
    if (current_ < -0.1f) {  // Negative current indicates charging
      state_ = BatteryState::CHARGING;
    } else if (current_ > 0.1f) {  // Positive current indicates discharging
      state_ = BatteryState::DISCHARGING;
    } else {
      state_ = BatteryState::NORMAL;  // Near zero current
    }
  } else {
    state_ = BatteryState::NORMAL;  // Default if current unavailable
  }
}

void BatteryMonitor::SendStatusMessage() {
  if (!serial_manager_) return;
  
  // Create status message with all battery data
  String message = "id=0";
  
  if (!isnan(voltage_)) {
    message += ",v=" + String(voltage_, 2);
  }
  
  if (!isnan(current_)) {
    message += ",c=" + String(current_, 3);
  }
  
  if (!isnan(power_)) {
    message += ",p=" + String(power_, 2);
  }
  
  if (!isnan(charge_percentage_)) {
    message += ",pct=" + String(charge_percentage_, 2);
  }
  
  // Add state information
  message += ",state=";
  switch (state_) {
    case BatteryState::UNKNOWN: message += "UNKNOWN"; break;
    case BatteryState::CHARGING: message += "CHARGING"; break;
    case BatteryState::DISCHARGING: message += "DISCHARGING"; break;
    case BatteryState::CRITICAL: message += "CRITICAL"; break;
    case BatteryState::WARNING: message += "WARNING"; break;
    case BatteryState::NORMAL: message += "NORMAL"; break;
  }
  
  // Add sensor health
  message += ",sensors=";
  if (ina226_available_) message += "INA226";
  if (analog_available_) {
    if (ina226_available_) message += "+";
    message += "ANALOG";
  }
  if (!sensor_healthy_) message += "NONE";
  
  serial_manager_->SendMessage("BATT", message.c_str());
}

void BatteryMonitor::CheckSafetyThresholds() {
  uint32_t current_time = millis();
  bool new_violation = false;
  
  // Check voltage thresholds
  bool voltage_critical_new = !isnan(voltage_) && (voltage_ < config_.critical_low_voltage);
  if (voltage_critical_new && !voltage_critical_) {
    String estop_msg = "active=true,source=battery,reason=low_voltage,value=" + String(voltage_, 2);
    serial_manager_->SendMessage("ESTOP", estop_msg.c_str());
    new_violation = true;
  } else if (!voltage_critical_new && voltage_critical_) {
    serial_manager_->SendMessage("ESTOP", "active=false,source=battery,reason=low_voltage");
  }
  voltage_critical_ = voltage_critical_new;
  
  // Check current thresholds
  bool current_critical_new = !isnan(current_) && (fabs(current_) > config_.critical_high_current);
  if (current_critical_new && !current_critical_) {
    String estop_msg = "active=true,source=battery,reason=high_current,value=" + String(current_, 2);
    serial_manager_->SendMessage("ESTOP", estop_msg.c_str());
    new_violation = true;
  } else if (!current_critical_new && current_critical_) {
    serial_manager_->SendMessage("ESTOP", "active=false,source=battery,reason=high_current");
  }
  current_critical_ = current_critical_new;
  
  // Check power thresholds
  bool power_critical_new = !isnan(power_) && (power_ > config_.max_power);
  if (power_critical_new && !power_critical_) {
    String estop_msg = "active=true,source=battery,reason=high_power,value=" + String(power_, 2);
    serial_manager_->SendMessage("ESTOP", estop_msg.c_str());
    new_violation = true;
  } else if (!power_critical_new && power_critical_) {
    serial_manager_->SendMessage("ESTOP", "active=false,source=battery,reason=high_power");
  }
  power_critical_ = power_critical_new;
  
  // Track safety violation timing
  if (new_violation) {
    safety_violation_start_ = current_time;
  }
}

float BatteryMonitor::EstimateChargePercentage(float voltage) const {
  // Simple linear approximation for Li-ion battery pack
  // Adjust these values based on actual battery characteristics
  constexpr float empty_voltage = 32.0f;   // 0% charge
  constexpr float full_voltage = 42.0f;    // 100% charge
  
  if (voltage < empty_voltage) {
    return 0.0f;
  } else if (voltage > full_voltage) {
    return 1.0f;
  } else {
    return (voltage - empty_voltage) / (full_voltage - empty_voltage);
  }
}

void BatteryMonitor::PrintStatus() const {
  Serial.print("Battery Status - Voltage: ");
  Serial.print(voltage_, 2);
  Serial.print("V, Current: ");
  Serial.print(current_, 2);
  Serial.print("A, Power: ");
  Serial.print(power_, 2);
  Serial.print("W, State: ");
  
  switch (state_) {
    case BatteryState::UNKNOWN:
      Serial.print("UNKNOWN");
      break;
    case BatteryState::CHARGING:
      Serial.print("CHARGING");
      break;
    case BatteryState::DISCHARGING:
      Serial.print("DISCHARGING");
      break;
    case BatteryState::CRITICAL:
      Serial.print("CRITICAL");
      break;
    case BatteryState::WARNING:
      Serial.print("WARNING");
      break;
    case BatteryState::NORMAL:
      Serial.print("NORMAL");
      break;
  }
  
  Serial.print(", Charge: ");
  Serial.print(charge_percentage_ * 100.0f, 1);
  Serial.println("%");
}

}  // namespace sigyn_teensy
