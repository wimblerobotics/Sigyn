/*
 * Simple VL53L0X Calibration Program
 * 
 * Hardware Setup:
 * - VL53L0X SDA -> Teensy SDA (pin 18)
 * - VL53L0X SCL -> Teensy SCL (pin 19)  
 * - VL53L0X VCC -> 3.3V
 * - VL53L0X GND -> GND
 * 
 * Commands:
 * - 'm' + Enter: Read distance and display it
 * - 'c' + Enter: Calibrate to 200mm (hold sensor exactly 200mm from target)
 */

#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

VL53L0X sensor;

float calibration_offset = 0.0;
bool is_calibrated = false;
const int EEPROM_ADDR = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  
  Serial.println("VL53L0X Simple Calibration");
  Serial.println("Commands: 'm' = measure, 'c' = calibrate to 200mm");
  
  Wire.begin();
  Wire.setClock(400000);
  
  if (!sensor.init()) {
    Serial.println("ERROR: VL53L0X init failed!");
    while(1);
  }
  
  sensor.setTimeout(500);
  sensor.startContinuous(50);
  
  // Load calibration from EEPROM if available
  EEPROM.get(EEPROM_ADDR, calibration_offset);
  if (!isnan(calibration_offset) && abs(calibration_offset) < 1000) {
    is_calibrated = true;
    Serial.println("Loaded calibration: " + String(calibration_offset, 1) + "mm offset");
  } else {
    calibration_offset = 0.0;
    Serial.println("No calibration found");
  }
  
  Serial.println("Ready. Type 'm' or 'c' + Enter");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Clear any remaining characters
    while (Serial.available()) Serial.read();
    
    if (cmd == 'm' || cmd == 'M') {
      measureDistance();
    } else if (cmd == 'c' || cmd == 'C') {
      calibrate();
    }
  }
}

void measureDistance() {
  uint16_t raw_distance = sensor.readRangeContinuousMillimeters();
  
  if (sensor.timeoutOccurred()) {
    Serial.println("Measurement timeout!");
    return;
  }
  
  float calibrated_distance = raw_distance + calibration_offset;
  
  Serial.print("Distance: ");
  if (is_calibrated) {
    Serial.print(String(calibrated_distance, 1));
    Serial.print("mm (calibrated, raw: ");
    Serial.print(raw_distance);
    Serial.println("mm)");
  } else {
    Serial.print(String(raw_distance));
    Serial.println("mm (NOT CALIBRATED)");
  }
}

void calibrate() {
  Serial.println("Calibrating to 200mm...");
  Serial.println("Position sensor exactly 200mm from target and press Enter");
  
  // Wait for Enter key
  while (Serial.read() != '\n') {
    if (Serial.available()) Serial.read();
  }
  
  // Take measurement
  uint16_t measured = sensor.readRangeContinuousMillimeters();
  
  if (sensor.timeoutOccurred()) {
    Serial.println("Calibration failed - timeout!");
    return;
  }
  
  calibration_offset = 200.0 - measured;
  is_calibrated = true;
  
  // Save to EEPROM
  EEPROM.put(EEPROM_ADDR, calibration_offset);
  
  Serial.println("Calibration complete!");
  Serial.println("Measured: " + String(measured) + "mm");
  Serial.println("Offset: " + String(calibration_offset, 1) + "mm");
  Serial.println("Saved to EEPROM");
}
