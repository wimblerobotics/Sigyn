// SigynCore - Main Firmware for Teensy 4.2 Robot Control
//
// This firmware manages low-level hardware control for the Sigyn robot.
// It communicates with a host PC (e.g., running ROS2) via USB Serial.

#include "Arduino.h"
#include "BatteryModule.h"
#include "RoboClawModule.h"
#include "SdModule.h"
#include "SerialManager.h"
#include "TModule.h"
// #include "config.h"

BatteryModule& battery_module =
    BatteryModule::singleton();  // Battery monitoring module
RoboClawModule& roboclaw_module =
    RoboClawModule::singleton();              // RoboClaw motor control module
SdModule& sd_module = SdModule::singleton();  // SD card logging module

void setup() {
  Serial.begin(921600);  // Initialize USB Serial for communication with PC
  while (!Serial) {
  };  // Wait for serial port to connect (max 5s)

  TModule::Setup();  // Setup all registered modules
}

// Buffer for accumulating incoming serial data
String serialBuffer;

void loop() {
  // static unsigned long last_loop_time = 0;
  // unsigned long current_time = millis();
  
  // // Log if loop takes too long (more than 100ms since last iteration)
  // if (last_loop_time > 0 && (current_time - last_loop_time) > 100) {
  //   Serial.println("[TIMING] Loop gap detected: " + String(current_time - last_loop_time) + "ms at " + String(current_time));
  // }
  
  TModule::Loop();  // Call Loop on all registered modules
  
  // // Check if TModule::Loop took a long time
  // unsigned long after_tmodule = millis();
  // if ((after_tmodule - current_time) > 100) {
  //   Serial.println("[TIMING] TModule::Loop took " + String(after_tmodule - current_time) + "ms at " + String(after_tmodule));
  // }

  // Non-blocking serial read and message handling
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBuffer.length() > 0) {
        handleIncomingMessage(serialBuffer);
        serialBuffer = "";
      }
      // If multiple newlines in a row, just skip
    } else {
      serialBuffer += c;
    }
  }
  
  // last_loop_time = millis();
}

void handleIncomingMessage(const String& message) {
   // Parse message format: "TYPE:DATA"
  int colonIndex = message.indexOf(':');
  if (colonIndex == -1) return;

  String type = message.substring(0, colonIndex);
  String data = message.substring(colonIndex + 1);

  if (type == "TWIST") {
    roboclaw_module.handleTwistMessage(data);
  } else if (type == "SDDIR") {
    sd_module.handleDirMessage(data);
  } else if (type == "SDFILE") {
    sd_module.handleFileDump(data);
  } else {
    Serial.println("Unknown message type: " + type);
  }
}
