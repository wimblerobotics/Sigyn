#include <Wire.h>

#include "Arduino.h"
#include "bno055_module.h"
#include "module.h"
#include "serial_manager.h"

BNO055Module& bno055_module =
    BNO055Module::singleton();  // BNO055 sensor module

void setup() {
  // Wire.begin();
  Serial.begin(115200);  // Initialize USB Serial for communication with PC
  while (!Serial) {
  };  // Wait for serial port to connect (max 5s)

  Module::Setup();  // Setup all registered modules
}

void loop() {
  Module::Loop();  // Call Loop on all registered modules

  // Non-blocking serial read and message handling
  String serialBuffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBuffer.length() > 0) {
        // Handle the incoming message
        handleIncomingMessage(serialBuffer);
        serialBuffer = "";
      }
      // If multiple newlines in a row, just skip
    } else {
      serialBuffer += c;
    }
  }
}

void handleIncomingMessage(const String& message) {
  // Parse message format: "TYPE:DATA"
  int colonIndex = message.indexOf(':');
  if (colonIndex == -1) return;

  String type = message.substring(0, colonIndex);
  String data = message.substring(colonIndex + 1);

  if (type == "IMU") {
    // Handle IMU commands via BNO055Module
    bno055_module.handleCommand(data);
  } else if (type == "TWIST") {
    // roboclaw_module.handleTwistMessage(data);
  } else if (type == "SDDIR") {
    // sd_module.handleDirMessage(data);
  } else if (type == "SDFILE") {
    // sd_module.handleFileDump(data);
  } else {
    Serial.println("Unknown message type: " + type);
  }
}
