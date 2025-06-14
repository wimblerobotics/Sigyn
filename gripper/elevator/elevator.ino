#include "Arduino.h"
#include "MotorModule.h"
#include "SdModule.h"
#include "SerialManager.h"
#include "TModule.h"

MotorModule& motor_module = MotorModule::singleton();  // Motor control module
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
   TModule::Loop();  // Call Loop on all registered modules

  // // Check if TModule::Loop took a long time
  // unsigned long after_tmodule = millis();
  // if ((after_tmodule - current_time) > 100) {
  //   Serial.println("[TIMING] TModule::Loop took " + String(after_tmodule -
  //   current_time) + "ms at " + String(after_tmodule));
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
    motor_module.handleTwistMessage(data);
  } else if (type == "SDDIR") {
    sd_module.handleDirMessage(data);
  } else if (type == "SDFILE") {
    sd_module.handleFileDump(data);
  } else {
    Serial.println("Unknown message type: " + type);
  }
}