#include <Arduino.h>

#include "battery.h"
#include "diagnostic_message.h"
#include "roboclaw.h"
#include "tmodule.h"

RoboclawModule& roboclaw = RoboclawModule::singleton();
// BatteryModule& battery = BatteryModule::singleton();
DiagnosticMessage& diagnostic_message = DiagnosticMessage::singleton();

// Buffer for accumulating incoming serial data
String serialBuffer;

void setup() {
  Serial.begin(921600);
  while (!Serial) {
    ;  // Wait for serial port to connect
  }

  TModule::DoSetup();

  Serial.println("Teensy communication bridge started");
}

void loop() {
  TModule::DoLoop();

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

  // delay(1);  // Small delay to prevent overwhelming the loop
}

void handleIncomingMessage(const String& message) {
  // Parse message format: "TYPE:DATA"
  int colonIndex = message.indexOf(':');
  if (colonIndex == -1) return;

  String type = message.substring(0, colonIndex);
  String data = message.substring(colonIndex + 1);

  if (type == "TWIST") {
    roboclaw.handleTwistMessage(data);
  }
  // Add more message types as needed
}
