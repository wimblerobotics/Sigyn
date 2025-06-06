#include <Arduino.h>

#include "battery.h"
#include "roboclaw.h"
#include "tmodule.h"

RoboclawModule& roboclaw = RoboclawModule::singleton();
BatteryModule& battery = BatteryModule::singleton();

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for serial port to connect
  }

  TModule::DoSetup();

  Serial.println("Teensy communication bridge started");
}

void loop() {
  TModule::DoLoop();

  // Handle incoming serial messages
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    handleIncomingMessage(message);
  }

  delay(1);  // Small delay to prevent overwhelming the loop
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
