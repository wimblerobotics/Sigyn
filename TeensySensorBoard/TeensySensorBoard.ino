#include "Arduino.h"
#include "analog_test.h"

TAnalogTest& analogTest = TAnalogTest::singleton();

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize sensors and other components here
  // For example, if using a temperature sensor:
  // pinMode(TEMP_SENSOR_PIN, INPUT);

  // Print a message to indicate setup is complete
  Serial.println("Teensy Sensor Board setup complete.");

  Serial.println("Number of devices: " + String(analogTest.numberOfDevices()));
  for (uint8_t i = 0; i < analogTest.numberOfDevices(); i++) {
    Serial.println("Device " + String(i) + ": " + analogTest.deviceName(i));
    Serial.println("  Description: " + String(analogTest.deviceDescription(i)));
    Serial.println("  Number of tests: " +
                   String(analogTest.numberOfTestsPerDevice()));
    for (uint8_t j = 0; j < analogTest.numberOfTestsPerDevice(); j++) {
      Serial.println("    Test " + String(j) + ": " + analogTest.testName(j));
      if (analogTest.runTest(i, j)) {
        Serial.println("      Test passed.");
      } else {
        Serial.println("  Test failed.");
      }
    }
  }
}

void loop() {
  // Read sensor data
  // For example, if reading a temperature sensor:
  // int temperature = analogRead(TEMP_SENSOR_PIN);

  // Print the sensor data to the serial monitor
  // Serial.print("Temperature: ");
  // Serial.println(temperature);

  // Add a delay for readability
  delay(1000);
}