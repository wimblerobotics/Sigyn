#include "analog_test.h"

#include <stdint.h>

const char* TAnalogTest::deviceDescription(uint8_t deviceIndex) {
  if (deviceIndex >= numberOfDevices()) {
    return "Unknown device";
  }

  switch (deviceIndex) {
    case 0:
      return "Analog Sensor 0";
    case 1:
      return "Analog Sensor 1";
    case 2:
      return "Analog Sensor 2";
    case 3:
      return "Analog Sensor 3";
    case 4:
      return "Analog Sensor 4";
    case 5:
      return "Analog Sensor 5";
    case 6:
      return "Analog Sensor 6";
    case 7:
      return "Analog Sensor 7";
    default:
      return "bad device index";
  }
}

const char* TAnalogTest::deviceName(uint8_t deviceIndex) {
  if (deviceIndex >= numberOfDevices()) {
    return "Unknown";
  }

  switch (deviceIndex) {
    case 0:
      return "Analog0";
    case 1:
      return "Analog1";
    case 2:
      return "Analog2";
    case 3:
      return "Analog3";
    case 4:
      return "Analog4";
    case 5:
      return "Analog5";
    case 6:
      return "Analog6";
    case 7:
      return "Analog7";
    default:
      return "bad device index";
  }
}

const int TAnalogTest::numberOfDevices() { return 8; }

const int TAnalogTest::numberOfTestsPerDevice() {
  // Assuming each device has 1 test for simplicity
  return 1;
}

bool TAnalogTest::runTest(uint8_t testDevice, uint8_t testIndex) {
  if (testDevice >= numberOfDevices() ||
      testIndex >= numberOfTestsPerDevice()) {
    return false;  // Invalid device or test index
  }
  int raw = value(testDevice);

  // For simplicity, we will just print the value to the serial monitor
  // Serial.print("Test Device: ");
  // Serial.print(testDevice);
  // Serial.print(", Test Index: ");
  // Serial.print(testIndex);
  Serial.print(", Value: ");
  Serial.println(raw);

  return true;  // Indicate that the test was run successfully
}
const char* TAnalogTest::testName(uint8_t testIndex) {
  switch (testIndex) {
    case 0:
      return "Analog Read Test";
    default:
      return "Unknown test";
  }
}

int16_t TAnalogTest::value(uint8_t deviceIndex) {
  // This function should read the analog value from the specified device
  // For simplicity, we will return a dummy value here
  // In a real implementation, you would read from the corresponding analog pin
  switch (deviceIndex) {
    case 0:
      return analogRead(TAnalogTest::ANALOG_0_PIN);
    case 1:
      return analogRead(TAnalogTest::ANALOG_1_PIN);
    case 2:
      return analogRead(TAnalogTest::ANALOG_2_PIN);
    case 3:
      return analogRead(TAnalogTest::ANALOG_3_PIN);
    case 4:
      return analogRead(TAnalogTest::ANALOG_4_PIN);
    case 5:
      return analogRead(TAnalogTest::ANALOG_5_PIN);
    case 6:
      return analogRead(TAnalogTest::ANALOG_6_PIN);
    case 7:
      return analogRead(TAnalogTest::ANALOG_7_PIN);
    default:
      return -1;  // Invalid device index
  }
}

// Note: Ensure that the analog pins are set up correctly in the setup function

TAnalogTest::TAnalogTest() : TTestModule() {
  analogReadResolution(10);
  pinMode(TAnalogTest::ANALOG_0_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_1_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_2_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_3_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_4_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_5_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_6_PIN, INPUT);
  pinMode(TAnalogTest::ANALOG_7_PIN, INPUT);
  // analogReadAveraging(4);  // Average 4 samples for smoother readings
}

void TAnalogTest::setup() {
  // Setup code for analog test can be added here if needed
  // For example, initializing analog pins or other configurations
}

TAnalogTest& TAnalogTest::singleton() {
  if (!g_singleton) {
    g_singleton = new TAnalogTest();
  }
  return *g_singleton;
}

TAnalogTest* TAnalogTest::g_singleton = nullptr;

// Add any missing declarations or definitions
// Ensure proper closure of the file