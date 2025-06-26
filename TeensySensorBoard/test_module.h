#pragma once
#include <Arduino.h>

class TTestModule {
 public:
  static TTestModule& singleton();

  virtual const int numberOfDevices() { return 0; }
  virtual void getDeviceName(uint8_t deviceIndex, String& name) { strcpy }
  virtual const char* getDeviceDescription(uint8_t deviceIndex) { return nullptr; }
  virtual const int numberOfTestsPerDevice() { return 0; }
  virtual const char* getTestName(uint8_t testIndex) { return nullptr; }
  virtual const char* getTestDescription(uint8_t testIndex) { return nullptr; }
  virtual bool runTest(uit8_t testDevice, uint8_t testIndex) { return false; }

  virtual void setup() {}
  //   void loop();

 private:
  TTestModule();
  static TTestModule* g_singleton;
};