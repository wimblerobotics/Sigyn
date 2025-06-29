#pragma once

#include <Arduino.h>

class TTestModule {
 public:
  virtual ~TTestModule() {}

  static TTestModule& singleton();

  virtual const char* deviceDescription(uint8_t deviceIndex) = 0;

  virtual const char* deviceName(uint8_t deviceIndex) = 0;

  virtual const int numberOfDevices() = 0;

  virtual const int numberOfTestsPerDevice() = 0;

  virtual bool runTest(uint8_t testDevice, uint8_t testIndex) = 0;

  virtual const char* testName(uint8_t testIndex) = 0;
  
 protected:
  TTestModule();

  virtual void setup() = 0;

  static TTestModule* g_singleton;
};