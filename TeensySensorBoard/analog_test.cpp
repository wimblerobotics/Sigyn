#include "analog_test.h"

#include <stdint.h>

const int TAnalogTest::numberOfDevices() { return 4;}
void TAnalogTest::getDeviceName(uint8_t deviceIndex, String& name) {
    if (deviceIndex >= numberOfDevices()) {
        name = "Unknown";
        return;
    }
    
    String result = "Analog" + String(deviceIndex);
    name = result;
}

TAnalogTest& TAnalogTest::singleton() {
  if (!g_singleton) {
    g_singleton = new TAnalogTest();
  }
  return *g_singleton;
}

int16_t TAnalogTest::getValueTenthsC(uint8_t device) {
  switch (device) {
    case 0:
      return g_analog0Value;

    case 1:
      return g_analog1Value;

    default:
      return -1;
  }
}