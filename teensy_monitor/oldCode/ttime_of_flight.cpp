#include "ttime_of_flight.h"

#include <VL53L0X.h>
#include <Wire.h>
#include <stdint.h>

#include "talert.h"

int TTimeOfFlight::getValueMm(TIMEOFFLIGHT device) {
  if (device >= NUMBER_TIME_OF_FLIGHT) {
    return -1;
  }

  // Get the sensor values only once every give number of milli seconds.
  static uint8_t lastSensedIndex = 0;
  static const unsigned long GIVEN_NUMBER_OF_MILLISECONDS = 10;
  static unsigned long lastWallTime = millis();
  unsigned long currentWallTime = millis();
  unsigned long durationSinceLastSense = currentWallTime - lastWallTime;

  if (durationSinceLastSense > GIVEN_NUMBER_OF_MILLISECONDS) {
    if (g_sensor[lastSensedIndex] != nullptr) {
      selectTimeOfFlightSensor(static_cast<TIMEOFFLIGHT>(lastSensedIndex));
      g_cachedValue[lastSensedIndex] =
          g_sensor[lastSensedIndex]->readRangeContinuousMillimeters();
    }

    lastSensedIndex += 1;
    if (lastSensedIndex >= NUMBER_TIME_OF_FLIGHT) {
      lastSensedIndex = 0;
    }

    lastWallTime = currentWallTime;
  }

  if (g_sensor[device]) {
    return g_cachedValue[device];
  } else {
    return -1;
  }
}

void TTimeOfFlight::loop() {
  static const TAlert::TAlertSource map[] = {
      TAlert::TOF_UPPER_LEFT_FORWARD,  TAlert::TOF_UPPER_RIGHT_FORWARD,
      TAlert::TOF_UPPER_LEFT_SIDEWAY,  TAlert::TOF_UPPER_RIGHT_SIDEWAY,
      TAlert::TOF_LOWER_LEFT_SIDEWAY,  TAlert::TOF_LOWER_RIGHT_SIDEWAY,
      TAlert::TOF_LOWER_LEFT_BACKWARD, TAlert::TOF_LOWER_RIGHT_BACKWARD};

  for (uint8_t i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
    int mm = getValueMm(static_cast<TIMEOFFLIGHT>(i));
    if (doStopMotorsOnCollisionThreat && (mm != -1) && (mm < ALERT_DISTANCE_MM)) {
      TAlert::singleton().set(map[i]);
    } else {
      TAlert::singleton().reset(map[i]);
    }
  }
}

void TTimeOfFlight::selectTimeOfFlightSensor(TIMEOFFLIGHT device) {
  if (device >= NUMBER_TIME_OF_FLIGHT) return;

  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  Wire.write(1 << device);
  Wire.endTransmission();
}

void TTimeOfFlight::setup() {
  uint8_t numberSensorsFound = 0;
  for (uint8_t i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
    selectTimeOfFlightSensor(static_cast<TIMEOFFLIGHT>(i));
    VL53L0X *sensor = new VL53L0X();
    sensor->setTimeout(500);
    if (sensor->init()) {
      g_sensor[i] = sensor;
      numberSensorsFound++;
      sensor->setMeasurementTimingBudget(20000);
      sensor->startContinuous();
    } else {
      g_sensor[i] = nullptr;
    }
  }
}

TTimeOfFlight::TTimeOfFlight() {
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  for (int i = 0; i < NUMBER_TIME_OF_FLIGHT; i++) {
    g_cachedValue[i] = -1;
  }
}

TTimeOfFlight &TTimeOfFlight::singleton() {
  if (!g_singleton) {
    g_singleton = new TTimeOfFlight();
  }

  return *g_singleton;
}

TTimeOfFlight *TTimeOfFlight::g_singleton = nullptr;

int TTimeOfFlight::g_cachedValue[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT];
VL53L0X *TTimeOfFlight::g_sensor[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT];
