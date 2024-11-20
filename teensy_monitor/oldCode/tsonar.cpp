#include "tsonar.h"

#include <stdint.h>

#include "Arduino.h"
#include "TimerOne.h"
#include "Wire.h"
#include "talert.h"

void TSonar::echo0InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO0)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[0] = (endTime - startTime) * g_TimeToMmScaler;
      break;
  }
}

void TSonar::echo1InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO1)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[1] = (endTime - startTime) * g_TimeToMmScaler;
      break;
  }
}

void TSonar::echo2InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO2)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[2] = (endTime - startTime) * g_TimeToMmScaler;
      break;
  }
}

void TSonar::echo3InterruptHandler() {
  static long endTime = 0;
  static long startTime = 0;
  switch (digitalRead(PIN_ECHO3)) {
    case HIGH:
      endTime = 0;
      startTime = micros();
      break;

    case LOW:
      endTime = micros();
      g_valuesMm[3] = (endTime - startTime) * g_TimeToMmScaler;
      break;
  }
}

int TSonar::getValueMm(SONAR device) {
  if (static_cast<int>(device) >= NUMBER_SONARS) {
    return -1;
  } else {
    return g_valuesMm[static_cast<int>(device)];
  }
}

void TSonar::loop() {
  const int ALERT_DISTANCE_MM = 3 * 25.4;
  TAlert::TAlertSource map[] = {TAlert::SONAR_FRONT, TAlert::SONAR_RIGHT,
                                TAlert::SONAR_BACK, TAlert::SONAR_LEFT};

  for (uint8_t i = 0; i < NUMBER_SONARS; i++) {
    if (doStopMotorsOnCollisionThreat && (getValueMm(static_cast<SONAR>(i)) < ALERT_DISTANCE_MM)) {
      TAlert::singleton().set(map[i]);
    } else {
      TAlert::singleton().reset(map[i]);
    }
  }
}

void TSonar::setup() {
  pinMode(PIN_ECHO0, INPUT);
  pinMode(PIN_TRIGGER0, OUTPUT);
  pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_TRIGGER1, OUTPUT);
  pinMode(PIN_ECHO2, INPUT);
  pinMode(PIN_TRIGGER2, OUTPUT);
  pinMode(PIN_ECHO3, INPUT);
  pinMode(PIN_TRIGGER3, OUTPUT);
  Timer1.initialize(TIMER_PERIOD_USEC);
  Timer1.attachInterrupt(timerInterruptHandler);
  attachInterrupt(PIN_ECHO0, echo0InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO1, echo1InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO2, echo2InterruptHandler, CHANGE);
  attachInterrupt(PIN_ECHO3, echo3InterruptHandler, CHANGE);
}

void TSonar::timerInterruptHandler() {
  typedef enum { COUNTDOWN, PULSE_HIGH, PULSE_LOW } TTimerState;

  static volatile TTimerState state = COUNTDOWN;
  static volatile long countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;

  if (--countdown == 0) {
    state = PULSE_HIGH;
    countdown = TIMER_COUNTS_PER_SAMPLING_PERIOD;
  }

  switch (state) {
    case COUNTDOWN:
      break;

    case PULSE_HIGH:
      if ((g_nextSensorIndex % 4) == 0) {
        digitalWrite(PIN_TRIGGER0, HIGH);
      } else if ((g_nextSensorIndex % 4) == 1) {
        digitalWrite(PIN_TRIGGER1, HIGH);
      } else if ((g_nextSensorIndex % 4) == 2) {
        digitalWrite(PIN_TRIGGER2, HIGH);
      } else {
        digitalWrite(PIN_TRIGGER3, HIGH);
      }

      state = PULSE_LOW;
      break;

    case PULSE_LOW:
      if ((g_nextSensorIndex % 4) == 0) {
        digitalWrite(PIN_TRIGGER0, LOW);
      } else if ((g_nextSensorIndex % 4) == 1) {
        digitalWrite(PIN_TRIGGER1, LOW);
      } else if ((g_nextSensorIndex % 4) == 2) {
        digitalWrite(PIN_TRIGGER2, LOW);
      } else {
        digitalWrite(PIN_TRIGGER3, LOW);
      }

      g_nextSensorIndex++;
      state = COUNTDOWN;
      break;
  }
}

TSonar::TSonar() {}

TSonar& TSonar::singleton() {
  if (!g_singleton) {
    g_singleton = new TSonar();
  }

  return *g_singleton;
}

uint8_t TSonar::g_nextSensorIndex = 0;

TSonar* TSonar::g_singleton = nullptr;

int TSonar::g_valuesMm[TSonar::NUMBER_SONARS] = {-1, -1, -1, -1};

const float TSonar::g_TimeToMmScaler = (10.0 / 2.0) / 29.1;