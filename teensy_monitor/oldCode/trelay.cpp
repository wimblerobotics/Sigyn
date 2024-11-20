#include "trelay.h"

#include <Arduino.h>
#include <stdint.h>

#include "talarm.h"

bool TRelay::isPoweredOn(TRelayDevice device) {
  return g_deviceSetTimeMs[device] != 0;
}

void TRelay::loop() {
  uint32_t now = millis();
  if (g_deviceSetTimeMs[INTEL_RESET] &&
      (now > (g_deviceSetTimeMs[INTEL_RESET] + RESET_DURATION_MS))) {
    digitalWrite(INTEL_RESET_PIN, LOW);
    g_deviceSetTimeMs[INTEL_RESET] = 0;
  }

  if (g_deviceSetTimeMs[NVIDIA_RESET] &&
      (now > (g_deviceSetTimeMs[NVIDIA_RESET] + RESET_DURATION_MS))) {
    digitalWrite(NVIDIA_RESET_PIN, LOW);
    g_deviceSetTimeMs[NVIDIA_RESET] = 0;
  }
}

void TRelay::powerOff(TRelay::TRelayDevice device) {
  g_deviceSetTimeMs[device] = 0;
  switch (device) {
    case INTEL_POWER:
      digitalWrite(INTEL_ON_OFF_PIN, LOW);
      break;

    case INTEL_RESET:
      digitalWrite(INTEL_RESET_PIN, LOW);
      break;

    case MOTOR_POWER:
      digitalWrite(MOTOR_ON_OFF_PIN, LOW);
      break;

    case NVIDIA_POWER:
      digitalWrite(NVIDIA_ON_OFF_PIN, LOW);
      break;

    case NVIDIA_RESET:
      digitalWrite(NVIDIA_RESET_PIN, LOW);
      break;

    case MOTOR_ESTOP:
      digitalWrite(MOTOR_ESTOP_PIN, LOW);
      break;

    default:
      break;
  }
}

void TRelay::powerOn(TRelay::TRelayDevice device) {
  switch (device) {
    case INTEL_POWER:
      digitalWrite(INTEL_ON_OFF_PIN, HIGH);
      break;

    case INTEL_RESET:
      digitalWrite(INTEL_RESET_PIN, HIGH);
      break;

    case MOTOR_POWER:
      digitalWrite(MOTOR_ON_OFF_PIN, HIGH);

      // Manually resetting the motor power will also reset the overcurrent
      // alarm.
      TAlarm::singleton().reset(TAlarm::MOTOR_ALARM);
      break;

    case NVIDIA_POWER:
      digitalWrite(NVIDIA_ON_OFF_PIN, HIGH);
      break;

    case NVIDIA_RESET:
      digitalWrite(NVIDIA_RESET_PIN, HIGH);
      break;

    case MOTOR_ESTOP:
      digitalWrite(MOTOR_ESTOP_PIN, HIGH);
      break;

    default:
      break;
  }

  g_deviceSetTimeMs[device] = millis();
}

void TRelay::setup() {
  pinMode(INTEL_ON_OFF_PIN, OUTPUT);
  digitalWrite(INTEL_ON_OFF_PIN, LOW);
  g_deviceSetTimeMs[INTEL_POWER] = millis();

  pinMode(INTEL_RESET_PIN, OUTPUT);
  digitalWrite(INTEL_RESET_PIN, LOW);
  g_deviceSetTimeMs[INTEL_RESET] = 0;

  pinMode(MOTOR_ON_OFF_PIN, OUTPUT);
  digitalWrite(MOTOR_ON_OFF_PIN, LOW);
  g_deviceSetTimeMs[MOTOR_POWER] = 0;

  pinMode(NVIDIA_ON_OFF_PIN, OUTPUT);
  digitalWrite(NVIDIA_ON_OFF_PIN, LOW);
  g_deviceSetTimeMs[NVIDIA_POWER] = millis();

  pinMode(NVIDIA_RESET_PIN, OUTPUT);
  digitalWrite(NVIDIA_RESET_PIN, LOW);
  g_deviceSetTimeMs[NVIDIA_RESET] = 0;

  pinMode(MOTOR_ESTOP_PIN, OUTPUT);
  digitalWrite(MOTOR_ESTOP_PIN, LOW);
  g_deviceSetTimeMs[MOTOR_ESTOP] = 0;
}

TRelay::TRelay() {}

TRelay& TRelay::singleton() {
  if (!g_singleton) {
    g_singleton = new TRelay();
  }

  return *g_singleton;
}

uint32_t TRelay::g_deviceSetTimeMs[TRelay::NUMBER_DEVICES];

TRelay* TRelay::g_singleton = nullptr;
