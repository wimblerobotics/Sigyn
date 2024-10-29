#include "trelay.h"

#include <Arduino.h>
#include <stdint.h>

#include "tconfiguration.h"
#include "tmicro_ros.h"

bool TRelay::IsPoweredOn(TRelayDevice device) {
  return g_device_set_time_ms_[device] != 0;
}

void TRelay::loop() {
  // TMicroRos::singleton().PublishDiagnostic("INFO [TRelay::loop]");
  // uint32_t now = millis();
  // if (g_device_set_time_ms_[kUnused1] &&
  //     (now > (g_device_set_time_ms_[kUnused1] + kResetDurationMs))) {
  //   digitalWrite(kUnused1Pin, LOW);
  //   g_device_set_time_ms_[kUnused1] = 0;
  // }
}

void TRelay::PowerOff(TRelay::TRelayDevice device) {
  char diagnostic_message[64];
  g_device_set_time_ms_[device] = 0;
  if (TM5::kDoDetailDebug) {
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TRelay::PowerOff] >> device: %d", (uint8_t)device);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  switch (device) {
      // case kUnused0:
      //   digitalWrite(kUnused0Pin, LOW);
      //   break;

      // case kUnused1:
      //   digitalWrite(kUnused1Pin, LOW);
      //   break;

      // case kUnused2:
      //   digitalWrite(kUnused2Pin, LOW);
      //   break;

      // case kUnused3:
      //   digitalWrite(kUnused3Pin, LOW);
      //   break;

    case kMotorEStop:
      digitalWrite(kMotorEStopPin, LOW);
      break;

    default:
      break;
  }
}

void TRelay::PowerOn(TRelay::TRelayDevice device) {
  char diagnostic_message[64];
  g_device_set_time_ms_[device] = 0;
  if (TM5::kDoDetailDebug) {
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TRelay::PowerOn] >> device: %d", (uint8_t)device);
    TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }

  switch (device) {
      // case kUnused0:
      //   digitalWrite(kUnused0Pin, HIGH);
      //   break;

      // case kUnused1:
      //   digitalWrite(kUnused1Pin, HIGH);
      //   break;

      // case kUnused2:
      //   digitalWrite(kUnused2Pin, HIGH);

      //   // Manually resetting the motor power will also reset the overcurrent
      //   // alarm.
      //    break;

      // case kUnused3:
      //   digitalWrite(kUnused3Pin, HIGH);
      //   break;

    case kMotorEStop:
      digitalWrite(kMotorEStopPin, HIGH);
      break;

    default:
      break;
  }

  g_device_set_time_ms_[device] = millis();
}

void TRelay::setup() {
  if (TM5::kDoDetailDebug) {
    TMicroRos::singleton().PublishDiagnostic("INFO [TRelay::setup] >>enter");
  }

  // pinMode(kUnused0Pin, OUTPUT);
  // digitalWrite(kUnused0Pin, LOW);
  // g_device_set_time_ms_[kUnused0] = millis();

  // pinMode(kUnused1Pin, OUTPUT);
  // digitalWrite(kUnused1Pin, LOW);
  // g_device_set_time_ms_[kUnused1] = 0;

  // pinMode(kUnused2Pin, OUTPUT);
  // digitalWrite(kUnused2Pin, LOW);
  // g_device_set_time_ms_[kUnused2] = 0;

  // pinMode(kUnused3Pin, OUTPUT);
  // digitalWrite(kUnused3Pin, LOW);
  // g_device_set_time_ms_[kUnused3] = millis();

  pinMode(kMotorEStopPin, OUTPUT);
  digitalWrite(kMotorEStopPin, LOW);
  g_device_set_time_ms_[kMotorEStop] = 0;
}

TRelay::TRelay() : TModule(TModule::kRelay) {}

TRelay& TRelay::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TRelay();
  }

  return *g_singleton_;
}

uint32_t TRelay::g_device_set_time_ms_[TRelay::kNumberDevices];

TRelay* TRelay::g_singleton_ = nullptr;
