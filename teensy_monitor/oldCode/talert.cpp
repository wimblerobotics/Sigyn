#include "talert.h"

#include <Arduino.h>
#include <stdint.h>

#include "talarm.h"
#include "ton_off_button.h"
#include "trelay.h"

void TAlert::loop() {
  bool turnMotorsOff = false;
  for (uint8_t i = 0; i < (uint8_t)NUMER_ALERT_SOURCES; i++) {
    switch ((TAlertSource)i) {
      case CURRENT_LEFT_MOTOR:
        break;

      case CURRENT_RIGHT_MOTOR:
        break;

      case SONAR_BACK:
      case SONAR_FRONT:
      case SONAR_LEFT:
      case SONAR_RIGHT:
        if (g_alertsTriggered[i]) {
          turnMotorsOff = true;
        }

        break;

      case TEMP_LEFT_MOTOR:
        break;

      case TEMP_RIGHT_MOTOR:
        break;

      case TOF_LOWER_LEFT_BACKWARD:
      case TOF_LOWER_LEFT_SIDEWAY:
      case TOF_LOWER_RIGHT_BACKWARD:
      case TOF_LOWER_RIGHT_SIDEWAY:
      case TOF_UPPER_LEFT_FORWARD:
      case TOF_UPPER_LEFT_SIDEWAY:
      case TOF_UPPER_RIGHT_FORWARD:
      case TOF_UPPER_RIGHT_SIDEWAY:
        if (g_alertsTriggered[i]) {
          turnMotorsOff = true;
        }

        break;

      default:
        break;
    }  // switch
  }    // for

  if (turnMotorsOff) {
    TRelay::singleton().powerOff(TRelay::MOTOR_POWER);
    TAlarm::singleton().raise(TAlarm::MOTOR_ALARM);
    TOnOffButton::setState(1, TOnOffButton::OFF);
  }
}

void TAlert::reset(TAlertSource alert) {
  g_alertsTriggered[(uint8_t)alert] = false;
}

void TAlert::set(TAlertSource alert) {
  g_alertsTriggered[(uint8_t)alert] = true;
}

void TAlert::setup() {}

TAlert::TAlert() : TModule() {}

TAlert& TAlert::singleton() {
  if (!g_singleton) {
    g_singleton = new TAlert();
  }

  return *g_singleton;
}

bool TAlert::g_alertsTriggered[NUMER_ALERT_SOURCES];

TAlert* TAlert::g_singleton = nullptr;
