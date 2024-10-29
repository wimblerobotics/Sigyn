#pragma once

#include "tmodule.h"

class TAlert : TModule {
 public:
  // List of possible alert sources.
  typedef enum TAlertSource {
    CURRENT_LEFT_MOTOR,
    CURRENT_RIGHT_MOTOR,
    SONAR_BACK,
    SONAR_FRONT,
    SONAR_LEFT,
    SONAR_RIGHT,
    TEMP_LEFT_MOTOR,
    TEMP_RIGHT_MOTOR,
    TOF_LOWER_LEFT_BACKWARD,
    TOF_LOWER_LEFT_SIDEWAY,
    TOF_LOWER_RIGHT_BACKWARD,
    TOF_LOWER_RIGHT_SIDEWAY,
    TOF_UPPER_LEFT_FORWARD,
    TOF_UPPER_LEFT_SIDEWAY,
    TOF_UPPER_RIGHT_FORWARD,
    TOF_UPPER_RIGHT_SIDEWAY,
    NUMER_ALERT_SOURCES // Number of alert sources.
  } TAlertSource;

  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "TAlert"; }

  // Set the alert.
  void set(TAlertSource alert);

  // From TModule.
  void setup();

  // Singleton constructor.
  static TAlert& singleton();

  // Unset the alert.
  void reset(TAlertSource alert);

 private:
  // Private constructor.
  TAlert();
  
  // State of each possible alert.
  static bool g_alertsTriggered[NUMER_ALERT_SOURCES];

  // Singleton instance.
  static TAlert* g_singleton;
};
