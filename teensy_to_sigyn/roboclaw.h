#ifndef ROBOCLAW_H
#define ROBOCLAW_H

#include "tmodule.h"
#include <Arduino.h>

class RoboclawModule : TModule {
public:
  // Singleton constructor.
  static RoboclawModule &singleton();

protected:
  // From TModule.
  void loop();

  // From TModule.
  const char *name() { return "Batt"; }

  // From TModule.
  void setup();

private:
  RoboclawModule();
  void sendStatusMessage();
  float linear_x;
  float angular_z;
  unsigned long last_twist_time;
  unsigned long last_status_send_time;
  unsigned long status_send_interval; // milliseconds

  // Singleton instance.
  static RoboclawModule *g_singleton_;

public:
  void handleTwistMessage(const String &data);
};

#endif