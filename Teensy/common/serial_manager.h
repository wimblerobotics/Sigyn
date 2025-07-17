#pragma once

#include "Arduino.h"

#define MAX_SERIAL_BUFFER 64

class SerialManager {
 public:
  // Singleton constructor.
  static SerialManager &singleton();

  void SendRoboClawStatus(const char *msg);
 
  // to get data void SendModuleStats(Module* module);
  void SendDiagnosticMessage(const String &message);

  void SendOdometry(const char *msg);

 private:
  SerialManager();  // Typically Serial for USB

  static SerialManager *g_singleton_;
};
