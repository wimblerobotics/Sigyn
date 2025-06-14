#pragma once

#include "Arduino.h"

#define MAX_SERIAL_BUFFER 64

class SerialManager {
 public:
  // Singleton constructor.
  static SerialManager &singleton();

  // to get data void SendModuleStats(TModule* module);
  void SendDiagnosticMessage(const String &message);

 private:
  SerialManager();  // Typically Serial for USB

  static SerialManager *g_singleton_;
};
