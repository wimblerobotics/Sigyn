#pragma once

#include <SD.h>

#include "tmodule.h"

class TSd : TModule {
 public:
  // Write message to log file.
  void log(const char* message);

  // From TModule.
  void loop();

  // From TModule.
  virtual const char* name() { return "TSd"; }

  void setup();

  // Singleton constructor.
  static TSd& singleton();

 private:
  // Private constructor.
  TSd();

  static SDClass g_sd;

  // Has SD device been properly initialized?
  static bool g_initialized;

  static File g_logFile;

  // Singleton instance.
  static TSd* g_singleton;
};