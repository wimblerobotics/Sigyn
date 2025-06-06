#include <Arduino.h>

#include "tmodule.h"

class DiagnosticMessage : TModule {
 public:
  // Singleton constructor.
  static DiagnosticMessage &singleton();

  // Send a diagnostic message.
  void sendMessage(const char *msg);

 protected:
  // From TModule.
  void loop();

  // From TModule.
  const char *name() { return "Diag"; }

  // From TModule.
  void setup();

 private:
  DiagnosticMessage();

  // Singleton instance.
  static DiagnosticMessage *g_singleton_;
};
