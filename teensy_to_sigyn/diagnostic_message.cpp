#include "diagnostic_message.h"

DiagnosticMessage::DiagnosticMessage() : TModule(TModule::kDiagnosticMessage) {
  // Constructor code here if needed.
}

void DiagnosticMessage::loop() {
  // This module does not need to do anything in the loop.
}

void DiagnosticMessage::setup() {
  // This module does not need to do anything in the setup.
  // It is only used to send diagnostic messages.
}

void DiagnosticMessage::sendMessage(const char *msg) {
  Serial.print("DiagnosticMessage: ");
  Serial.println(msg);
  // #if USE_TSD
  //     TSd::singleton().log(msg);
  // #endif
}

DiagnosticMessage &DiagnosticMessage::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new DiagnosticMessage();
  }
  return *g_singleton_;
}

DiagnosticMessage *DiagnosticMessage::g_singleton_ = nullptr;
