#include "SerialManager.h"

#include "SdModule.h"
// #include "config.h"

SerialManager::SerialManager() {}

void SerialManager::SendDiagnosticMessage(const String& message) {
  // Format: "diag <message_string>"
  // Handle multi-line messages by replacing newlines with spaces or sending
  // as-is
  Serial.print("DIAG:");
  Serial.println(message);
  String logMsg = String("DIAG:" + message);
  SdModule::singleton().log(logMsg.c_str());
}

SerialManager& SerialManager::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new SerialManager();
  }
  return *g_singleton_;
}

SerialManager* SerialManager::g_singleton_ = nullptr;
