#include "SerialManager.h"

#include "SdModule.h"

SerialManager::SerialManager() {}

void SerialManager::SendBatteryStatus(float voltage, float percentage) {
  Serial.print("BATTERY:");
  Serial.print(voltage);
  Serial.print(",");
  Serial.println(percentage);
  String logMsg =
      String("BATTERY:") + String(voltage) + "V, " + String(percentage) + "%";
  SdModule::singleton().log(logMsg.c_str());
}

void SerialManager::SendOdometry(const char* msg) {
  Serial.print("ODOM:");
  Serial.println(msg);
  String logMsg = String("ODOM:") + String(msg);
  SdModule::singleton().log(logMsg.c_str());
}

void SerialManager::SendRoboClawStatus(const char* msg) {
  Serial.print("ROBOCLAW:");
  Serial.println(msg);
  String logMsg = String("ROBOCLAW:") + String(msg);
  SdModule::singleton().log(logMsg.c_str());
}

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
