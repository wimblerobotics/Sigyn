
#include "serial_manager.h"

#include "config.h"
#include "sd_module.h"

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

  // Odometry messages would fill up the SD card quickly, so we skip some.
  static uint16_t odometry_log_skip_count = ODOMETRY_LOG_SKIP_INTERVAL;
  if (odometry_log_skip_count == 0) {
    SdModule::singleton().log(logMsg.c_str());
    odometry_log_skip_count = ODOMETRY_LOG_SKIP_INTERVAL;
  } else {
    odometry_log_skip_count--;
  }
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
