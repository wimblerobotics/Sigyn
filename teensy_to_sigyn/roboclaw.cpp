#include "roboclaw.h"

RoboclawModule::RoboclawModule() : TModule(TModule::kRoboClaw){
    linear_x = 0.0;
    angular_z = 0.0;
    last_twist_time = 0;
    last_status_send_time = 0;
    status_send_interval = 100; // Send status every 100ms
}

void RoboclawModule::setup() {
}

void RoboclawModule::loop() {
    unsigned long current_time = millis();
    
    // Send status message at configured rate
    if (current_time - last_status_send_time >= status_send_interval) {
        sendStatusMessage();
        last_status_send_time = current_time;
    }
    
    // TODO: Add actual motor control code here
    // For now, just cache the latest twist message
}

void RoboclawModule::handleTwistMessage(const String& data) {
    // Parse twist data format: "linear_x,angular_z"
    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
        linear_x = data.substring(0, commaIndex).toFloat();
        angular_z = data.substring(commaIndex + 1).toFloat();
        last_twist_time = millis();
        
        // Serial.print("Received twist: linear_x=");
        // Serial.print(linear_x);
        // Serial.print(", angular_z=");
        // Serial.println(angular_z);
    }
}

void RoboclawModule::sendStatusMessage() {
    // Send a simple status message back to the robot
    Serial.print("STATUS:Roboclaw OK, last_cmd_age=");
    Serial.print(millis() - last_twist_time);
    Serial.print("ms");
    Serial.print(", linear_x=");
    Serial.print(linear_x);
    Serial.print(", angular_z=");
    Serial.println(angular_z);
}

RoboclawModule& RoboclawModule::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new RoboclawModule();
  }

  return *g_singleton_;
}

RoboclawModule* RoboclawModule::g_singleton_ = nullptr;