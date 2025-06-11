// SigynCore - Main Firmware for Teensy 4.2 Robot Control
//
// This firmware manages low-level hardware control for the Sigyn robot.
// It communicates with a host PC (e.g., running ROS2) via USB Serial.

#include "Arduino.h"
#include "BatteryModule.h"
#include "RoboClawModule.h"
#include "SerialManager.h"
#include "TModule.h"
#include "config.h"
#include "utils.h"

BatteryModule& battery_module =
    BatteryModule::singleton();  // Battery monitoring module
RoboClawModule& roboclaw_module =
    RoboClawModule::singleton();  // RoboClaw motor control module

// // Overall safety status
// bool global_e_stop_active = false;

void setup() {
  Serial.begin(921600);  // Initialize USB Serial for communication with PC
  while (!Serial) {
  };  // Wait for serial port to connect (max 5s)

  TModule::Setup();  // Setup all registered modules
}

// Buffer for accumulating incoming serial data
String serialBuffer;

void loop() {
  TModule::Loop();  // Call Loop on all registered modules

  // Non-blocking serial read and message handling
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBuffer.length() > 0) {
        handleIncomingMessage(serialBuffer);
        serialBuffer = "";
      }
      // If multiple newlines in a row, just skip
    } else {
      serialBuffer += c;
    }
  }
}

void handleIncomingMessage(const String& message) {
  // Parse message format: "TYPE:DATA"
  int colonIndex = message.indexOf(':');
  if (colonIndex == -1) return;

  String type = message.substring(0, colonIndex);
  String data = message.substring(colonIndex + 1);

  if (type == "TWIST") {
    roboclaw_module.handleTwistMessage(data);
  }
}
// void checkGlobalSafety() {
//     bool unsafe_condition_found = false;
//     String unsafe_reason = "";

//     for (int i = 0; i < num_modules; ++i) {
//         if (modules[i] && modules[i]->IsUnsafe()) {
//             unsafe_condition_found = true;
//             unsafe_reason = String("Module Unsafe: ") +
//             modules[i]->GetName(); break; // First unsafe condition triggers
//             action
//         }
//     }

//     if (unsafe_condition_found) {
//         if (!global_e_stop_active) {
//             activateGlobalEStop(unsafe_reason);
//         }
//     } else {
//         // If no module reports unsafe, and global E-Stop was active due to
//         modules,
//         // it means individual modules might have reset their flags.
//         // However, RoboClaw's E-Stop is reset by a zero twist command.
//         // This global_e_stop_active flag reflects the overall state.
//         // If RoboClaw itself clears its e_stop_active_ flag (e.g., via zero
//         Twist),
//         // then this global flag should also clear.
//         if (global_e_stop_active && !roboclaw_module.IsEStopActive()) {
//             // This case implies RoboClaw's E-Stop was reset (e.g. by zero
//             cmd_vel)
//             // and no other module is reporting unsafe.
//             resetGlobalEStop();
//         }
//     }
// }

// void activateGlobalEStop(const String& reason) {
//     global_e_stop_active = true;
//     // The RoboClaw module's triggerEStop handles the actual hardware (pin &
//     RoboClaw cmd)
//     // We ensure it's called if not already by its internal checks.
//     if (!roboclaw_module.IsEStopActive()) {
//         roboclaw_module.triggerEStop(reason.c_str()); // This will also send
//         a diagnostic
//     } else {
//         // If RoboClaw E-Stop is already active, just log the additional
//         global reason serial_manager.SendDiagnosticMessage(String("Global
//         E-Stop reinforced by: ") + reason);
//     }
//     // Command all motors to stop again, just in case.
//     roboclaw_module.SetTargetSpeed({0.0f, 0.0f});
// }

// void resetGlobalEStop() {
//     // Global E-Stop is reset if no module reports unsafe AND RoboClaw's
//     E-Stop has been reset.
//     // RoboClawModule's resetEStop() handles its specific flags and pin.
//     // This function primarily updates the global state flag.
//     if (global_e_stop_active) {
//         global_e_stop_active = false;
//         // serial_manager.SendDiagnosticMessage("Global E-Stop condition
//         cleared.");
//         // Modules should have reset their own safety flags if conditions
//         improved. for (int i = 0; i < num_modules; ++i) {
//             if (modules[i]) {
//                 modules[i]->ResetSafetyFlags();
//             }
//         }
//     }
// }
