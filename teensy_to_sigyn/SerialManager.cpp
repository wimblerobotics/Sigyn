#include "SerialManager.h"

SerialManager::SerialManager() {
    // Initialize serial port, buffer, and other members
    // serial_port_.begin(921600); // USB Serial for PC communication
    // buffer_index_ = 0;
    // new_command_available_ = false;
    // memset(serial_buffer_, 0, MAX_SERIAL_BUFFER);
    // last_received_twist_ = Twist();
    // loop_counter_ = 0;
}

// void SerialManager::processIncomingSerial() {
//     while (serial_port_.available() > 0) {
//         char received_char = serial_port_.read();
//         if (received_char == '\n' || received_char == '\r') { // Command terminator
//             if (buffer_index_ > 0) { // If buffer is not empty
//                 serial_buffer_[buffer_index_] = '\0'; // Null-terminate the string
//                 parseCommand(serial_buffer_);
//                 buffer_index_ = 0; // Reset buffer index for next command
//                 memset(serial_buffer_, 0, MAX_SERIAL_BUFFER); // Clear buffer
//             }
//         } else if (buffer_index_ < MAX_SERIAL_BUFFER - 1) {
//             serial_buffer_[buffer_index_++] = received_char;
//         } else {
//             // Buffer overflow, discard and reset
//             buffer_index_ = 0;
//             memset(serial_buffer_, 0, MAX_SERIAL_BUFFER);
//             // SendDiagnosticMessage("Serial buffer overflow");
//         }
//     }
// }

// void SerialManager::parseCommand(const char* command_str) {
//     // Example command: "v <linear_x> <angular_z>"
//     // Example: "v 0.5 0.2"
//     if (command_str[0] == 'v' && command_str[1] == ' ') {
//         float linear_x, angular_z;
//         // sscanf is convenient but can be heavy on microcontrollers.
//         // For robustness, manual parsing might be better.
//         if (sscanf(command_str + 2, "%f %f", &linear_x, &angular_z) == 2) {
//             last_received_twist_.linear_x = constrain_value(linear_x, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS);
//             last_received_twist_.angular_z = constrain_value(angular_z, -MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPS);
//             new_command_available_ = true;
//         } else {
//             // SendDiagnosticMessage("Invalid Twist cmd format");
//         }
//     }
//     // Add other command parsers here if needed
// }

// bool SerialManager::GetTwistCommand(Twist& cmd) {
//     if (new_command_available_) {
//         cmd = last_received_twist_;
//         new_command_available_ = false; // Command has been consumed
//         return true;
//     }
//     return false;
// }

// void SerialManager::SendOdometry(const Pose2D& pose, const Twist& velocity) {
//     // Format: "o <x_m> <y_m> <theta_rad> <vx_mps> <vy_mps> <vtheta_rps>"
//     // vy_mps is typically 0 for differential drive.
//     char msg_buffer[100]; // Ensure buffer is large enough
//     sprintf(msg_buffer, "o %.3f %.3f %.3f %.3f %.3f %.3f",
//             pose.x, pose.y, pose.theta,
//             velocity.linear_x, 0.0f, velocity.angular_z);
//     serial_port_.println(msg_buffer);
// }

void SerialManager::SendBatteryStatus(float voltage, float percentage) {
  Serial.print("BATTERY:");
  Serial.print(voltage);
  Serial.print(",");
  Serial.println(percentage);
}

void SerialManager::SendOdometry(const char* msg) {
    Serial.print("ODOM:");
    Serial.println(msg);
}


void SerialManager::SendRoboClawStatus(const char* msg) {
    Serial.print("ROBOCLAW:");
    Serial.println(msg);
}

// void SerialManager::SendModuleStats(TModule* module) {
//     if (!module) return;
//     // Format: "stat <module_name> <loop_calls> <min_us> <max_us> <avg_us>"
//     TModule::Stats stats = module->GetStatistics(); // Gets calculated average
//     char msg_buffer[100];
//     sprintf(msg_buffer, "stat %s %lu %lu %lu %.2f",
//             module->GetName(),
//             stats.loop_calls,
//             stats.min_loop_time_us,
//             stats.max_loop_time_us,
//             stats.avg_loop_time_us);
//     serial_port_.println(msg_buffer);
//     module->ResetStatistics(); // Reset after reporting
// }

void SerialManager::SendDiagnosticMessage(const String& message) {
    // Format: "diag <message_string>"
    // Handle multi-line messages by replacing newlines with spaces or sending as-is
    Serial.print("DIAG:");
    Serial.println(message);
}

SerialManager &SerialManager::singleton() {
    if (!g_singleton_) {
        g_singleton_ = new SerialManager();
    }
    return *g_singleton_;
}

SerialManager *SerialManager::g_singleton_ = nullptr;
