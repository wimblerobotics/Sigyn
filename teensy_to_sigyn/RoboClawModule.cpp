#include "RoboClawModule.h"

#include "Arduino.h"        // For micros, millis, pinMode, digitalWrite
#include "RoboClaw.h"       // External RoboClaw library
#include "SerialManager.h"  // For sending diagnostic messages

RoboClawModule::RoboClawModule()
    : TModule(),
      current_state_(State::kReconnect),
      roboclaw_(RoboClaw(&ROBOCLAW_SERIAL, ROBOCLAW_TIMEOUT_US)),
      angular_z_(0.0f),
      linear_x_(0.0f),
      last_twist_received_time_ms_(0),
      last_commanded_qpps_m1_(0),
      last_commanded_qpps_m2_(0),
      last_command_time_ms_(0),
      runaway_detection_initialized_(false),
      last_runaway_encoder_m1_(0),
      last_runaway_encoder_m2_(0),
      last_runaway_check_time_ms_(0),
      motor_runaway_fault_m1_(false),
      motor_runaway_fault_m2_(false) {
  current_pose_ = {0.0f, 0.0f, 0.0f};
  current_velocity_ = {0.0f, 0.0f};
  reconnect();
}

void RoboClawModule::doMixedSpeedAccelDist(
    uint32_t accel_quad_pulses_per_second, int32_t m1_quad_pulses_per_second,
    uint32_t m1_max_distance, int32_t m2_quad_pulses_per_second,
    uint32_t m2_max_distance) {
  if ((m1_quad_pulses_per_second == 0) && (m2_quad_pulses_per_second == 0)) {
    // Reset e-stop once motors are stopped.
    motor_runaway_fault_m1_ = false;
    motor_runaway_fault_m2_ = false;
    digitalWrite(E_STOP_PIN, LOW);  // LOW = E-Stop not active.
  }

  roboclaw_.SpeedAccelDistanceM1M2(
      ROBOCLAW_ADDRESS, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m1_max_distance, m2_quad_pulses_per_second, m2_max_distance, 1);
  // char msg[256];
  // snprintf(
  //     msg, sizeof(msg),
  //     "INFO:[TRoboClaw::DoMixedSpeedAccelDist] M1: %ld qpps, M2: %ld qpps,
  //     " "M1 dist: %ld, M2 dist: %ld", m1_quad_pulses_per_second,
  //     m2_quad_pulses_per_second, m1_max_distance, m2_max_distance);
  // SerialManager::singleton().SendRoboClawStatus(msg);
}

uint32_t RoboClawModule::getError() {
  return roboclaw_.ReadError(ROBOCLAW_ADDRESS);
}

float RoboClawModule::getLogicBatteryVoltage() {
  bool valid;
  int16_t voltage;
  voltage = roboclaw_.ReadLogicBatteryVoltage(ROBOCLAW_ADDRESS, &valid);
  if (valid) {
    return voltage / 10.0f;  // Convert to volts
  } else {
    return -1.0f;  // Error reading voltage
  }
}

void RoboClawModule::getMotorCurrents(float& current_m1, float& current_m2) {
  int16_t currentM1;
  int16_t currentM2;
  bool valid = roboclaw_.ReadCurrents(ROBOCLAW_ADDRESS, currentM1, currentM2);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetCurrents] fail");
    current_m1 = current_m2 = NAN;       // Error reading currents
    current_state_ = State::kReconnect;  // Set state to reconnect
  } else {
    current_m1 = currentM1 / 100.0f;  // Convert to amps
    current_m2 = currentM2 / 100.0f;  // Convert to amps
  }
}

uint32_t RoboClawModule::getM1Encoder() {
  bool valid;
  uint8_t status;
  int32_t value = roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, &status, &valid);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetEncoderM1] fail");
    current_state_ = State::kReconnect;  // Set state to reconnect
    return 0;                            // Error reading encoder
  } else {
    return value;
  }
}

uint32_t RoboClawModule::getM2Encoder() {
  bool valid;
  uint8_t status;
  int32_t value = roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS, &status, &valid);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetEncoderM2] fail");
    current_state_ = State::kReconnect;  // Set state to reconnect
    return 0;                            // Error reading encoder
  } else {
    return value;
  }
}

int32_t RoboClawModule::getM1Speed() {
  bool valid;
  uint8_t status;
  uint32_t speed = roboclaw_.ReadSpeedM1(ROBOCLAW_ADDRESS, &status, &valid);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetSpeedM1] fail");
    current_state_ = State::kReconnect;  // Set state to reconnect
    return 0;
  } else {
    return speed;
  }
}

int32_t RoboClawModule::getM2Speed() {
  bool valid;
  uint8_t status;
  uint32_t speed = roboclaw_.ReadSpeedM2(ROBOCLAW_ADDRESS, &status, &valid);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetSpeedM2] fail");
    current_state_ = State::kReconnect;  // Set state to reconnect
    return 0;
  } else {
    return speed;
  }
}

float RoboClawModule::getMainBatteryVoltage() {
  bool valid;
  int16_t voltage;
  voltage = roboclaw_.ReadMainBatteryVoltage(ROBOCLAW_ADDRESS, &valid);
  if (!valid) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR:[TRoboClaw::GetMainBattery] fail");
    current_state_ = State::kReconnect;  // Set state to reconnect
    return NAN;                          // Error reading voltage
  } else {
    return voltage / 10.0f;  // Convert to volts
  }
}

void RoboClawModule::setM1PID(float p, float i, float d, uint32_t qpps) {
  roboclaw_.SetM1VelocityPID(ROBOCLAW_ADDRESS, p, i, d, qpps);
}

void RoboClawModule::setM2PID(float p, float i, float d, uint32_t qpps) {
  roboclaw_.SetM2VelocityPID(ROBOCLAW_ADDRESS, p, i, d, qpps);
}

void RoboClawModule::setup() {
  // Initialize E-Stop pin
  pinMode(E_STOP_PIN, OUTPUT);
  digitalWrite(E_STOP_PIN, LOW);  // LOW = E-Stop not active.
  // // Initialize encoder readings
  // bool valid1, valid2;
  // prev_encoder_m1_ = roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, nullptr,
  // &valid1); prev_encoder_m2_ = roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS,
  // nullptr, &valid2); if (!valid1 || !valid2) {
  //     Serial.println("RoboClaw: Failed to read initial encoder values.");
  //     SerialManager::singleton().SendDiagnosticMessage("RoboClaw: EncRead
  //     FAIL init");
  // }
}

void RoboClawModule::handleTwistMessage(const String& data) {
  // Parse twist data format: "linear_x,angular_z"
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {
    linear_x_ = data.substring(0, commaIndex).toFloat();
    angular_z_ = data.substring(commaIndex + 1).toFloat();
    last_twist_received_time_ms_ = millis();
  }
}

bool RoboClawModule::isVersionOk() {
  static char version[32];
  version[0] = '\0';
  static bool version_matched = false;

  if (!version_matched && roboclaw_.ReadVersion(ROBOCLAW_ADDRESS, version)) {
    char msg[512];
    if (strcmp(version, ROBOCLAW_SOFTWARE_VERSION) != 0) {
      snprintf(msg, sizeof(msg),
               "ERROR [RoboClawModule::GetVersion] version mismatch, found: "
               "'%s', expected: '%s'",
               version, ROBOCLAW_SOFTWARE_VERSION);
      SerialManager::singleton().SendDiagnosticMessage(msg);
      return false;
    } else {
      snprintf(msg, sizeof(msg),
               "INFO [RoboClawModule::GetVersion] version match, found: '%s'",
               version);
      SerialManager::singleton().SendDiagnosticMessage(msg);
      version_matched = true;
      return true;
    }
  } else if (!version_matched) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR [RoboClawModule::GetVersion fail");
    return false;
  } else {
    return true;
  }
}

void RoboClawModule::loop() {
  switch (current_state_) {
    case State::kReconnect:
      SerialManager::singleton().SendDiagnosticMessage(
          "INFO [RoboClawModule::Loop] Attempting to reconnect to RoboClaw");
      reconnect();
      break;

    case State::kLoop:
      updateMotorCommands();
      updateOdometry();
      checkForRunaway();  // Check for motor runaway conditions
      publishStatus();    // Publish status periodically
      break;

    default:
      // Handle unexpected states if necessary
      break;
  }
}

void RoboClawModule::publishStatus() {
  // Takes about 24 to 25 ms to execute.
  static uint32_t start_loop_ms = millis();
  uint32_t now_ms = millis();
  if ((now_ms - start_loop_ms) >= ROBOCLAW_PUBLISH_STATUS_INTERVAL_MS) {
    if (current_state_ !=
        kReconnect) {  // Don't publish status if in version check state
      const size_t MAXSIZE = 256;
      char msg[MAXSIZE];
      uint32_t error = getError();
      float current_m1 = 0.0f;
      float current_m2;
      getMotorCurrents(current_m1, current_m2);
      snprintf(msg, MAXSIZE,
               "{\"LogicVoltage\":%-2.1f,\"MainVoltage\":%-2.1f,\"Encoder_"
               "Left\":%-ld,\"Encoder_Right\":"
               "%-ld,\"LeftMotorCurrent\":%-2.3f,\"RightMotorCurrent\":%-2.3f,"
               "\"LeftMotorSpeed\":%ld,\"RightMotorSpeed\":%ld,"
               "\"Error\":%-lX}",
               getLogicBatteryVoltage(), getMainBatteryVoltage(),
               getM1Encoder(), getM2Encoder(), current_m1, current_m2,
               getM1Speed(), getM2Speed(), error);
      SerialManager::singleton().SendRoboClawStatus(msg);
    }

    start_loop_ms = now_ms;
  }
}

void RoboClawModule::reconnect() {
  current_state_ = State::kReconnect;  // Set state to reconnect

  // Attempt to reinitialize the RoboClaw connection
  roboclaw_.~RoboClaw();  // Clean up the previous instance
  roboclaw_ = RoboClaw(&ROBOCLAW_SERIAL, ROBOCLAW_TIMEOUT_US);
  roboclaw_.begin(ROBOCLAW_BAUD_RATE);
  if (isVersionOk()) {
    SerialManager::singleton().SendDiagnosticMessage(
        "INFO [RoboClawModule::Loop] Version check passed");
    current_state_ = State::kLoop;  // Transition to the loop state
    setM1PID(7.26239, 2.43, 00, 2437);
    setM2PID(7.26239, 2.43, 00, 2437);
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR [RoboClawModule::Loop] Version check failed, staying in "
        "version state");
  }
}

//     // If E-Stop is active, do not process new motor commands other than
//     stop.
// // Assuming global SerialManager instance for diagnostics
// extern SerialManager serial_manager;

//     // Initialize encoder readings
//     bool valid1, valid2;
//     prev_encoder_m1_ = roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, nullptr,
//     &valid1); prev_encoder_m2_ = roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS,
//     nullptr, &valid2); if (!valid1 || !valid2) {
//         // Serial.println("RoboClaw: Failed to read initial encoder
//         values."); serial_manager.SendDiagnosticMessage("RoboClaw: EncRead
//         FAIL init");
//     }
//     last_odom_update_time_us_ = micros();
//     // Serial.println("RoboClawModule: Setup complete.");
// }

// void RoboClawModule::Loop() {

//     // For now, prioritize odometry and safety checks on each relevant loop
//     cycle.
//     // Odometry should be updated frequently.
//     updateOdometry();

//     // Safety checks can be done slightly less frequently if needed, but
//     critical.
//     // Example: Check safety every few loops if full check is too long.
//     // For now, call it, assuming it's quick enough or internally manages
//     its time. if (millis() - last_status_check_time_ms_ > 50) { // Check
//     status/safety every 50ms
//         checkMotorSafety();
//         last_status_check_time_ms_ = millis();
//     }

//     // If E-Stop is active, do not process new motor commands other than
//     stop.
//     // Motor commands are applied via SetTargetSpeed.

//     EndLoopTiming();
// }

void RoboClawModule::updateMotorCommands() {
  // This code typically takes about 26 ms to execute if it isn't
  // bypassed  by the rate limiting logic below.
  if ((millis() - last_twist_received_time_ms_) >
      MAX_MS_TO_WAIT_FOR_CMD_VEL_BEFORE_STOP_MOTORS) {
    // If no twist message received for 200 ms, stop the motors.
    linear_x_ = 0.0f;
    angular_z_ = 0.0f;
  }

  // Rate limiting code to prevent pounding on the RoboClaw controller.
  static uint32_t last_commanded_time_ms = millis();
  uint32_t current_time_ms = millis();

  uint32_t duration_ms = current_time_ms - last_commanded_time_ms;
  if (duration_ms < (MAX_SECONDS_COMMANDED_TRAVEL * 0.25f * 1000.0f)) {
    // Don't pound the controller at max loop rate.
    // Instead, process commands about four times as fast as keyboard_teleop
    // would normally send them, or about 80 times per second.
    return;
  }

  last_commanded_time_ms = current_time_ms;

  float v_left_mps = linear_x_ - (angular_z_ * WHEEL_BASE_M / 2.0f);
  float v_right_mps = linear_x_ + (angular_z_ * WHEEL_BASE_M / 2.0f);

  // Convert m/s to RPM
  // RPM = (m/s * 60) / (PI * WHEEL_DIAMETER_M)
  float rpm_left = (v_left_mps * 60.0f) / (M_PI * WHEEL_DIAMETER_M);
  float rpm_right = (v_right_mps * 60.0f) / (M_PI * WHEEL_DIAMETER_M);

  // Convert RPM to QPPS (Quadrature Pulses Per Second for RoboClaw)
  // QPPS = (RPM / 60) * QUADRATURE_PULSES_PER_REVOLUTION
  int32_t qpps_m1 = static_cast<int32_t>(
      (rpm_left / 60.0f) *
      QUADRATURE_PULSES_PER_REVOLUTION);  // M1 is left motor
  int32_t qpps_m2 = static_cast<int32_t>(
      (rpm_right / 60.0f) *
      QUADRATURE_PULSES_PER_REVOLUTION);  // M2 is right motor

  // Constrain QPPS to max safe speed
  qpps_m1 =
      constrain_value(qpps_m1, -static_cast<int32_t>(MAX_MOTOR_SPEED_QPPS),
                      static_cast<int32_t>(MAX_MOTOR_SPEED_QPPS));
  qpps_m2 =
      constrain_value(qpps_m2, -static_cast<int32_t>(MAX_MOTOR_SPEED_QPPS),
                      static_cast<int32_t>(MAX_MOTOR_SPEED_QPPS));

  const int32_t qpps_m1_max_distance =
      fabs(qpps_m1 * MAX_SECONDS_COMMANDED_TRAVEL);
  const int32_t qpps_m2_max_distance =
      fabs(qpps_m2 * MAX_SECONDS_COMMANDED_TRAVEL);
  // {
  //   char msg[128];
  //   snprintf(msg, sizeof(msg),
  //            "RoboClawModule: rpm_left=%4.3f, qpps_m1=%d, qpps_m2=%d, "
  //            "qpps_m1_max_distance=%d, qpps_m2_max_distance=%d",
  //            rpm_left, qpps_m1, qpps_m2, qpps_m1_max_distance,
  //            qpps_m2_max_distance);
  //   SerialManager::singleton().SendDiagnosticMessage(msg);
  // }

  // Capture commanded QPPS values and time for runaway detection
  last_commanded_qpps_m1_ = qpps_m1;
  last_commanded_qpps_m2_ = qpps_m2;
  last_command_time_ms_ = millis();

  // Reset runaway faults if commanded speed is zero
  if (qpps_m1 == 0) {
    motor_runaway_fault_m1_ = false;
  }
  if (qpps_m2 == 0) {
    motor_runaway_fault_m2_ = false;
  }

  doMixedSpeedAccelDist(MAX_ACCELERATION_QPPS2, qpps_m1, qpps_m1_max_distance,
                        qpps_m2, qpps_m2_max_distance);
}

void RoboClawModule::updateOdometry() {
  static uint32_t last_commanded_time_ms = millis();
  uint32_t current_time_ms = millis();

  // Initialize on first call
  uint32_t duration_ms = current_time_ms - last_commanded_time_ms;
  if (duration_ms < ODOMETRY_UPDATE_PERIOD_MS) {
    // Don't pound the controller at max loop rate.
    return;
  }

  last_commanded_time_ms = current_time_ms;

  static uint32_t last_odom_update_time_us_ = micros();
  static long prev_encoder_m1_ = 0;
  static long prev_encoder_m2_ = 0;
  static bool odom_initialized_ = false;
  if (!odom_initialized_) {
    // Initialize odometry on first call
    bool valid_m1, valid_m2;
    uint8_t status_m1, status_m2;
    prev_encoder_m1_ =
        roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, &status_m1, &valid_m1);
    prev_encoder_m2_ =
        roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS, &status_m2, &valid_m2);
    last_odom_update_time_us_ = micros();
    odom_initialized_ = true;
  }

  unsigned long current_time_us = micros();
  float dt_s = (current_time_us - last_odom_update_time_us_) / 1000000.0f;
  if (dt_s <= 0.0f) return;  // Avoid division by zero or negative dt

  bool valid_m1, valid_m2;
  uint8_t status_m1, status_m2;
  long current_encoder_m1 =
      roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, &status_m1, &valid_m1);
  long current_encoder_m2 =
      roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS, &status_m2, &valid_m2);

  if (!valid_m1 || !valid_m2) {
    // Failed to read encoders, skip odometry update this cycle
    // Consider incrementing an error counter or logging
    SerialManager::singleton().SendDiagnosticMessage(
        "RoboClaw: EncRead FAIL odom");
    return;
  }

  long delta_encoder_m1 = current_encoder_m1 - prev_encoder_m1_;
  long delta_encoder_m2 = current_encoder_m2 - prev_encoder_m2_;

  prev_encoder_m1_ = current_encoder_m1;
  prev_encoder_m2_ = current_encoder_m2;

  float dist_m1 = ticksToMeters(delta_encoder_m1);  // Left wheel distance
  float dist_m2 = ticksToMeters(delta_encoder_m2);  // Right wheel distance

  float delta_distance = (dist_m1 + dist_m2) / 2.0f;
  float delta_theta = (dist_m2 - dist_m1) / WHEEL_BASE_M;

  current_pose_.x +=
      delta_distance * cos(current_pose_.theta + delta_theta / 2.0f);
  current_pose_.y +=
      delta_distance * sin(current_pose_.theta + delta_theta / 2.0f);
  current_pose_.theta = normalize_angle(current_pose_.theta + delta_theta);

  current_velocity_.linear_x = delta_distance / dt_s;
  current_velocity_.angular_z = delta_theta / dt_s;

  last_odom_update_time_us_ = current_time_us;

  float q[4];
  EulerToQuaternion(0, 0, current_pose_.theta, q);
  char msg[256];
  snprintf(msg, sizeof(msg),
           "px=%.2f,py=%.2f,ox=%.2f,oy=%.2f,oz=%.2f,ow=%.2f,"
           "vx=%.2f,vy=%.2f,wz=%.2f",
           current_pose_.x, current_pose_.y, q[1], q[2], q[3], q[0],
           current_velocity_.linear_x,
           0.0f,  // Assuming no linear_y for 2D pose
           current_velocity_.angular_z);
  SerialManager::singleton().SendOdometry(msg);
}

float RoboClawModule::ticksToMeters(long quadrature_pulses) {
  return (static_cast<float>(quadrature_pulses) /
          QUADRATURE_PULSES_PER_REVOLUTION) *
         M_PI * WHEEL_DIAMETER_M;
}

// -- When velocity set to zero, begin reset by entereing loop
// -- state where we wait for the various conditions to register
// -- such as temperature, current and speed. Then release e-stop.

// void RoboClawModule::checkMotorSafety() {
//     if (e_stop_active_) return; // Don't check if already e-stopped

//     // --- Over Current Check ---
//     float current_m1 = GetMotor1Current(); // These are placeholders, need
//     actual implementation float current_m2 = GetMotor2Current();

//     if (current_m1 > ROBOCLAW_M1_MAX_CURRENT_A || current_m2 >
//     ROBOCLAW_M2_MAX_CURRENT_A) {
//         motor_overcurrent_detected_ = true;
//         triggerEStop("Motor Overcurrent");
//         return;
//     } else {
//         motor_overcurrent_detected_ = false;
//     }

//     // --- Runaway/Stall Detection (based on current_velocity_ from
//     odometry and last_commanded_qpps) ---
//     // This is a simplified check. More sophisticated checks might involve
//     PID error, etc.

//     // If there was a non-zero command recently
//     if (millis() - last_nonzero_command_time_ms_ < 1000) { // Check within
//     1 second of last command
//         float expected_speed_m1_rps =
//         (static_cast<float>(last_commanded_qpps_m1_) /
//         TICKS_PER_REVOLUTION)
//         * 2.0f * M_PI; // rad/s wheel float expected_speed_m2_rps =
//         (static_cast<float>(last_commanded_qpps_m2_) /
//         TICKS_PER_REVOLUTION)
//         * 2.0f * M_PI;

//         // Actual wheel speeds from odometry (dist_m1/dt_s is linear speed
//         of wheel contact point)
//         // Wheel angular speed = linear_wheel_speed / WHEEL_RADIUS_M
//         float actual_speed_m1_rps = (current_velocity_.linear_x -
//         (current_velocity_.angular_z * WHEEL_BASE_M / 2.0f)) /
//         WHEEL_RADIUS_M; float actual_speed_m2_rps =
//         (current_velocity_.linear_x + (current_velocity_.angular_z *
//         WHEEL_BASE_M / 2.0f)) / WHEEL_RADIUS_M;

//         // Stall detection
//         bool m1_should_move = abs(last_commanded_qpps_m1_) >
//         MOTOR_STALL_ENCODER_THRESHOLD * 5; // A reasonable threshold for
//         "commanded to move" bool m2_should_move =
//         abs(last_commanded_qpps_m2_) > MOTOR_STALL_ENCODER_THRESHOLD * 5;

//         if (m1_should_move && abs(actual_speed_m1_rps) < 0.1f) { //
//         Arbitrary small speed threshold for stall
//             if (motor_stall_start_time_m1_ms_ == 0)
//             motor_stall_start_time_m1_ms_ = millis(); if (millis() -
//             motor_stall_start_time_m1_ms_ > MOTOR_STALL_DETECTION_MS) {
//                 motor_stall_detected_m1_ = true;
//                 triggerEStop("Motor M1 Stall");
//                 return;
//             }
//         } else {
//             motor_stall_start_time_m1_ms_ = 0;
//             motor_stall_detected_m1_ = false;
//         }

//         if (m2_should_move && abs(actual_speed_m2_rps) < 0.1f) {
//              if (motor_stall_start_time_m2_ms_ == 0)
//              motor_stall_start_time_m2_ms_ = millis();
//             if (millis() - motor_stall_start_time_m2_ms_ >
//             MOTOR_STALL_DETECTION_MS) {
//                 motor_stall_detected_m2_ = true;
//                 triggerEStop("Motor M2 Stall");
//                 return;
//             }
//         } else {
//             motor_stall_start_time_m2_ms_ = 0;
//             motor_stall_detected_m2_ = false;
//         }

//         // Runaway detection (moving significantly faster than commanded)
//         if ( (abs(expected_speed_m1_rps) > 0.1f && abs(actual_speed_m1_rps)
//         > abs(expected_speed_m1_rps) * MOTOR_RUNAWAY_SPEED_FACTOR) ||
//              (abs(expected_speed_m2_rps) > 0.1f && abs(actual_speed_m2_rps)
//              > abs(expected_speed_m2_rps) * MOTOR_RUNAWAY_SPEED_FACTOR) ) {
//             motor_runaway_detected_ = true;
//             triggerEStop("Motor Runaway");
//             return;
//         } else {
//             motor_runaway_detected_ = false;
//         }
//     } else {
//         // No recent non-zero command, reset stall/runaway timers/flags
//         motor_stall_start_time_m1_ms_ = 0;
//         motor_stall_start_time_m2_ms_ = 0;
//         motor_stall_detected_m1_ = false;
//         motor_stall_detected_m2_ = false;
//         motor_runaway_detected_ = false;
//     }
// }

bool RoboClawModule::IsUnsafe() {
  //     // This module's specific unsafe conditions, E-Stop activation is the
  //     master unsafe flag return e_stop_active_ || motor_runaway_detected_
  //     || motor_overcurrent_detected_ || motor_stall_detected_m1_ ||
  //     motor_stall_detected_m2_;
  return false;  // Placeholder, implement actual checks
}

// void RoboClawModule::triggerEStop(const char* reason) {
//     if (!e_stop_active_) {
//         e_stop_active_ = true;
//         roboclaw_.SpeedM1M2(ROBOCLAW_ADDRESS, 0, 0); // Command motors to
//         stop immediately digitalWrite(E_STOP_PIN, HIGH); // Activate E-Stop
//         (HIGH = power OFF to motors, or trigger relay)
//         // Serial.print("E-Stop Triggered: "); Serial.println(reason);
//         serial_manager.SendDiagnosticMessage(String("E-Stop: ") + reason);
//     }
// }

// void RoboClawModule::resetEStop() {
//     if (e_stop_active_) {
//         // Only reset if underlying safety conditions are cleared
//         // For now, assume a zero command is enough to attempt reset.
//         // More sophisticated logic might check if the *cause* of E-Stop is
//         resolved. e_stop_active_ = false; motor_runaway_detected_ = false;
//         motor_overcurrent_detected_ = false;
//         motor_stall_detected_m1_ = false;
//         motor_stall_detected_m2_ = false;
//         digitalWrite(E_STOP_PIN, LOW); // Deactivate E-Stop (LOW = power
//         ON)
//         // Serial.println("E-Stop Reset.");
//         serial_manager.SendDiagnosticMessage("E-Stop Reset");
//         // Reset RoboClaw errors if any.
//         roboclaw_.ResetEncoders(ROBOCLAW_ADDRESS); // Example: reset
//         encoders might clear some error flags.
//                                                     // Or use specific
//                                                     error reset commands if
//                                                     available.
//     }
// }

void RoboClawModule::ResetSafetyFlags() {
  //     // This is essentially what resetEStop does for this module's flags.
  //     // If E-Stop is not active, these flags should already be false from
  //     checks. if (!e_stop_active_) {
  //         motor_runaway_detected_ = false;
  //         motor_overcurrent_detected_ = false;
  //         motor_stall_detected_m1_ = false;
  //         motor_stall_detected_m2_ = false;
  //     }
}

void RoboClawModule::checkForRunaway() {
  uint32_t current_time_ms = millis();

  // Initialize on first call
  if (!runaway_detection_initialized_) {
    bool valid_m1, valid_m2;
    uint8_t status_m1, status_m2;
    last_runaway_encoder_m1_ =
        roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, &status_m1, &valid_m1);
    last_runaway_encoder_m2_ =
        roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS, &status_m2, &valid_m2);

    if (!valid_m1 || !valid_m2) {
      SerialManager::singleton().SendDiagnosticMessage(
          "ERROR: [RoboClawModule::checkForRunaway] Failed to initialize "
          "encoder readings");
      return;
    }

    last_runaway_check_time_ms_ = current_time_ms;
    runaway_detection_initialized_ = true;
    return;
  }

  // Check if enough time has passed since last check
  if ((current_time_ms - last_runaway_check_time_ms_) < TEST_MOTOR_RUNAWAY_MS) {
    return;
  }

  // Read current encoder values
  bool valid_m1, valid_m2;
  uint8_t status_m1, status_m2;
  long current_encoder_m1 =
      roboclaw_.ReadEncM1(ROBOCLAW_ADDRESS, &status_m1, &valid_m1);
  long current_encoder_m2 =
      roboclaw_.ReadEncM2(ROBOCLAW_ADDRESS, &status_m2, &valid_m2);

  if (!valid_m1 || !valid_m2) {
    SerialManager::singleton().SendDiagnosticMessage(
        "ERROR: [RoboClawModule::checkForRunaway] Failed to read current "
        "encoder values");
    return;
  }

  // Calculate time difference in seconds
  float dt_s = (current_time_ms - last_runaway_check_time_ms_) / 1000.0f;
  if (dt_s <= 0.0f) return;  // Avoid division by zero

  // Calculate actual encoder speeds (QPPS - Quadrature Pulses Per Second)
  long delta_encoder_m1 = current_encoder_m1 - last_runaway_encoder_m1_;
  long delta_encoder_m2 = current_encoder_m2 - last_runaway_encoder_m2_;

  float actual_qpps_m1 = delta_encoder_m1 / dt_s;
  float actual_qpps_m2 = delta_encoder_m2 / dt_s;

  // Check for runaway conditions
  // Only check if we have a non-zero commanded speed
  if (last_commanded_qpps_m1_ != 0) {
    float expected_qpps_m1 = static_cast<float>(last_commanded_qpps_m1_);
    float threshold_qpps_m1 =
        fabs(expected_qpps_m1) * (CRITICAL_MOTOR_RUNAWAY_PERCENT / 100.0f);

    if (fabs(actual_qpps_m1) > threshold_qpps_m1) {
      if (!motor_runaway_fault_m1_) {
        motor_runaway_fault_m1_ = true;
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "FAULT: Motor M1 runaway detected! Actual: %.1f QPPS, "
                 "Expected: %.1f QPPS, Threshold: %.1f QPPS",
                 actual_qpps_m1, expected_qpps_m1, threshold_qpps_m1);
        SerialManager::singleton().SendDiagnosticMessage(msg);
      }
    }
  }

  if (last_commanded_qpps_m2_ != 0) {
    float expected_qpps_m2 = static_cast<float>(last_commanded_qpps_m2_);
    float threshold_qpps_m2 =
        fabs(expected_qpps_m2) * (CRITICAL_MOTOR_RUNAWAY_PERCENT / 100.0f);

    if (fabs(actual_qpps_m2) > threshold_qpps_m2) {
      if (!motor_runaway_fault_m2_) {
        motor_runaway_fault_m2_ = true;
        char msg[128];
        snprintf(msg, sizeof(msg),
                 "FAULT: Motor M2 runaway detected! Actual: %.1f QPPS, "
                 "Expected: %.1f QPPS, Threshold: %.1f QPPS",
                 actual_qpps_m2, expected_qpps_m2, threshold_qpps_m2);
        SerialManager::singleton().SendDiagnosticMessage(msg);
      }
    }
  }

  // Update state for next check
  last_runaway_encoder_m1_ = current_encoder_m1;
  last_runaway_encoder_m2_ = current_encoder_m2;
  last_runaway_check_time_ms_ = current_time_ms;
}

// bool RoboClawModule::IsEStopActive() const {
//     return e_stop_active_;
// }

RoboClawModule& RoboClawModule::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new RoboClawModule();  // Assuming ROBOCLAW_SERIAL is defined
                                          // as a HardwareSerial reference
  }
  return *g_singleton_;
}

RoboClawModule* RoboClawModule::g_singleton_ = nullptr;