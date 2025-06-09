#include "troboclaw.h"

#include <string.h>

#include <limits>

#include "Arduino.h"
#include "RoboClaw.h"
#include "config.h"
#include "diagnostic_message.h"
#include "kinematics.h"
#include "trelay.h"

#define HWSERIAL Serial6

void TRoboClaw::CheckForMotorStall() {
  static const float kStallCurrentThreshold = 10.0;  // Stall current (AMPS).
  static const uint32_t kMaxAllowedConsecutiveStallFaults = 5;

  static uint32_t consecutiveStallFaultsLeftMotor = 0;
  static uint32_t consecutiveStallFaultsRightMotor = 0;

  float motor_current = 0.0;
  char msg[512];

  if (abs(motor_current = GetM1Current()) > kStallCurrentThreshold) {
    consecutiveStallFaultsLeftMotor += 1;
    snprintf(msg, sizeof(msg),
             "WARN:[TRoboclaw::CheckForMotorStall] M1 overcurrent: %f, "
             "consecutive faults: %ld",
             motor_current, consecutiveStallFaultsLeftMotor);
    DiagnosticMessage::singleton().sendMessage(msg);
  } else {
    consecutiveStallFaultsLeftMotor = 0;
  }

  if (abs(motor_current = GetM2Current()) > kStallCurrentThreshold) {
    consecutiveStallFaultsRightMotor += 1;
    snprintf(msg, sizeof(msg),
             "WARN:[TRoboclaw::CheckForMotorStall] M2 overcurrent: %f, "
             "consecutive faults: %ld",
             motor_current, consecutiveStallFaultsRightMotor);
    DiagnosticMessage::singleton().sendMessage(msg);
  } else {
    consecutiveStallFaultsRightMotor = 0;
  }

  bool left_motor_stalled =
      consecutiveStallFaultsLeftMotor > kMaxAllowedConsecutiveStallFaults;
  bool right_motor_stalled =
      consecutiveStallFaultsRightMotor > kMaxAllowedConsecutiveStallFaults;
  if (left_motor_stalled || right_motor_stalled) {
    if (left_motor_stalled) {
      DiagnosticMessage::singleton().sendMessage(
          "ERROR:[TRoboClaw::CheckForMotorStall] STALL for M1 (left) motor");
    } else {
      DiagnosticMessage::singleton().sendMessage(
          "ERROR:[TRoboClaw::CheckForMotorStall] STALL for M2 (right) motor");
    }

    TRelay::singleton().PowerOn(TRelay::kMotorEStop);  // E-stop the motors.
  }
}

void TRoboClaw::DoMixedSpeedDist(int32_t m1_quad_pulses_per_second,
                                 int32_t m1_max_distance,
                                 int32_t m2_quad_pulses_per_second,
                                 int32_t m2_max_distance) {
  if (TRelay::singleton().IsPoweredOn(TRelay::kMotorEStop) &&
      (m1_quad_pulses_per_second == 0) && (m2_quad_pulses_per_second == 0)) {
    TRelay::singleton().PowerOff(TRelay::kMotorEStop);  // UN E-stop the motors.
    DiagnosticMessage::singleton().sendMessage(
        "INFO:[TRoboClaw::DoMixedSpeedDist] Removing E-Stop because of zero "
        "velocity command");
  }

  g_roboclaw_.SpeedDistanceM1M2(kDeviceAddress, m1_quad_pulses_per_second,
                                m1_max_distance, m2_quad_pulses_per_second,
                                m2_max_distance, 1);
}

void TRoboClaw::DoMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                      int32_t m1_quad_pulses_per_second,
                                      uint32_t m1_max_distance,
                                      int32_t m2_quad_pulses_per_second,
                                      uint32_t m2_max_distance) {
  if (TRelay::singleton().IsPoweredOn(TRelay::kMotorEStop) &&
      (m1_quad_pulses_per_second == 0) && (m2_quad_pulses_per_second == 0)) {
    TRelay::singleton().PowerOff(TRelay::kMotorEStop);  // UN E-stop the motors.
    DiagnosticMessage::singleton().sendMessage(
        "INFO:[TRoboClaw::DoMixedSpeedAccelDist] Removing E-Stop because of "
        "zero velocity command");
  }

  // char msg[512];
  // snprintf(msg, sizeof(msg),
  //          "INFO:[TRoboClaw::DoMixedSpeedAccelDist] accel_qpps: %ld,
  //          m1_qpps:"
  //          "%ld, m1_max_dist: %ld, m2_qpps: %ld, "
  //          "m2_max_dist: %ld",
  //          accel_quad_pulses_per_second, m1_quad_pulses_per_second,
  //          m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  // DiagnosticMessage::singleton().sendMessage(msg);
  g_roboclaw_.SpeedAccelDistanceM1M2(
      kDeviceAddress, accel_quad_pulses_per_second, m1_quad_pulses_per_second,
      m1_max_distance, m2_quad_pulses_per_second, m2_max_distance, 1);
}

float TRoboClaw::GetBatteryLogic() { return g_logic_battery_ / 10.0; }

float TRoboClaw::GetBatteryMain() { return g_main_battery_ / 10.0; }

bool TRoboClaw::GetCurrents() {
  int16_t currentM1;
  int16_t currentM2;
  bool valid = g_roboclaw_.ReadCurrents(kDeviceAddress, currentM1, currentM2);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetCurrents] fail");
    return false;
  } else {
    g_current_m1_10ma_ = currentM1;
    g_current_m2_10ma_ = currentM2;
    return true;
  }
}

bool TRoboClaw::GetEncoderM1() {
  bool valid;
  uint8_t status;
  int32_t value = g_roboclaw_.ReadEncM1(kDeviceAddress, &status, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetEncoderM1] fail");
    return false;
  } else {
    g_encoder_m1_ = value;
    return true;
  }
}

bool TRoboClaw::GetEncoderM2() {
  bool valid;
  uint8_t status;
  int32_t value = g_roboclaw_.ReadEncM2(kDeviceAddress, &status, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetEncoderM2] fail");
    return false;
  } else {
    g_encoder_m2_ = value;
    return true;
  }
}

uint32_t TRoboClaw::getError() { return g_roboclaw_.ReadError(kDeviceAddress); }

bool TRoboClaw::GetLogicBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw_.ReadLogicBatteryVoltage(kDeviceAddress, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetLogicBattery] fail");
    return false;
  } else {
    g_logic_battery_ = voltage;
    return true;
  }
}

float TRoboClaw::GetM1Current() { return g_current_m1_10ma_ / 100.0; }

int32_t TRoboClaw::GetM1Encoder() { return g_encoder_m1_; }

int32_t TRoboClaw::GetM1Speed() { return g_speed_m1_; }

float TRoboClaw::GetM2Current() { return g_current_m2_10ma_ / 100.0; }

int32_t TRoboClaw::GetM2Encoder() { return g_encoder_m2_; }

int32_t TRoboClaw::GetM2Speed() { return g_speed_m2_; }

bool TRoboClaw::GetMainBattery() {
  bool valid;
  int16_t voltage;
  voltage = g_roboclaw_.ReadMainBatteryVoltage(kDeviceAddress, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetMainBattery] fail");
    return false;
  } else {
    g_main_battery_ = voltage;
    return true;
  }
}

bool TRoboClaw::GetSpeedM1() {
  bool valid;
  uint8_t status;
  uint32_t speed = g_roboclaw_.ReadSpeedM1(kDeviceAddress, &status, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetSpeedM1] fail");
    g_speed_m1_ = std::numeric_limits<uint32_t>::min();
    return false;
  } else {
    g_speed_m1_ = speed;
    return true;
  }
}

bool TRoboClaw::GetSpeedM2() {
  bool valid;
  uint8_t status;
  int32_t speed = g_roboclaw_.ReadSpeedM2(kDeviceAddress, &status, &valid);
  if (!valid) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetSpeedM2] fail");
    g_speed_m2_ = std::numeric_limits<uint32_t>::min();
    return false;
  } else {
    g_speed_m2_ = speed;
    return true;
  }
}

bool TRoboClaw::GetVersion() {
  static char version[32];
  version[0] = '\0';
  static bool version_matched = false;

  if (!version_matched && g_roboclaw_.ReadVersion(kDeviceAddress, version)) {
    char msg[512];
    if (strcmp(version, kDeviceVersion) != 0) {
      snprintf(msg, sizeof(msg),
               "ERROR:[TRoboClaw::GetVersion] version mismatch, found: '%s'",
               version);
      DiagnosticMessage::singleton().sendMessage(msg);
      return false;
    } else {
      snprintf(msg, sizeof(msg),
               "INFO:[TRoboClaw::GetVersion] version match, found: '%s'",
               version);
      DiagnosticMessage::singleton().sendMessage(msg);
      version_matched = true;
      return true;
    }
  } else if (!version_matched) {
    DiagnosticMessage::singleton().sendMessage(
        "ERROR:[TRoboClaw::GetVersion fail");
    return false;
  } else {
    return true;
  }
}

void TRoboClaw::ResetEncoders() {
  g_roboclaw_.SetEncM1(kDeviceAddress, 0);
  g_roboclaw_.SetEncM2(kDeviceAddress, 0);
}

void TRoboClaw::SetM1PID(float p, float i, float d, uint32_t qpps) {
  g_roboclaw_.SetM1VelocityPID(kDeviceAddress, p, i, d, qpps);
}

void TRoboClaw::SetM2PID(float p, float i, float d, uint32_t qpps) {
  g_roboclaw_.SetM2VelocityPID(kDeviceAddress, p, i, d, qpps);
}

void TRoboClaw::CheckForRunaway(TRoboClaw::WhichMotor whichMotor) {
  static const float kEncoderCountFaultThresholdPerSecond = 1900.0;
  static const uint32_t kMaxAllowedConsecutiveEncoderFaults = uint32_t(
      1566.0 * 0.1);  // Pulses/meter * <max-allowed-fault-distance-in-meters>.
  static uint32_t last_checked_m1_encoder_time_ms = millis();
  static uint32_t last_checked_m2_encoder_time_ms = millis();

  static int32_t last_checked_m1_encoder_value = g_encoder_m1_;
  static int32_t last_checked_m2_encoder_value = g_encoder_m2_;

  static uint32_t accumulated_m1_encoder_diffs = 0;
  static uint32_t accumulated_m2_encoder_diffs = 0;

  static bool is_setup = false;

  float duration_since_last_runaway_check_for_encoder = 0;
  float encoder_diff_per_second = 0;
  bool runaway_fault = false;

  const char* motor_name;

  if (!is_setup) {
    last_checked_m1_encoder_value = g_encoder_m1_;
    last_checked_m2_encoder_value = g_encoder_m2_;
    is_setup = true;
  }

  uint32_t now_ms = millis();
  switch (whichMotor) {
    case kLeftMotor:
      duration_since_last_runaway_check_for_encoder =
          ((now_ms * 1.0) - last_checked_m1_encoder_time_ms) / 1000.0;
      encoder_diff_per_second =
          abs(g_encoder_m1_ - last_checked_m1_encoder_value) /
          duration_since_last_runaway_check_for_encoder;
      motor_name = "M1";
      break;
    case kRightMotor:
      duration_since_last_runaway_check_for_encoder =
          ((now_ms * 1.0) - last_checked_m2_encoder_time_ms) / 1000.0;
      encoder_diff_per_second =
          abs(g_encoder_m2_ - last_checked_m2_encoder_value) /
          duration_since_last_runaway_check_for_encoder;
      motor_name = "M2";
      break;
  }

  if (encoder_diff_per_second > kEncoderCountFaultThresholdPerSecond) {
    // The encodeers are spinning too fast.
    // A fault is triggered only if the robot travels at least a certain
    // distance at this high speed.
    if (whichMotor == kLeftMotor) {
      accumulated_m1_encoder_diffs +=
          abs(g_encoder_m1_ - last_checked_m1_encoder_value);
      runaway_fault =
          accumulated_m1_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
    } else {
      accumulated_m2_encoder_diffs +=
          abs(g_encoder_m2_ - last_checked_m2_encoder_value);
      runaway_fault =
          accumulated_m2_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
    }

    if (runaway_fault) {
      TRelay::singleton().PowerOn(TRelay::kMotorEStop);  // E-stop the motors.
      char msg[512];
      snprintf(msg, sizeof(msg),
               "ERROR:[TRoboClaw::CheckForRunaway] RUNAWAY for motor: %s, "
               "duration_since_last_runaway_check_for_encoder: %-2.3f",
               motor_name, duration_since_last_runaway_check_for_encoder);
      DiagnosticMessage::singleton().sendMessage(msg);
      if (whichMotor == kLeftMotor) {
        accumulated_m1_encoder_diffs = 0;
      } else {
        accumulated_m2_encoder_diffs = 0;
      }
    }
  } else {
    // The motors are not spinning too fast now.
    if (whichMotor == kLeftMotor) {
      accumulated_m1_encoder_diffs = 0;
    } else {
      accumulated_m2_encoder_diffs = 0;
    }
    // TRelay::singleton().PowerOff(TRelay::kMotorEStop);  // ###
  }

  if (whichMotor == kLeftMotor) {
    last_checked_m1_encoder_value = g_encoder_m1_;
    last_checked_m1_encoder_time_ms = now_ms;
  } else {
    last_checked_m2_encoder_value = g_encoder_m2_;
    last_checked_m2_encoder_time_ms = now_ms;
  }
}

void TRoboClaw::SetMotorVelocityAndUpdateOdometry() {
  static uint32_t last_odom_pub_usec = micros();
  unsigned long current_time_usec = micros();
  unsigned long delta_time_usec = current_time_usec - last_odom_pub_usec;

  static Kinematics kinematics(
      Kinematics::DIFFERENTIAL_DRIVE, SuperDroid_1831.max_rpm, 1.0,
      SuperDroid_1831.motor_operating_voltage,
      SuperDroid_1831.motor_power_max_voltage, Sigyn_specs.wheel_diameter,
      Sigyn_specs.wheels_separation);

  double delta_time_minutes = ((double)delta_time_usec) / 60'000'000;

  if ((millis() - last_twist_received_time_ms_) > 200) {
    // If no twist message received for 200 ms, stop the motors.
    linear_x_ = 0.0f;
    angular_z_ = 0.0f;
  }

  // {
  //   if (linear_x_ != 0.0f || angular_z_ != 0.0f) {
  //     char msg[512];
  //     snprintf(msg, sizeof(msg),
  //              "INFO:[TRoboClaw::SetMotorVelocityAndUpdateOdometry] "
  //              "at: %lu, ms since received twist: %lu, linear_x: %.2f, "
  //              "angular_z: %.2f",
  //              millis(), millis() - last_twist_received_time_ms_, linear_x_,
  //              angular_z_);
  //     DiagnosticMessage::singleton().sendMessage(msg);
  //   }
  // }

  // Update motor commands based on twist message
  UpdateMotorCommands(kinematics, delta_time_minutes);

  // Calculate odometry from encoder readings
  CalculateOdometry(kinematics, delta_time_usec);

  last_odom_pub_usec = current_time_usec;
}

void TRoboClaw::UpdateMotorCommands(const Kinematics& kinematics,
                                    float delta_time_minutes) {
  static const float kMaxSecondsUncommandedTravel = 0.25;
  static const int32_t kAccelQuadPulsesPerSecond = 1000;

  // Create a non-const reference to call the method
  Kinematics& non_const_kinematics = const_cast<Kinematics&>(kinematics);
  Kinematics::rpm rpm =
      non_const_kinematics.getRPM(linear_x_, 0.0f, angular_z_);

  const int32_t m1_quad_pulses_per_second =
      rpm.motor1 * SuperDroid_1831.quad_pulses_per_revolution / 60.0;
  const int32_t m2_quad_pulses_per_second =
      rpm.motor2 * SuperDroid_1831.quad_pulses_per_revolution / 60.0;
  const int32_t m1_max_distance =
      fabs(m1_quad_pulses_per_second * kMaxSecondsUncommandedTravel);
  const int32_t m2_max_distance =
      fabs(m2_quad_pulses_per_second * kMaxSecondsUncommandedTravel);

  TRoboClaw::singleton().DoMixedSpeedAccelDist(
      kAccelQuadPulsesPerSecond, m1_quad_pulses_per_second, m1_max_distance,
      m2_quad_pulses_per_second, m2_max_distance);
}

void TRoboClaw::CalculateOdometry(const Kinematics& kinematics,
                                  float delta_time_usec) {
  // Initialize odometry state on first run
  if (!odometry_initialized_) {
    prev_m1_ticks_ = TRoboClaw::singleton().GetM1Encoder();
    prev_m2_ticks_ = TRoboClaw::singleton().GetM2Encoder();
    x_pos_ = 0.0f;
    y_pos_ = 0.0f;
    heading_ = 0.0f;
    odometry_initialized_ = true;
    return;
  }

  if (delta_time_usec == 0) {
    // If no time has passed, skip odometry update
    return;
  }

  // Get current encoder readings
  int32_t current_m1_ticks = TRoboClaw::singleton().GetEncoderM1();
  int32_t current_m2_ticks = TRoboClaw::singleton().GetEncoderM2();
  int32_t delta_ticks_m1 = current_m1_ticks - prev_m1_ticks_;
  int32_t delta_ticks_m2 = current_m2_ticks - prev_m2_ticks_;

  // Calculate current RPM for each motor
  double delta_time_minutes = ((double)delta_time_usec) / 60'000'000;
  float current_rpm1 = (((float)delta_ticks_m1 /
                         (float)SuperDroid_1831.quad_pulses_per_revolution) /
                        delta_time_minutes);
  float current_rpm2 = (((float)delta_ticks_m2 /
                         (float)SuperDroid_1831.quad_pulses_per_revolution) /
                        delta_time_minutes);

  // Get current velocities
  Kinematics& non_const_kinematics = const_cast<Kinematics&>(kinematics);
  Kinematics::velocities current_vel =
      non_const_kinematics.getVelocities(current_rpm1, current_rpm2, 0.0, 0.0);

  // Update odometry only if not in version state
  if (g_state_ != kVersion) {
    float vel_dt = delta_time_usec / 1'000'000.0;

    // Calculate deltas
    float delta_heading = current_vel.angular_z * vel_dt;
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x =
        (current_vel.linear_x * cos_h - current_vel.linear_y * sin_h) * vel_dt;
    float delta_y =
        (current_vel.linear_x * sin_h + current_vel.linear_y * cos_h) * vel_dt;

    // Update position and heading
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    // Publish odometry data
    PublishOdometryData(x_pos_, y_pos_, heading_, current_vel);
  }

  // Update previous encoder values
  prev_m1_ticks_ = current_m1_ticks;
  prev_m2_ticks_ = current_m2_ticks;
}

void TRoboClaw::PublishOdometryData(float x_pos, float y_pos, float heading,
                                    const Kinematics::velocities& current_vel) {
  // Calculate quaternion from heading
  float q[4];
  TRoboClaw::singleton().EulerToQuaternion(0, 0, heading, q);

  // Publish odometry message
  Serial.print("ODOM:");
  Serial.print("px=");
  Serial.print(x_pos);
  Serial.print(",py=");
  Serial.print(y_pos);
  Serial.print("ox=");
  Serial.print((double)q[1]);
  Serial.print(",oy=");
  Serial.print((double)q[2]);
  Serial.print(",oz=");
  Serial.print((double)q[3]);
  Serial.print(",ow=");
  Serial.print((double)q[0]);
  Serial.print(",vx=");
  Serial.print(current_vel.linear_x);
  Serial.print(",vy=");
  Serial.print(current_vel.linear_y);
  Serial.print(",wz=");
  Serial.println(current_vel.angular_z);
}

void TRoboClaw::handleTwistMessage(const String& data) {
  // Parse twist data format: "linear_x,angular_z"
  int commaIndex = data.indexOf(',');
  if (commaIndex != -1) {
    linear_x_ = data.substring(0, commaIndex).toFloat();
    angular_z_ = data.substring(commaIndex + 1).toFloat();
    last_twist_received_time_ms_ = millis();
    // {
    //   char msg[128];
    //   snprintf(msg, sizeof(msg),
    //            "INFO:[TRoboClaw::handleTwistMessage] At: %lu Received twist: "
    //            "linear_x=%.2f, angular_z=%.2f",
    //            millis(), linear_x_, angular_z_);
    //   DiagnosticMessage::singleton().sendMessage(msg);
    // }

    // Serial.print("Received twist: linear_x=");
    // Serial.print(linear_x);
    // Serial.print(", angular_z=");
    // Serial.println(angular_z);
  }
}

void TRoboClaw::loop() {
  PublishRoboClawStats();
  {
    static uint32_t last_odom_publish_time_ms = millis();
    uint32_t now_ms = millis();
    if (((now_ms - last_odom_publish_time_ms) >= 40) &&
        (g_state_ != kVersion)) {
      last_odom_publish_time_ms = now_ms;
      SetMotorVelocityAndUpdateOdometry();
    }
  }

  // Notes about GetXxx speeds.
  // GetCurrents: 3.7 ms
  // GetEncoderM1: 3.7 ms
  // Get LogicBattery: 2.5 ms
  // GetMainBattery: 3.1 ms
  switch (g_state_) {
    case kVersion:
      if (GetVersion()) {
        g_state_ = kSpeedM1;
      } else {
        Reconnect();
        g_state_ = kVersion;
      }
      break;

    case kSpeedM1:
      //   if (GetSpeedM1() && GetSpeedM2()) {
      //     g_state_ = kEncoderM1;
      //   } else {
      //     Reconnect();
      //     g_state_ = kVersion;
      //   }
      break;

      // case kEncoderM1:
      //   // if (GetEncoderM1() && GetEncoderM2()) {
      //   g_state_ = kLogicBattery;
      //   CheckForRunaway(kLeftMotor);
      //   CheckForRunaway(kRightMotor);
      //   // } else {
      //   //   Reconnect();
      //   //   g_state_ = kVersion;
      //   // }
      //   break;

      // case kLogicBattery:
      //   if (GetLogicBattery() && GetMainBattery() && GetCurrents()) {
      //     g_state_ = kSpeedM1;
      //   } else {
      //     Reconnect();
      //     g_state_ = kVersion;
      //   }

    default:
      g_state_ = kVersion;
      break;
  }
}

const void TRoboClaw::EulerToQuaternion(float roll, float pitch, float yaw,
                                        float* q) {
  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);

  q[0] = cy * cp * cr + sy * sp * sr;
  q[1] = cy * cp * sr - sy * sp * cr;
  q[2] = sy * cp * sr + cy * sp * cr;
  q[3] = sy * cp * cr - cy * sp * sr;
}

void TRoboClaw::PublishRoboClawStats() {
  static uint32_t start_loop_ms = millis();
  uint32_t now_ms = millis();
  if (now_ms - start_loop_ms >= 1000) {
    if (g_state_ != kVersion) {
      const size_t MAXSIZE = 512;
      char msg[MAXSIZE];
      uint32_t error = TRoboClaw::singleton().getError();
      snprintf(msg, MAXSIZE,
               "{\"LogicVoltage\":%-2.1f,\"MainVoltage\":%-2.1f,\"Encoder_"
               "Left\":%-ld,\"Encoder_Right\":"
               "%-ld,\"LeftMotorCurrent\":%-2.3f,\"RightMotorCurrent\":%-2.3f,"
               "\"LeftMotorSpeed\":%ld,\"RightMotorSpeed\":%ld,"
               "\"Error\":%-lX}",
               TRoboClaw::singleton().GetBatteryLogic(),
               TRoboClaw::singleton().GetBatteryMain(),
               TRoboClaw::singleton().GetM1Encoder(),
               TRoboClaw::singleton().GetM2Encoder(),
               TRoboClaw::singleton().GetM1Current(),
               TRoboClaw::singleton().GetM2Current(),
               TRoboClaw::singleton().GetM1Speed(),
               TRoboClaw::singleton().GetM2Speed(), error);
      Serial.print("ROBOCLAW:");
      Serial.println(msg);
      // #if USE_TSD
      //       TSd::singleton().log(g_singleton_->string_msg_.data.data);
      // #endif
    }

    start_loop_ms = now_ms;
  }
}

void TRoboClaw::Reconnect() {
  g_current_m1_10ma_ = 0;
  g_current_m2_10ma_ = 0;
  g_encoder_m1_ = 0;
  g_encoder_m2_ = 0;
  g_logic_battery_ = 0;
  g_main_battery_ = 0;
  g_roboclaw_.~RoboClaw();
  g_roboclaw_ = RoboClaw(&Serial6, 10'000);
  g_roboclaw_.begin(kBaudRate);
  g_speed_m1_ = 0;
  g_speed_m2_ = 0;
}

void TRoboClaw::setup() {
  Reconnect();
  GetEncoderM1();
  GetEncoderM2();
}

TRoboClaw::TRoboClaw()
    : TModule(TModule::kRoboClaw),
      g_current_m1_10ma_(0),
      g_current_m2_10ma_(0),
      g_encoder_m1_(0),
      g_encoder_m2_(0),
      g_logic_battery_(0),
      g_main_battery_(0),
      g_speed_m1_(0),
      g_speed_m2_(0),
      linear_x_(0.0f),
      angular_z_(0.0f),
      last_twist_received_time_ms_(0),
      x_pos_(0.0f),
      y_pos_(0.0f),
      heading_(0.0f),
      odometry_initialized_(false),
      prev_m1_ticks_(0),
      prev_m2_ticks_(0) {
  Serial6.begin(kBaudRate);
}

TRoboClaw& TRoboClaw::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TRoboClaw();
  }

  return *g_singleton_;
}

const char* TRoboClaw::kDeviceVersion = "USB Roboclaw 2x15a v4.2.8\n";

RoboClaw TRoboClaw::g_roboclaw_(&Serial6, 10'000);

TRoboClaw* TRoboClaw::g_singleton_ = nullptr;

TRoboClaw::State TRoboClaw::g_state_ = TRoboClaw::kVersion;