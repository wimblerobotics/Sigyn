#include "troboclaw.h"

#include <string.h>

#include <limits>

#include "Arduino.h"
#include "RoboClaw.h"
#include "tmicro_ros.h"
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
             "WARN [TRoboclaw::CheckForMotorStall] M1 overcurrent: %f, "
             "consecutive faults: %ld",
             motor_current, consecutiveStallFaultsLeftMotor);
    TMicroRos::singleton().PublishDiagnostic(msg);
  } else {
    consecutiveStallFaultsLeftMotor = 0;
  }

  if (abs(motor_current = GetM2Current()) > kStallCurrentThreshold) {
    consecutiveStallFaultsRightMotor += 1;
    snprintf(msg, sizeof(msg),
             "WARN [TRoboclaw::CheckForMotorStall] M2 overcurrent: %f, "
             "consecutive faults: %ld",
             motor_current, consecutiveStallFaultsRightMotor);
    TMicroRos::singleton().PublishDiagnostic(msg);
  } else {
    consecutiveStallFaultsRightMotor = 0;
  }

  bool left_motor_stalled =
      consecutiveStallFaultsLeftMotor > kMaxAllowedConsecutiveStallFaults;
  bool right_motor_stalled =
      consecutiveStallFaultsRightMotor > kMaxAllowedConsecutiveStallFaults;
  if (left_motor_stalled || right_motor_stalled) {
    if (left_motor_stalled) {
      TMicroRos::singleton().PublishDiagnostic(
          "ERROR [TRoboClaw::CheckForMotorStall] STALL for M1 (left) motor");
    } else {
      TMicroRos::singleton().PublishDiagnostic(
          "ERROR [TRoboClaw::CheckForMotorStall] STALL for M2 (right) motor");
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
    TMicroRos::singleton().PublishDiagnostic(
        "INFO [TRoboClaw::DoMixedSpeedDist] Removing E-Stop because of zero "
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
    TMicroRos::singleton().PublishDiagnostic(
        "INFO [TRoboClaw::DoMixedSpeedAccelDist] Removing E-Stop because of "
        "zero velocity command");
  }

  char msg[512];
  snprintf(msg, sizeof(msg),
           "INFO [TRoboClaw::DoMixedSpeedAccelDist] accel_qpps: %ld, m1_qpps: "
           "%ld, m1_max_dist: %ld, m2_qpps: %ld, "
           "m2_max_dist: %ld",
           accel_quad_pulses_per_second, m1_quad_pulses_per_second,
           m1_max_distance, m2_quad_pulses_per_second, m2_max_distance);
  TMicroRos::singleton().PublishDiagnostic(msg);
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetCurrents] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetEncoderM1] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetEncoderM2] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetLogicBattery] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetMainBattery] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetSpeedM1a] fail");
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
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetSpeedM2b] fail");
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
               "ERROR [TRoboClaw::GetVersion] version mismatch, found: '%s'",
               version);
      TMicroRos::singleton().PublishDiagnostic(msg);
      return false;
    } else {
      snprintf(msg, sizeof(msg),
               "INFO [TRoboClaw::GetVersion] version match, found: '%s'",
               version);
      TMicroRos::singleton().PublishDiagnostic(msg);
      version_matched = true;
      return true;
    }
  } else if (!version_matched) {
    TMicroRos::singleton().PublishDiagnostic(
        "ERROR [TRoboClaw::GetVersion fail");
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
               "ERROR [TRoboClaw::Loop] RUNAWAY for motor: %s, "
               "duration_since_last_runaway_check_for_encoder: %-2.3f",
               motor_name, duration_since_last_runaway_check_for_encoder);
      TMicroRos::singleton().PublishDiagnostic(msg);
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

void TRoboClaw::loop() {
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
      if (GetSpeedM1() && GetSpeedM2()) {
        g_state_ = kEncoderM1;
      } else {
        Reconnect();
        g_state_ = kVersion;
      }
      break;

    case kEncoderM1:
      if (GetEncoderM1() && GetEncoderM2()) {
        g_state_ = kSpeedM1;
        CheckForRunaway(kLeftMotor);
        CheckForRunaway(kRightMotor);
        PublishOdometry();
      } else {
        Reconnect();
        g_state_ = kVersion;
      }
      break;

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

void TRoboClaw::PublishOdometry() {
  static const float inter_wheel_distance = 0.395;
  static const uint32_t quad_pulses_per_revolution = 1000;
  static const float wheel_radius_meters = 0.05;
  static const float wheel_circumference = wheel_radius_meters * 2 * PI;
  static uint32_t last_checked_encoder_time_microseconds = micros();
  static int32_t last_m1_encoder = g_encoder_m1_;
  static int32_t last_m2_encoder = g_encoder_m2_;
  static float x_pos = 0.0;
  static float y_pos = 0.0;
  static float heading = 0.0;

  static bool initialized = false;
  if (!initialized) {
    // Code is here because static initialization of x_pos and y_pos didn't work.
    // heading = 0.0;
    // last_checked_encoder_time_microseconds = micros();
    // last_m1_encoder = g_encoder_m1_;
    // last_m2_encoder = g_encoder_m2_;
    x_pos = 0.0;
    y_pos = 0.0;
    initialized = true;
  {
    char m[256];
    snprintf(m, sizeof(m),
             " starting x_pos: %3.4f"
             ", y_pos: %3.4f",
             x_pos, y_pos);
    TMicroRos::singleton().PublishDiagnostic(m);
  }
  }
  // {
  //   char m[256];
  //   snprintf(m, sizeof(m),
  //            " starting x_pos: %3.4f"
  //            ", y_pos: %3.4f",
  //            x_pos, y_pos);
  //   TMicroRos::singleton().PublishDiagnostic(m);
  // }

  // {
  //   char m[256];
  //   snprintf(m, sizeof(m),
  //            "PRE heading: %3.4f"
  //            ", g_encoder_m1_: %ld"
  //            ", g_encoder_m2_: %ld"
  //            ", last_m1_encoder: %ld"
  //            ", last_m2_encoder: %ld",
  //            heading, g_encoder_m1_, g_encoder_m2_, last_m1_encoder,
  //            last_m2_encoder);
  //   TMicroRos::singleton().PublishDiagnostic(m);
  // }
  uint32_t now_microseconds = micros();
  float delta_time_secs =
      (now_microseconds - last_checked_encoder_time_microseconds) / 1'000'000.0;
  float delta_time_minutes = delta_time_secs / 60.0;
  last_checked_encoder_time_microseconds = now_microseconds;

  int32_t delta_m1_encoder = g_encoder_m1_ - last_m1_encoder;
  int32_t delta_m2_encoder = g_encoder_m2_ - last_m2_encoder;
  last_m1_encoder = g_encoder_m1_;
  last_m2_encoder = g_encoder_m2_;

  float rpm_m1 = ((float)delta_m1_encoder / (float)quad_pulses_per_revolution) /
                 delta_time_minutes;
  float rpm_m2 = ((float)delta_m2_encoder / (float)quad_pulses_per_revolution) /
                 delta_time_minutes;

  float average_rps_x = ((rpm_m1 + rpm_m2) / 2.0) / 60.0;
  float velocity_x = average_rps_x * wheel_circumference;
  float velocity_y = 0;
  float average_rps_angle =
      (-rpm_m1 + rpm_m2) / 2.0 /
      60.0;  // m1 in equation represents right motor, m2 is left.
  float anglular_velocity_z_rps =
      (average_rps_angle * wheel_circumference) / (inter_wheel_distance / 2.0);

  // {
  //   char m[256];
  //   snprintf(m, sizeof(m),
  //            " #5 x_pos: %3.4f"
  //            ", y_pos: %3.4f",
  //            x_pos, y_pos);
  //   TMicroRos::singleton().PublishDiagnostic(m);
  // }
  float delta_heading = anglular_velocity_z_rps * delta_time_secs;
  float cos_h = cos(heading);
  float sin_h = sin(heading);
  float delta_x = (velocity_x * cos_h - velocity_y * sin_h) * delta_time_secs;
  float delta_y = (velocity_x * sin_h + velocity_y * cos_h) * delta_time_secs;
  delta_heading = (((delta_m1_encoder * 1.0 / quad_pulses_per_revolution) *
                    wheel_circumference) -
                   ((delta_m2_encoder * 1.0 / quad_pulses_per_revolution) *
                    wheel_circumference)) /
                  inter_wheel_distance;

  x_pos += delta_x;
  y_pos += delta_y;
  heading += delta_heading;

  float q[4];
  EulerToQuaternion(0, 0, heading, q);

  // {
  //   char m[256];
  //   snprintf(m, sizeof(m),
  //            " x_pos: %3.4f"
  //            ", delta_x: %3.4f"
  //            ", velocity_x: %3.4f"
  //            ", velocity_y: %3.4f"
  //            ", delta_time_secs: %3.4f",
  //            x_pos, delta_x, velocity_x, velocity_y, delta_time_secs);
  //   TMicroRos::singleton().PublishDiagnostic(m);
  // }
  TMicroRos::singleton().PublishOdometry(x_pos, y_pos, velocity_x, velocity_y,
                                         anglular_velocity_z_rps, q);

  {
    // char msg[256];
    // snprintf(msg, sizeof(msg),
    //          "delta_time_secs: %3.4f"
    //          ", average_rps_x: %3.4f"
    //          ", average_rps_angle: %3.4f"
    //          ", delta_heading: %3.4f"
    //          ", cos_h: %3.4f"
    //          ", sin_h: %3.4f"
    //          ", dx: %3.4f"
    //          ", dy: %3.4f"
    //          ", x_pos: %3.4f"
    //          ", y_posy: %3.4f"
    //          ", heading: %3.4f",
    //          (double)delta_time_secs, (double)average_rps_x,
    //          (double)average_rps_angle, (double)delta_heading, (double)cos_h,
    //          (double)sin_h, (double)delta_x, (double)delta_y, (double)x_pos,
    //          (double)y_pos, (double)heading);
    // TMicroRos::singleton().PublishDiagnostic(msg);
    // snprintf(msg, sizeof(msg),
    //          "ODOM delta_m1_encoder: %ld"
    //          ", delta_m2_encoder: %ld"
    //          ", rpm_m`: %3.4f"
    //          ", rpm_m2: %3.4f"
    //          ", velocity_x: %3.4f"
    //          ", velocity_y: %3.4f"
    //          ", anglular_velocity_z_rps: %3.4f",
    //          delta_m1_encoder, delta_m2_encoder, (double)rpm_m1,
    //          (double)rpm_m2, (double)velocity_x, (double)velocity_y,
    //          (double)anglular_velocity_z_rps);
    // TMicroRos::singleton().PublishDiagnostic(msg);
  }

  // float average_delta_encoder = (delta_m1_encoder + delta_m2_encoder) / 2.0;
  // float pulses_traveled_in_one_second = average_delta_encoder /
  // delta_time_secs; float revolutions_in_one_second =
  //     pulses_traveled_in_one_second / quad_pulses_per_revolution;
  // float distance_in_one_second =
  //     (2 * 3.14159 * wheel_radius_meters) * revolutions_in_one_second;
  // float angle_traveled =
  //     (((-delta_m1_encoder + delta_m2_encoder) / 2.0) / delta_time_secs) /
  //     (inter_wheel_distance / 2.0);

  // float q[4]; // Quaternion equivalent of roll/pitch/yaw.
  // EulerToQuaternion(0.0, 0.0, angle_traveled, q);
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
      g_speed_m2_(0) {
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