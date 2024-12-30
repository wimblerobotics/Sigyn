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

  bool left_motor_stalled = consecutiveStallFaultsLeftMotor > kMaxAllowedConsecutiveStallFaults;
  bool right_motor_stalled = consecutiveStallFaultsRightMotor > kMaxAllowedConsecutiveStallFaults;
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

void TRoboClaw::DoMixedSpeedDist(int32_t m1_quad_pulses_per_second, int32_t m1_max_distance,
                                 int32_t m2_quad_pulses_per_second, int32_t m2_max_distance) {
  if (TRelay::singleton().IsPoweredOn(TRelay::kMotorEStop) && (m1_quad_pulses_per_second == 0) &&
      (m2_quad_pulses_per_second == 0)) {
    TRelay::singleton().PowerOff(TRelay::kMotorEStop);  // UN E-stop the motors.
    TMicroRos::singleton().PublishDiagnostic(
        "INFO [TRoboClaw::DoMixedSpeedDist] Removing E-Stop because of zero "
        "velocity command");
  }

  g_roboclaw_.SpeedDistanceM1M2(kDeviceAddress, m1_quad_pulses_per_second, m1_max_distance,
                                m2_quad_pulses_per_second, m2_max_distance, 1);
}

void TRoboClaw::DoMixedSpeedAccelDist(uint32_t accel_quad_pulses_per_second,
                                      int32_t m1_quad_pulses_per_second, uint32_t m1_max_distance,
                                      int32_t m2_quad_pulses_per_second, uint32_t m2_max_distance) {
  if (TRelay::singleton().IsPoweredOn(TRelay::kMotorEStop) && (m1_quad_pulses_per_second == 0) &&
      (m2_quad_pulses_per_second == 0)) {
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
           accel_quad_pulses_per_second, m1_quad_pulses_per_second, m1_max_distance,
           m2_quad_pulses_per_second, m2_max_distance);
  TMicroRos::singleton().PublishDiagnostic(msg);
  g_roboclaw_.SpeedAccelDistanceM1M2(kDeviceAddress, accel_quad_pulses_per_second,
                                     m1_quad_pulses_per_second, m1_max_distance,
                                     m2_quad_pulses_per_second, m2_max_distance, 1);
}

float TRoboClaw::GetBatteryLogic() { return g_logic_battery_ / 10.0; }

float TRoboClaw::GetBatteryMain() { return g_main_battery_ / 10.0; }

bool TRoboClaw::GetCurrents() {
  int16_t currentM1;
  int16_t currentM2;
  bool valid = g_roboclaw_.ReadCurrents(kDeviceAddress, currentM1, currentM2);
  if (!valid) {
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetCurrents] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetEncoderM1] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetEncoderM2] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetLogicBattery] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetMainBattery] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetSpeedM1] fail");
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
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetSpeedM2] fail");
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
      snprintf(msg, sizeof(msg), "ERROR [TRoboClaw::GetVersion] version mismatch, found: '%s'",
               version);
      TMicroRos::singleton().PublishDiagnostic(msg);
      return false;
    } else {
      snprintf(msg, sizeof(msg), "INFO [TRoboClaw::GetVersion] version match, found: '%s'",
               version);
      TMicroRos::singleton().PublishDiagnostic(msg);
      version_matched = true;
      return true;
    }
  } else if (!version_matched) {
    TMicroRos::singleton().PublishDiagnostic("ERROR [TRoboClaw::GetVersion fail");
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
  static const uint32_t kMaxAllowedConsecutiveEncoderFaults =
      uint32_t(1566.0 * 0.1);  // Pulses/meter * <max-allowed-fault-distance-in-meters>.
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
      encoder_diff_per_second = abs(g_encoder_m1_ - last_checked_m1_encoder_value) /
                                duration_since_last_runaway_check_for_encoder;
      motor_name = "M1";
      break;
    case kRightMotor:
      duration_since_last_runaway_check_for_encoder =
          ((now_ms * 1.0) - last_checked_m2_encoder_time_ms) / 1000.0;
      encoder_diff_per_second = abs(g_encoder_m2_ - last_checked_m2_encoder_value) /
                                duration_since_last_runaway_check_for_encoder;
      motor_name = "M2";
      break;
  }

  if (encoder_diff_per_second > kEncoderCountFaultThresholdPerSecond) {
    // The encodeers are spinning too fast.
    // A fault is triggered only if the robot travels at least a certain
    // distance at this high speed.
    if (whichMotor == kLeftMotor) {
      accumulated_m1_encoder_diffs += abs(g_encoder_m1_ - last_checked_m1_encoder_value);
      runaway_fault = accumulated_m1_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
    } else {
      accumulated_m2_encoder_diffs += abs(g_encoder_m2_ - last_checked_m2_encoder_value);
      runaway_fault = accumulated_m2_encoder_diffs > kMaxAllowedConsecutiveEncoderFaults;
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

const void TRoboClaw::EulerToQuaternion(float roll, float pitch, float yaw, float* q) {
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