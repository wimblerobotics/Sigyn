// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>

#include "Arduino.h"  // Mock Arduino
#include "common/core/config.h"
#include "modules/roboclaw/roboclaw_monitor.h"
#include "test/mocks/mock_roboclaw.h"

using namespace sigyn_teensy;

namespace {

static unsigned long g_test_millis = 1000;

int32_t expectedQpps(const RoboClawConfig& cfg, float linear_x, float angular_z, bool left) {
  const float v = left ? (linear_x - (angular_z * cfg.wheel_base_m / 2.0f))
                       : (linear_x + (angular_z * cfg.wheel_base_m / 2.0f));
  const float rpm = (v * 60.0f) / (static_cast<float>(M_PI) * cfg.wheel_diameter_m);
  const float qpps = (rpm / 60.0f) * static_cast<float>(cfg.quadrature_pulses_per_revolution);
  return static_cast<int32_t>(qpps);
}

uint32_t expectedDistance(const RoboClawConfig& cfg, int32_t qpps) {
  return static_cast<uint32_t>(std::abs(qpps) * cfg.max_seconds_commanded_travel_s);
}
}  // namespace

class RoboClawMonitorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arduino_mock::reset();
    g_test_millis += 1000;
    arduino_mock::setMillis(g_test_millis);

    monitor_ = &RoboClawMonitor::getInstance();
    mock_.reset();

    monitor_->setRoboClawForTesting(&mock_);
    monitor_->setConnectedForTesting(true);

    // Force a known config for deterministic tests.
    RoboClawConfig cfg;
    cfg.max_speed_qpps = 1392;
    cfg.default_acceleration = 3000;
    monitor_->updateConfig(cfg);

    // Ensure E-stop isn’t latched from another test run.
    monitor_->clearEmergencyStop();
  }

  RoboClawMonitor* monitor_ = nullptr;
  MockRoboClaw mock_;
};

TEST_F(RoboClawMonitorTest, SetVelocityCommand_Forward_CommandsExpectedQppsAndDistance) {
  const float linear_x = 0.50f;
  const float angular_z = 0.0f;

  // Simulate processVelocityCommands to trigger transmission
  monitor_->setVelocityCommand(linear_x, angular_z);
  monitor_->processVelocityCommandsForTest();

  // Switched to SpeedAccelM1M2 (continuous mix)
  // Calls last_cmd_speed_accel now, NOT speed_accel_distance
  // Actually, we need to check if we updated the test to verify SpeedAccelM1M2 usage
  // The monitor uses roboclaw_->SpeedAccelM1M2()

  // Verify that the continuous velocity function was called
  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  
  const RoboClawConfig cfg = monitor_->getConfig();
  const int32_t qpps_left = expectedQpps(cfg, linear_x, angular_z, /*left=*/true);
  const int32_t qpps_right = expectedQpps(cfg, linear_x, angular_z, /*left=*/false);

  const int32_t max_qpps = static_cast<int32_t>(monitor_->getConfig().max_speed_qpps);
  const auto clamp_qpps = [max_qpps](int32_t v) {
    if (v > max_qpps) return max_qpps;
    if (v < -max_qpps) return -max_qpps;
    return v;
  };

  const int32_t expected_left = clamp_qpps(qpps_left);
  const int32_t expected_right = clamp_qpps(qpps_right);

  // Check the values passed to SpeedAccelM1M2
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, expected_left);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, expected_right);
}

TEST_F(RoboClawMonitorTest, SetVelocityCommand_RotateInPlace_CommandsOppositeWheelSpeeds) {
  const float linear_x = 0.0f;
  const float angular_z = 1.0f;

  monitor_->setVelocityCommand(linear_x, angular_z);
  monitor_->processVelocityCommandsForTest();

  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);

  const RoboClawConfig cfg = monitor_->getConfig();
  const int32_t qpps_left = expectedQpps(cfg, linear_x, angular_z, /*left=*/true);
  const int32_t qpps_right = expectedQpps(cfg, linear_x, angular_z, /*left=*/false);

  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, qpps_left);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, qpps_right);

  // Rotate-in-place should be opposite directions (signs), unless both round to zero.
  if (qpps_left != 0 && qpps_right != 0) {
    EXPECT_LT(static_cast<int64_t>(qpps_left) * static_cast<int64_t>(qpps_right), 0);
  }
}

TEST_F(RoboClawMonitorTest, SetMotorSpeeds_ClampsToMaxSpeed) {
  RoboClawConfig cfg = monitor_->getConfig();
  cfg.max_speed_qpps = 100;
  monitor_->updateConfig(cfg);

  monitor_->setMotorSpeeds(500, -500);

  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 100);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, -100);
}

TEST_F(RoboClawMonitorTest, EmergencyStop_ForcesZeroAndBlocksFurtherCommands) {
  monitor_->setVelocityCommand(0.2f, 0.0f);
  monitor_->processVelocityCommandsForTest();
  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);

  // Reset flag for clearer assertion
  mock_.state.last_cmd_speed_accel.was_called = false;

  monitor_->setEmergencyStop();
  
  // Note: setEmergencyStop handles physical pins. 
  // It calls setMotorSpeeds(0,0) only if !CONTROLS_ROBOCLAW_ESTOP_PIN
  // Assuming defined for unit test environment or handled via logic check

#if !CONTROLS_ROBOCLAW_ESTOP_PIN
  // If we rely on soft stop
  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 0);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, 0);
#endif
  
  // Try sending a new command while in E-stop
  // monitor_->setMotorSpeeds(0, 0); // Don't manually reset, let E-stop logic handle it
  
  g_test_millis += 100;
  arduino_mock::setMillis(g_test_millis);

  mock_.state.last_cmd_speed_accel.was_called = false;
  monitor_->setVelocityCommand(0.5f, 0.0f);
  monitor_->processVelocityCommandsForTest();

  // It should internally command 0,0 due to the override in setMotorSpeeds
  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 0);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, 0);

  // While E-stop is active, even explicit motor commands should be forced to 0.
  mock_.state.last_cmd_speed_accel.was_called = false;
  monitor_->setMotorSpeeds(200, 200);

  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 0);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, 0);

  // Validate the E-stop output pin was driven LOW.
  EXPECT_EQ(arduino_mock::digital_pins[ESTOP_OUTPUT_PIN], LOW);

  monitor_->clearEmergencyStop();
  EXPECT_EQ(arduino_mock::digital_pins[ESTOP_OUTPUT_PIN], HIGH);

  mock_.state.last_cmd_speed_accel.was_called = false;
  monitor_->setMotorSpeeds(50, 60);

  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 50);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, 60);
}

TEST_F(RoboClawMonitorTest, TestCommunication_Failure_TriggersEmergencyStop) {
  mock_.reset();
  mock_.state.read_version_ok = false;

  const bool ok = monitor_->testCommunicationForTesting();
  EXPECT_FALSE(ok);
  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
  EXPECT_EQ(arduino_mock::digital_pins[ESTOP_OUTPUT_PIN], LOW);
}

TEST_F(RoboClawMonitorTest, Safety_Overcurrent_TriggersEmergencyStop) {
  RoboClawConfig cfg = monitor_->getConfig();
  cfg.max_current_m1 = 1.0f;
  cfg.max_current_m2 = 1.0f;
  monitor_->updateConfig(cfg);

  monitor_->setMotorCurrentsForTesting(/*m1_amps=*/2.0f, /*m2_amps=*/0.0f, /*valid=*/true);
  monitor_->runSafetyChecksForTesting();

  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
  EXPECT_EQ(arduino_mock::digital_pins[ESTOP_OUTPUT_PIN], LOW);
}

TEST_F(RoboClawMonitorTest, Safety_Runaway_TriggersEmergencyStop) {
  RoboClawConfig cfg = monitor_->getConfig();
  cfg.runaway_check_interval_ms = 10;
  cfg.runaway_speed_threshold_qpps = 50;
  monitor_->updateConfig(cfg);

  monitor_->setRunawayDetectionInitializedForTesting(true);
  monitor_->setLastCommandedQppsForTesting(0, 0);
  monitor_->setMotorSpeedFeedbackForTesting(/*m1_qpps=*/200, /*m2_qpps=*/0, /*valid=*/true);

  arduino_mock::setMillis(cfg.runaway_check_interval_ms + 1);
  monitor_->runSafetyChecksForTesting();

  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
  EXPECT_EQ(arduino_mock::digital_pins[ESTOP_OUTPUT_PIN], LOW);
}

TEST_F(RoboClawMonitorTest, HandleTwistMessage_ParsesAndCommandsMotorSpeeds) {
  const float linear_x = 0.10f;
  const float angular_z = 0.20f;

  monitor_->handleTwistMessage("linear_x:0.10,angular_z:0.20");
  monitor_->processVelocityCommandsForTest();

  ASSERT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);

  const RoboClawConfig cfg = monitor_->getConfig();
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.accel, monitor_->getConfig().default_acceleration);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, expectedQpps(cfg, linear_x, angular_z, /*left=*/true));
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, expectedQpps(cfg, linear_x, angular_z, /*left=*/false));
}

TEST_F(RoboClawMonitorTest, DecodeErrorStatus_FormatsKnownAndUnknownBits) {
  char decoded0[64] = {0};
  monitor_->decodeErrorStatus(0, decoded0, sizeof(decoded0));
  EXPECT_STREQ(decoded0, "No errors");

  const uint32_t error = static_cast<uint32_t>(RoboClawError::E_STOP) |
                         static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR) |
                         0x80000000u;

  char decoded[256] = {0};
  monitor_->decodeErrorStatus(error, decoded, sizeof(decoded));
  const std::string decoded_str(decoded);
  EXPECT_NE(decoded_str.find("E_STOP"), std::string::npos);
  EXPECT_NE(decoded_str.find("M1_DRIVER_FAULT_ERROR"), std::string::npos);
  EXPECT_NE(decoded_str.find("UNKNOWN_ERROR_BITS:0x"), std::string::npos);
}

TEST_F(RoboClawMonitorTest, ResetErrors_WhenDisconnected_DoesNotCallRoboClaw) {
  monitor_->setConnectedForTesting(false);
  mock_.reset();

  monitor_->resetErrors();

  EXPECT_EQ(mock_.state.speed_accel_distance_calls, 0u);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.was_called, false);
  EXPECT_EQ(mock_.state.reset_encoders_calls, 0u);
  EXPECT_EQ(mock_.state.read_error_calls, 0u);
}

TEST_F(RoboClawMonitorTest, ResetErrors_WhenConnected_StopsAndResetsEncoders) {
  monitor_->setConnectedForTesting(true);
  mock_.reset();

  mock_.state.reset_encoders_ok = true;
  mock_.state.read_error_valid = true;
  mock_.state.read_error_value = 0;

  monitor_->resetErrors();

  EXPECT_EQ(mock_.state.last_cmd_speed_accel.was_called, true);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed1, 0);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.speed2, 0);
  EXPECT_EQ(mock_.state.last_cmd_speed_accel.accel, monitor_->getConfig().default_acceleration);
  EXPECT_EQ(mock_.state.reset_encoders_calls, 1u);
  EXPECT_EQ(mock_.state.read_error_calls, 1u);
}
