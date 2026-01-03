// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "Arduino.h"  // Mock Arduino
#include "modules/roboclaw/roboclaw_monitor.h"
#include "test/mocks/mock_roboclaw.h"

using namespace sigyn_teensy;

namespace {
// Keep these in sync with the implementation constants in roboclaw_monitor.cpp.
constexpr float kWheelDiameterM = 0.102224144529039f;
constexpr float kWheelBaseM = 0.3906f;
constexpr uint32_t kQuadraturePulsesPerRev = 1000;
constexpr float kMaxSecondsCommandedTravel = 0.05f;

int32_t expectedQpps(float linear_x, float angular_z, bool left) {
  const float v = left ? (linear_x - (angular_z * kWheelBaseM / 2.0f)) : (linear_x + (angular_z * kWheelBaseM / 2.0f));
  const float rpm = (v * 60.0f) / (static_cast<float>(M_PI) * kWheelDiameterM);
  const float qpps = (rpm / 60.0f) * static_cast<float>(kQuadraturePulsesPerRev);
  return static_cast<int32_t>(qpps);
}

uint32_t expectedDistance(int32_t qpps) {
  return static_cast<uint32_t>(std::abs(qpps) * kMaxSecondsCommandedTravel);
}
}  // namespace

class RoboClawMonitorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arduino_mock::reset();
    arduino_mock::setMillis(0);

    monitor_ = &RoboClawMonitor::getInstance();
    mock_.reset();

    monitor_->setRoboClawForTesting(&mock_);
    monitor_->setConnectedForTesting(true);

    // Ensure E-stop isn’t latched from another test run.
    monitor_->clearEmergencyStop();
  }

  RoboClawMonitor* monitor_ = nullptr;
  MockRoboClaw mock_;
};

TEST_F(RoboClawMonitorTest, SetVelocityCommand_Forward_CommandsExpectedQppsAndDistance) {
  const float linear_x = 0.50f;
  const float angular_z = 0.0f;

  monitor_->setVelocityCommand(linear_x, angular_z);

  ASSERT_EQ(mock_.state.speed_accel_distance_calls, 1u);
  ASSERT_TRUE(mock_.state.last_cmd.was_called);

  const int32_t qpps_left = expectedQpps(linear_x, angular_z, /*left=*/true);
  const int32_t qpps_right = expectedQpps(linear_x, angular_z, /*left=*/false);

  const int32_t max_qpps = static_cast<int32_t>(monitor_->getConfig().max_speed_qpps);
  const auto clamp_qpps = [max_qpps](int32_t v) {
    if (v > max_qpps) return max_qpps;
    if (v < -max_qpps) return -max_qpps;
    return v;
  };

  const int32_t expected_left = clamp_qpps(qpps_left);
  const int32_t expected_right = clamp_qpps(qpps_right);

  EXPECT_EQ(mock_.state.last_cmd.speed1, expected_left);
  EXPECT_EQ(mock_.state.last_cmd.speed2, expected_right);

  EXPECT_EQ(mock_.state.last_cmd.distance1, expectedDistance(expected_left));
  EXPECT_EQ(mock_.state.last_cmd.distance2, expectedDistance(expected_right));

  EXPECT_EQ(mock_.state.last_cmd.flag, 1u);
}

TEST_F(RoboClawMonitorTest, SetVelocityCommand_RotateInPlace_CommandsOppositeWheelSpeeds) {
  const float linear_x = 0.0f;
  const float angular_z = 1.0f;

  monitor_->setVelocityCommand(linear_x, angular_z);

  ASSERT_EQ(mock_.state.speed_accel_distance_calls, 1u);
  ASSERT_TRUE(mock_.state.last_cmd.was_called);

  const int32_t qpps_left = expectedQpps(linear_x, angular_z, /*left=*/true);
  const int32_t qpps_right = expectedQpps(linear_x, angular_z, /*left=*/false);

  EXPECT_EQ(mock_.state.last_cmd.speed1, qpps_left);
  EXPECT_EQ(mock_.state.last_cmd.speed2, qpps_right);

  // Rotate-in-place should be opposite directions (signs), unless both round to zero.
  if (qpps_left != 0 && qpps_right != 0) {
    EXPECT_LT(static_cast<int64_t>(qpps_left) * static_cast<int64_t>(qpps_right), 0);
  }
}
