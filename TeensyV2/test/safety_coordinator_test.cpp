// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file safety_coordinator_test.cpp
 * @brief Unit tests for SafetyCoordinator module
 *
 * Tests fault tracking and integration with TemperatureMonitor
 */

#include <gtest/gtest.h>

#include "Arduino.h"  // Mock Arduino
#include "mock_analog_reader.h"
#include "modules/roboclaw/roboclaw_monitor.h"
#include "modules/sensors/temperature_monitor.h"
#include "modules/safety/safety_coordinator.h"
#include "test/mocks/mock_roboclaw.h"

using namespace sigyn_teensy;

/**
 * Test fixture for SafetyCoordinator tests
 */
class SafetyCoordinatorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Reset mock Arduino state before each test
    arduino_mock::reset();
    
    // Create mock analog reader
    mock_reader = std::make_unique<MockAnalogReader>();
    
    // Get instances and inject mock
    temp_monitor = &TemperatureMonitor::getInstance();
    temp_monitor->setAnalogReader(mock_reader.get());
    
    safety = &SafetyCoordinator::getInstance();
    
    // Reset state completely
    temp_monitor->resetSystemStatistics();
    temp_monitor->resetSafetyFlags();
    safety->resetSafetyFlags();
    
    // Set baseline temperature to 15°C (~59°F)
    mock_reader->setTemperature(25, 15.0f);
    mock_reader->setTemperature(26, 15.0f);
    
    // Initialize both modules
    temp_monitor->setup();
    safety->testSetup();
    
    // Let system stabilize at baseline (3 readings at 100ms intervals)
    for (int i = 0; i < 3; i++) {
      arduino_mock::setMillis(i * 100);
      temp_monitor->loop();
      safety->testLoop();
    }
    
    // Reset time for actual test
    arduino_mock::setMillis(0);
  }
  
  void TearDown() override {
    arduino_mock::reset();
  }
  
  std::unique_ptr<MockAnalogReader> mock_reader;
  TemperatureMonitor* temp_monitor;
  SafetyCoordinator* safety;
};

class SafetyCoordinatorRoboClawIntegrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    arduino_mock::reset();
    arduino_mock::setMillis(0);

    safety_ = &SafetyCoordinator::getInstance();
    monitor_ = &RoboClawMonitor::getInstance();

    monitor_->setRoboClawForTesting(&mock_);
    monitor_->setConnectedForTesting(true);

    // Ensure a known config.
    RoboClawConfig cfg = monitor_->getConfig();
    cfg.max_current_m1 = 100.0f;
    cfg.max_current_m2 = 100.0f;
    cfg.runaway_check_interval_ms = 10;
    cfg.runaway_speed_threshold_qpps = 50;
    monitor_->updateConfig(cfg);

    // Reset state so faults/pins don't leak between tests.
    safety_->resetSafetyFlags();
    monitor_->resetErrors();
    monitor_->clearEmergencyStop();
  }

  void TearDown() override { arduino_mock::reset(); }

  SafetyCoordinator* safety_ = nullptr;
  RoboClawMonitor* monitor_ = nullptr;
  MockRoboClaw mock_;
};

/**
 * Test: SafetyCoordinator tracks temperature warning fault
 */
TEST_F(SafetyCoordinatorTest, Integration_TemperatureWarningFault) {
  // Verify initial state - no faults
  const Fault& temp_fault = safety->getFault("TemperatureMonitor");
  EXPECT_FALSE(temp_fault.active);
  EXPECT_FALSE(safety->isUnsafe());
  
  // Fill buffer with baseline 15°C for 5 seconds (100 readings at 100ms intervals)
  for (int i = 0; i < 100; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify still no faults after stable period
  EXPECT_FALSE(safety->getFault("TemperatureMonitor").active);
  EXPECT_FALSE(safety->isUnsafe());
  
  // Ramp temperature slowly from 15°C to 71°C over 34 seconds
  // This ensures any 5-second window shows < 100°C/min
  // (56°C over 34s = 98.8°C/min average, any 5-second span < 100°C/min)
  for (int i = 0; i < 340; i++) {
    float current_temp = 15.0f + (56.0f * i / 340.0f);
    mock_reader->setTemperature(25, current_temp);
    mock_reader->setTemperature(26, current_temp);
    
    arduino_mock::setMillis(5000 + (i * 100));  // Continue from buffer fill
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Hold at 71°C for 2 seconds to allow EMA filter to converge (20 readings)
  for (int i = 0; i < 20; i++) {
    arduino_mock::setMillis(39000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // After ramping to 71°C, should have warning but NOT critical and NOT thermal runaway
  const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);

  EXPECT_TRUE(status.warning_high);
  EXPECT_FALSE(status.critical_high);
  EXPECT_FALSE(status.thermal_runaway);

  // Warnings are not "unsafe" (E-stop) for TemperatureMonitor
  EXPECT_FALSE(temp_monitor->isUnsafe());
  
  // SafetyCoordinator should track this as a temperature fault
  const Fault& fault_after_warning = safety->getFault("TemperatureMonitor");
  EXPECT_TRUE(fault_after_warning.active);
  EXPECT_EQ(fault_after_warning.severity, FaultSeverity::WARNING);
  EXPECT_STREQ(fault_after_warning.source, "TemperatureMonitor");

  // Warnings should not trigger E-stop
  EXPECT_FALSE(safety->isUnsafe());
}

TEST_F(SafetyCoordinatorRoboClawIntegrationTest, Integration_RoboClawOvercurrent_ActivatesCoordinatorFault) {
  RoboClawConfig cfg = monitor_->getConfig();
  cfg.max_current_m1 = 1.0f;
  cfg.max_current_m2 = 1.0f;
  monitor_->updateConfig(cfg);

  EXPECT_FALSE(safety_->getFault("RoboClawMonitor").active);
  EXPECT_FALSE(safety_->isUnsafe());

  monitor_->setMotorCurrentsForTesting(/*m1_amps=*/2.0f, /*m2_amps=*/0.0f, /*valid=*/true);
  monitor_->runSafetyChecksForTesting();

  const Fault& f = safety_->getFault("RoboClawMonitor");
  EXPECT_TRUE(f.active);
  EXPECT_EQ(f.severity, FaultSeverity::EMERGENCY_STOP);
  EXPECT_TRUE(safety_->isUnsafe());
  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
}

TEST_F(SafetyCoordinatorRoboClawIntegrationTest, Integration_RoboClawRunaway_ActivatesCoordinatorFault) {
  EXPECT_FALSE(safety_->getFault("RoboClawMonitor").active);

  monitor_->setRunawayDetectionInitializedForTesting(true);
  monitor_->setLastCommandedQppsForTesting(0, 0);
  monitor_->setMotorSpeedFeedbackForTesting(/*m1_qpps=*/200, /*m2_qpps=*/0, /*valid=*/true);

  arduino_mock::setMillis(monitor_->getConfig().runaway_check_interval_ms + 1);
  monitor_->runSafetyChecksForTesting();

  const Fault& f = safety_->getFault("RoboClawMonitor");
  EXPECT_TRUE(f.active);
  EXPECT_EQ(f.severity, FaultSeverity::EMERGENCY_STOP);
  EXPECT_TRUE(safety_->isUnsafe());
  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
}

TEST_F(SafetyCoordinatorRoboClawIntegrationTest, Integration_RoboClawCommFail_ActivatesCoordinatorFault) {
  EXPECT_FALSE(safety_->getFault("RoboClawMonitor").active);

  mock_.state.read_version_ok = false;
  const bool ok = monitor_->testCommunicationForTesting();
  EXPECT_FALSE(ok);

  const Fault& f = safety_->getFault("RoboClawMonitor");
  EXPECT_TRUE(f.active);
  EXPECT_EQ(f.severity, FaultSeverity::EMERGENCY_STOP);
  EXPECT_TRUE(safety_->isUnsafe());
  EXPECT_TRUE(monitor_->isEmergencyStopActiveForTesting());
}

/**
 * Test: SafetyCoordinator tracks temperature critical fault
 */
TEST_F(SafetyCoordinatorTest, Integration_TemperatureCriticalFault) {
  // Fill buffer with baseline 60°C for 5 seconds
  mock_reader->setTemperature(25, 60.0f);
  mock_reader->setTemperature(26, 60.0f);
  
  for (int i = 0; i < 100; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify no faults at 60°C (below 70°C warning)
  EXPECT_FALSE(safety->getFault("TemperatureMonitor").active);
  
  // Jump to 86°C (above 85°C critical threshold)
  mock_reader->setTemperature(25, 86.0f);
  mock_reader->setTemperature(26, 86.0f);
  
  // Let system detect critical temperature
  for (int i = 0; i < 10; i++) {
    arduino_mock::setMillis(10000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Should now have critical fault
  const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);
  EXPECT_TRUE(status.critical_high);
  EXPECT_TRUE(temp_monitor->isUnsafe());
  
  // SafetyCoordinator should track as EMERGENCY_STOP severity
  const Fault& fault = safety->getFault("TemperatureMonitor");
  EXPECT_TRUE(fault.active);
  EXPECT_EQ(fault.severity, FaultSeverity::EMERGENCY_STOP);
  EXPECT_STREQ(fault.source, "TemperatureMonitor");
  EXPECT_TRUE(safety->isUnsafe());
}


/**
 * Test: SafetyCoordinator tracks thermal runaway fault
 */
TEST_F(SafetyCoordinatorTest, Integration_ThermalRunawayFault) {
  // Fill buffer with stable 60°C for 5 seconds (100 readings)
  mock_reader->setTemperature(25, 60.0f);
  mock_reader->setTemperature(26, 60.0f);
  
  for (int i = 0; i < 100; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify no faults at stable 60°C
  EXPECT_FALSE(safety->getFault("TemperatureMonitor").active);
  
  // Simulate stalled motor: rapid 11°C rise in 4 seconds (165°C/min)
  // This exceeds the 100°C/min threshold and should trigger thermal runaway
  for (int i = 0; i < 40; i++) {
    float current_temp = 60.0f + (11.0f * i / 40.0f);
    mock_reader->setTemperature(25, current_temp);
    mock_reader->setTemperature(26, current_temp);
    
    arduino_mock::setMillis(10000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
    
    // Check if thermal runaway detected during ramp
    const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);
    if (status.thermal_runaway) {
      // Found it! Verify SafetyCoordinator tracked the fault
      const Fault& fault = safety->getFault("TemperatureMonitor");
      EXPECT_TRUE(fault.active);
      EXPECT_EQ(fault.severity, FaultSeverity::EMERGENCY_STOP);
      EXPECT_TRUE(safety->isUnsafe());
      return;  // Test passed
    }
  }
  
  // If we get here, thermal runaway was never detected
  FAIL() << "Thermal runaway should have been detected during rapid temperature rise";
}

/**
 * Test: SafetyCoordinator clears fault when temperature recovers
 */
TEST_F(SafetyCoordinatorTest, Integration_TemperatureRecovery) {
  // Fill buffer with baseline 15°C for 5 seconds
  for (int i = 0; i < 100; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }

  // Ramp slowly into warning (avoid triggering thermal runaway)
  // 56°C over 34 seconds keeps any 5-second window below 100°C/min.
  for (int i = 0; i < 340; i++) {
    float current_temp = 15.0f + (56.0f * i / 340.0f);  // 15 -> 71°C
    mock_reader->setTemperature(25, current_temp);
    mock_reader->setTemperature(26, current_temp);

    arduino_mock::setMillis(10000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
  }

  // Hold at 71°C for 2 seconds so EMA settles
  mock_reader->setTemperature(25, 71.0f);
  mock_reader->setTemperature(26, 71.0f);
  for (int i = 0; i < 20; i++) {
    arduino_mock::setMillis(44000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify warning fault is active
  EXPECT_TRUE(safety->getFault("TemperatureMonitor").active);
  EXPECT_FALSE(temp_monitor->isUnsafe());
  EXPECT_FALSE(safety->isUnsafe());
  EXPECT_EQ(safety->getFault("TemperatureMonitor").severity, FaultSeverity::WARNING);
  
  // Cool down well below warning threshold
  mock_reader->setTemperature(25, 60.0f);
  mock_reader->setTemperature(26, 60.0f);

  // Wait for recovery (50 readings at 100ms = 5 seconds for EMA + threshold clearing)
  for (int i = 0; i < 50; i++) {
    arduino_mock::setMillis(46000 + (i * 100));
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Temperature monitor should clear warning
  const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);
  EXPECT_FALSE(status.warning_high);
  EXPECT_FALSE(temp_monitor->isUnsafe());
  
  // SafetyCoordinator should clear the fault
  const Fault& fault = safety->getFault("TemperatureMonitor");
  EXPECT_FALSE(fault.active);
  EXPECT_FALSE(safety->isUnsafe());
}

/**
 * Test: Low temperature warning is tracked by SafetyCoordinator
 */
TEST_F(SafetyCoordinatorTest, Integration_LowTemperatureWarning) {
  // Start at baseline 15°C
  // Cool to 2°C (below 5°C warning threshold, above 0°C critical)
  mock_reader->setTemperature(25, 2.0f);
  mock_reader->setTemperature(26, 2.0f);
  
  // Let system detect low temperature (25 readings at 100ms = 2.5s for EMA to converge)
  for (int i = 0; i < 25; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Should have low temperature warning
  const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);
  EXPECT_TRUE(status.warning_low);
  EXPECT_FALSE(status.critical_low);
  EXPECT_FALSE(temp_monitor->isUnsafe());
  
  // SafetyCoordinator should track this
  const Fault& fault = safety->getFault("TemperatureMonitor");
  EXPECT_TRUE(fault.active);
  EXPECT_EQ(fault.severity, FaultSeverity::WARNING);
  EXPECT_FALSE(safety->isUnsafe());
}

/**
 * Test: Low temperature critical is tracked by SafetyCoordinator
 */
TEST_F(SafetyCoordinatorTest, Integration_LowTemperatureCritical) {
  // Cool to -10°C (well below 0°C critical threshold, avoids EMA not crossing)
  mock_reader->setTemperature(25, -10.0f);
  mock_reader->setTemperature(26, -10.0f);
  
  // Let system detect critical low temperature (50 readings at 100ms = 5s for EMA)
  for (int i = 0; i < 50; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Should have critical low temperature
  const TemperatureSensorStatus& status = temp_monitor->getSensorStatus(0);
  EXPECT_TRUE(status.critical_low);
  EXPECT_TRUE(temp_monitor->isUnsafe());
  
  // SafetyCoordinator should track as EMERGENCY_STOP
  const Fault& fault = safety->getFault("TemperatureMonitor");
  EXPECT_TRUE(fault.active);
  EXPECT_EQ(fault.severity, FaultSeverity::EMERGENCY_STOP);
  EXPECT_TRUE(safety->isUnsafe());
}
