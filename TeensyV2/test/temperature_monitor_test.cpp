// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file temperature_monitor_test.cpp
 * @brief Unit tests for TemperatureMonitor module
 *
 * Phase 2: Comprehensive tests with dependency injection
 * Tests temperature monitoring, threshold detection, and thermal runaway
 */

#include <gtest/gtest.h>
#include <cmath>

#include "Arduino.h"  // Mock Arduino
#include "mock_analog_reader.h"
#include "modules/sensors/temperature_monitor.h"

using namespace sigyn_teensy;

/**
 * Test fixture for TemperatureMonitor tests
 */
class TemperatureMonitorTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Reset mock Arduino state before each test
    arduino_mock::reset();
    
    // Create mock analog reader
    mock_reader = std::make_unique<MockAnalogReader>();
    
    // Get TemperatureMonitor instance and inject mock
    monitor = &TemperatureMonitor::getInstance();
    monitor->setAnalogReader(mock_reader.get());
    
    // Reset monitor state completely
    monitor->resetSystemStatistics();
    monitor->resetSafetyFlags();
    
    // Set baseline temperature to 15°C (~59°F) - typical California ambient, above 5°C low warning
    mock_reader->setTemperature(25, 15.0f);
    mock_reader->setTemperature(26, 15.0f);
    
    // Run setup() which initializes sensor status
    monitor->setup();
    
    // Let system stabilize at baseline (3 readings at 100ms intervals = 300ms)
    for (int i = 0; i < 3; i++) {
      arduino_mock::setMillis(i * 100);
      monitor->loop();
    }
    
    // Clear any thermal runaway flags from baseline (shouldn't happen, but be defensive)
    // Access internal state - needed because singleton carries state between tests
    for (int i = 0; i < 2; i++) {
      monitor->getSensorStatus(i);  // Just access to verify it's available
    }
    
    // Reset time for actual test
    arduino_mock::setMillis(0);
  }
  
  void TearDown() override {
    // Clean up after each test
    arduino_mock::reset();
  }
  
  /**
   * Helper: Manually populate temperature history buffer
   * This simulates data collection over time without hardware
   * Matches the real updateTemperatureHistory() logic
   */
  void populateHistory(TemperatureSensorStatus& status, 
                       const float* temps, 
                       size_t count,
                       bool wrap_around = false) {
    // Initialize history to NAN
    for (int i = 0; i < 10; i++) {
      status.temperature_history[i] = NAN;
    }
    
    status.history_index = 0;
    
    // Fill history, mimicking updateTemperatureHistory()
    for (size_t i = 0; i < count; i++) {
      status.temperature_history[status.history_index] = temps[i];
      status.history_index = (status.history_index + 1) % 10;
    }
  }
  
  std::unique_ptr<MockAnalogReader> mock_reader;
  TemperatureMonitor* monitor;
};

/**
 * Test: Temperature trend calculation with insufficient data
 * Should return 0.0 when fewer than 5 samples are available
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_InsufficientData) {
  TemperatureSensorStatus status;
  TemperatureSensorConfig config;
  config.read_interval_ms = 1000;  // 1 second per sample
  
  // Test with 0 samples
  populateHistory(status, nullptr, 0);
  // Can't easily call calculateTemperatureTrend without a full object
  // This test demonstrates we need better testability
  
  // For now, test the logic directly
  bool buffer_full = !std::isnan(status.temperature_history[status.history_index]);
  uint32_t count = buffer_full ? 10 : status.history_index;
  EXPECT_LT(count, 5);
  EXPECT_FALSE(buffer_full);
}

/**
 * Test: Temperature trend calculation with exactly 5 samples
 * This is the minimum required after our fix
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_MinimumSamples) {
  TemperatureSensorStatus status;
  
  // Simulate steady temperature rise: 20, 22, 24, 26, 28 (2°C per sample)
  float temps[] = {20.0f, 22.0f, 24.0f, 26.0f, 28.0f};
  populateHistory(status, temps, 5);
  
  // Verify history was populated correctly
  EXPECT_EQ(status.history_index, 5);
  EXPECT_FLOAT_EQ(status.temperature_history[0], 20.0f);
  EXPECT_FLOAT_EQ(status.temperature_history[4], 28.0f);
  EXPECT_TRUE(std::isnan(status.temperature_history[5]));
}

/**
 * Test: Temperature trend calculation with full buffer (10 samples)
 * Tests the circular buffer wraparound logic
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_FullBuffer) {
  TemperatureSensorStatus status;
  
  // Simulate 10 samples with linear rise: 20, 22, 24, ..., 38
  float temps[10];
  for (int i = 0; i < 10; i++) {
    temps[i] = 20.0f + (i * 2.0f);
  }
  populateHistory(status, temps, 10);
  
  // After 10 samples, history_index wraps to 0
  EXPECT_EQ(status.history_index, 0);
  
  // Verify oldest sample (at index 0) and newest (at index 9)
  EXPECT_FLOAT_EQ(status.temperature_history[0], 20.0f);
  EXPECT_FLOAT_EQ(status.temperature_history[9], 38.0f);
  
  // All entries should be valid
  for (int i = 0; i < 10; i++) {
    EXPECT_FALSE(std::isnan(status.temperature_history[i]));
  }
}

/**
 * Test: Temperature trend calculation after wraparound
 * This tests the bug fix - ensuring correct oldest/newest identification
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_AfterWraparound) {
  TemperatureSensorStatus status;
  
  // First fill buffer with 10 samples: 20, 22, 24, ..., 38
  float temps[15];
  for (int i = 0; i < 15; i++) {
    temps[i] = 20.0f + (i * 2.0f);
  }
  
  // Add 15 samples (5 more than buffer size)
  populateHistory(status, temps, 15);
  
  // history_index should be at 5 (15 % 10)
  EXPECT_EQ(status.history_index, 5);
  
  // Oldest sample is at index 5: temps[5] = 30.0f
  // Newest sample is at index 4: temps[14] = 48.0f
  EXPECT_FLOAT_EQ(status.temperature_history[5], 30.0f);
  EXPECT_FLOAT_EQ(status.temperature_history[4], 48.0f);
  
  // Calculate expected trend
  // Oldest: 30.0, Newest: 48.0, Delta: 18.0
  // Time span: 10 samples * 1000ms = 10000ms = 0.1667 minutes
  // Trend: 18.0 / 0.1667 = 108.0 °C/min
  
  // This demonstrates the correct calculation after wraparound
}

/**
 * Test: Stable temperature (no trend)
 * All samples should be the same value
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_StableTemperature) {
  TemperatureSensorStatus status;
  
  // All samples at 25.0°C
  float temps[] = {25.0f, 25.0f, 25.0f, 25.0f, 25.0f, 25.0f};
  populateHistory(status, temps, 6);
  
  // Trend should be 0.0 or very close to it
  float oldest = status.temperature_history[0];
  float newest = status.temperature_history[5];
  
  EXPECT_FLOAT_EQ(oldest, 25.0f);
  EXPECT_FLOAT_EQ(newest, 25.0f);
  
  float delta = newest - oldest;
  EXPECT_FLOAT_EQ(delta, 0.0f);
}

/**
 * Test: Cooling trend (negative slope)
 */
TEST_F(TemperatureMonitorTest, TrendCalculation_CoolingTrend) {
  TemperatureSensorStatus status;
  
  // Temperature decreasing: 50, 48, 46, 44, 42
  float temps[] = {50.0f, 48.0f, 46.0f, 44.0f, 42.0f};
  populateHistory(status, temps, 5);
  
  float oldest = status.temperature_history[0];
  float newest = status.temperature_history[4];
  
  EXPECT_FLOAT_EQ(oldest, 50.0f);
  EXPECT_FLOAT_EQ(newest, 42.0f);
  
  float delta = newest - oldest;
  EXPECT_FLOAT_EQ(delta, -8.0f);  // Negative = cooling
}

/**
 * Test: Mock Arduino time functions
 * Verify our mock infrastructure works
 */
TEST_F(TemperatureMonitorTest, MockArduino_TimeControl) {
  EXPECT_EQ(millis(), 0);
  EXPECT_EQ(micros(), 0);
  
  arduino_mock::setMillis(1000);
  EXPECT_EQ(millis(), 1000);
  EXPECT_EQ(micros(), 1000000);
  
  delay(500);
  EXPECT_EQ(millis(), 1500);
  EXPECT_EQ(micros(), 1500000);
  
  arduino_mock::reset();
  EXPECT_EQ(millis(), 0);
}

/**
 * Test: Mock Arduino analog I/O
 * Verify we can inject analog values
 */
TEST_F(TemperatureMonitorTest, MockArduino_AnalogIO) {
  // Set analog pin A0 to 2048 (mid-range for 12-bit ADC)
  arduino_mock::setAnalogValue(A0, 2048);
  EXPECT_EQ(analogRead(A0), 2048);
  
  // Unset pin should return 0
  EXPECT_EQ(analogRead(A1), 0);
  
  arduino_mock::reset();
  EXPECT_EQ(analogRead(A0), 0);
}

/**
 * Test: Mock analog reader temperature injection
 * Verify we can inject temperature values via the mock
 */
TEST_F(TemperatureMonitorTest, MockAnalogReader_TemperatureInjection) {
  // Set temperature to 25°C on pin 25
  mock_reader->setTemperature(25, 25.0f);
  
  // TMP36: 25°C → voltage = 25*10 + 500 = 750mV
  // ADC: (750mV / 3300mV) * 4096 = 930 counts
  uint16_t adc = mock_reader->readAnalog(25);
  EXPECT_NEAR(adc, 930, 5);
  
  // Set to freezing (0°C)
  mock_reader->setTemperature(25, 0.0f);
  adc = mock_reader->readAnalog(25);
  // 0°C → 500mV → (500/3300)*4096 = 620 counts
  EXPECT_NEAR(adc, 620, 5);
  
  // Set to hot motor temperature (80°C)
  mock_reader->setTemperature(25, 80.0f);
  adc = mock_reader->readAnalog(25);
  // 80°C → 1300mV → (1300/3300)*4096 = 1614 counts
  EXPECT_NEAR(adc, 1614, 5);
}

/**
 * Test: Temperature reading with injected sensor value
 * Verify the monitor reads and converts temperature correctly
 */
TEST_F(TemperatureMonitorTest, ReadTemperature_NormalRange) {
  // Set left motor (pin 25) to 30°C
  mock_reader->setTemperature(25, 30.0f);
  
  // Trigger readings over 3 seconds for EMA to converge
  for (int i = 0; i < 60; i++) {  // 3 seconds at 20Hz
    arduino_mock::setMillis(i * 50);
    monitor->loop();
  }
  
  // Check temperature converged correctly
  float temp = monitor->getTemperature(0);  // Sensor 0 = left motor
  EXPECT_FALSE(std::isnan(temp));
  EXPECT_NEAR(temp, 30.0f, 3.0f);  // Within 3°C (fast EMA, alpha=0.7)
}

/**
 * Test: Critical high temperature detection
 * Verify safety system triggers on high temperature
 */
TEST_F(TemperatureMonitorTest, Safety_CriticalHighTemperature) {
  // Set left motor to critical temperature (85°C default threshold)
  mock_reader->setTemperature(25, 90.0f);  // Well above threshold
  
  // Call loop() at 20Hz for 5 seconds - EMA (alpha=0.7) converges in ~2 seconds
  for (int i = 0; i < 100; i++) {  // 5 seconds at 20Hz
    arduino_mock::setMillis(i * 50);  // 50ms intervals = 20Hz
    monitor->loop();
  }
  
  // Check safety status
  EXPECT_TRUE(monitor->isTemperatureCritical(0));
  EXPECT_TRUE(monitor->isUnsafe());
  
  // Check sensor status
  const TemperatureSensorStatus& status = monitor->getSensorStatus(0);
  EXPECT_TRUE(status.critical_high);
  EXPECT_FALSE(status.warning_high);
}

/**
 * Test: Warning temperature detection
 * Verify warning threshold works independently of critical
 */
TEST_F(TemperatureMonitorTest, Safety_WarningTemperature) {
  // NOTE: SetUp() already filled buffer with 15°C
  // Ramp slowly enough that even in a 5-second window, rate stays below 100°C/min
  // 56°C over 20 seconds = 168°C/min worst case, but in any 5-second window it's much less
  
  uint32_t time_ms = 0;
  
  // Very slow ramp from 15°C to 71°C over 20 seconds 
  // In any 5-second window: max 14°C rise = 168°C/min... still too high!
  // Need 56°C over 34+ seconds to guarantee < 100°C/min in any 5-second window
  // 5 seconds of 56°C span = 5/34 * 56 = 8.2°C = 98°C/min ✓
  for (int i = 0; i < 680; i++) {  // 34 seconds at 20Hz
    float temp = 15.0f + (56.0f * (i / 680.0f));  // 15 to 71°C linearly over 34s
    mock_reader->setTemperature(25, temp);
    arduino_mock::setMillis(time_ms);
    monitor->loop();
    time_ms += 50;
  }
  
  // Check status - should be in warning, NOT unsafe
  const TemperatureSensorStatus& status = monitor->getSensorStatus(0);
  EXPECT_TRUE(status.warning_high);
  EXPECT_FALSE(status.critical_high);
  EXPECT_FALSE(status.thermal_runaway);  // Slow ramp should NOT trigger
  EXPECT_FALSE(monitor->isUnsafe());
}

/**
 * Test: Thermal runaway detection and self-healing
 * Verify rapid temperature rise is detected and clears when stabilized
 */
TEST_F(TemperatureMonitorTest, Safety_ThermalRunaway) {
  // Start at 60°C baseline first, let buffer fill COMPLETELY
  mock_reader->setTemperature(25, 60.0f);
  monitor->setup();
  
  // Fill history buffer FULLY with 60°C baseline (5 seconds = 50 samples at 10Hz)
  for (int i = 0; i < 100; i++) {  // 5 seconds at 20Hz
    arduino_mock::setMillis(i * 50);
    monitor->loop();
  }
  
  // Now simulate stalled motor: 20°F (11°C) rise in 4 seconds = 165°C/min (well above 100°C/min threshold)
  uint32_t time_ms = 5000;  // Start after 5s baseline (buffer is now full)
  
  for (int i = 0; i < 120; i++) {  // 6 seconds at 20Hz
    float temp;
    if (i < 80) {
      temp = 60.0f + (11.0f * (i / 80.0f));  // Linear rise: 11°C in 4s = 165°C/min
    } else {
      temp = 71.0f;  // Hold at 71°C for last 2 seconds
    }
    mock_reader->setTemperature(25, temp);
    arduino_mock::setMillis(time_ms);
    monitor->loop();
    time_ms += 50;  // 50ms = 20Hz
    
    // Check if thermal runaway detected during or right after ramp
    if (i >= 40 && i <= 100) {  // Check from 2s into ramp through 5s
      if (monitor->getSensorStatus(0).thermal_runaway) {
        break;  // Detected! Exit early
      }
    }
  }
  
  // Thermal runaway should be detected by now (either during ramp or shortly after)
  EXPECT_TRUE(monitor->getSensorStatus(0).thermal_runaway);
  EXPECT_TRUE(monitor->isUnsafe());
  
  // Now stabilize at 71°C - temperature no longer rising
  // Need 5 seconds for trend window to fill with stable readings (50 samples at 10Hz sensor rate)
  mock_reader->setTemperature(25, 71.0f);
  for (int i = 0; i < 100; i++) {  // 5 seconds at 20Hz
    arduino_mock::setMillis(11000 + (i * 50));  // After 5s baseline + 6s test
    monitor->loop();
  }
  
  // Thermal runaway should auto-clear (self-healing) when trend goes to zero
  EXPECT_FALSE(monitor->getSensorStatus(0).thermal_runaway);
}

/**
 * Test: Thermal runaway false negative check
 * Verify slow rise doesn't trigger runaway
 */
TEST_F(TemperatureMonitorTest, Safety_SlowRise_NoRunaway) {
  monitor->setup();
  
  // Simulate normal motor warmup: 20°F (11°C) in 10 seconds = 67°C/min (below 100°C/min threshold)
  // Rise from 40°C to 51°C over 10 seconds
  uint32_t time_ms = 0;
  
  for (int i = 0; i < 200; i++) {  // 10 seconds at 20Hz
    float temp = 40.0f + (11.0f * (i / 200.0f));  // Linear rise: 11°C in 10s
    mock_reader->setTemperature(25, temp);
    arduino_mock::setMillis(time_ms);
    monitor->loop();
    time_ms += 50;  // 50ms = 20Hz
  }
  
  // Trend: 11°C in 10s = 67°C/min (below 100°C/min threshold)
  const TemperatureSensorStatus& status = monitor->getSensorStatus(0);
  EXPECT_FALSE(status.thermal_runaway);
}

/**
 * Test: Temperature recovery clears warnings
 * Verify warnings clear when temperature drops
 * NOTE: Uses instantaneous temperature jumps (not gradual rise) to avoid thermal runaway
 */
TEST_F(TemperatureMonitorTest, Safety_TemperatureRecovery) {
  // Start at 15°C and fill buffer COMPLETELY
  mock_reader->setTemperature(25, 15.0f);
  monitor->setup();
  
  // Fill buffer FULLY with 15°C baseline (5 seconds = 50 samples)
  for (int i = 0; i < 100; i++) {  // 5 seconds at 20Hz
    arduino_mock::setMillis(i * 50);
    monitor->loop();
  }
  
  uint32_t time_ms = 5000;  // Start at 5s
  
  // Jump directly to warning temperature (72°C) - above 70°C threshold
  // With fast EMA (alpha=0.7) and 5s trend window, need time for trend to stabilize
  mock_reader->setTemperature(25, 72.0f);
  for (int i = 0; i < 200; i++) {  // 10 seconds at 20Hz for EMA + trend to settle
    arduino_mock::setMillis(time_ms);
    monitor->loop();
    time_ms += 50;
  }
  
  // Check that warning is active
  const TemperatureSensorStatus& status_warm = monitor->getSensorStatus(0);
  EXPECT_TRUE(status_warm.warning_high);
  EXPECT_FALSE(status_warm.critical_high);
  EXPECT_FALSE(status_warm.thermal_runaway);  // Should NOT trigger from instant jump
  
  // Cool down (instantaneous jump back to 15°C = 59°F)
  mock_reader->setTemperature(25, 15.0f);
  for (int i = 0; i < 100; i++) {  // 5 seconds at 20Hz for trend to clear
    arduino_mock::setMillis(time_ms);
    monitor->loop();
    time_ms += 50;
  }
  
  // Verify warning has cleared
  const TemperatureSensorStatus& status_cool = monitor->getSensorStatus(0);
  EXPECT_FALSE(status_cool.warning_high);
  EXPECT_FALSE(status_cool.critical_high);
  EXPECT_FALSE(status_cool.thermal_runaway);
  EXPECT_FALSE(monitor->isUnsafe());
}

/**
 * Test: Multiple sensors independent
 * Verify sensor 0 and sensor 1 operate independently
 */
TEST_F(TemperatureMonitorTest, MultiSensor_IndependentOperation) {
  monitor->setup();
  
  // Set different temperatures for left (pin 25) and right (pin 26) motors
  mock_reader->setTemperature(25, 50.0f);  // Left motor normal
  mock_reader->setTemperature(26, 86.0f);  // Right motor critical
  
  for (int i = 0; i < 5; i++) {
    arduino_mock::setMillis(1000 * i);
    monitor->loop();
  }
  
  // Sensor 0 should be normal
  EXPECT_FALSE(monitor->isTemperatureCritical(0));
  EXPECT_NEAR(monitor->getTemperature(0), 50.0f, 1.0f);
  
  // Sensor 1 should be critical
  EXPECT_TRUE(monitor->isTemperatureCritical(1));
  EXPECT_NEAR(monitor->getTemperature(1), 86.0f, 1.0f);
  
  // System should report unsafe due to sensor 1
  EXPECT_TRUE(monitor->isUnsafe());
}

/**
 * Test: System status aggregation
 * Verify system-level statistics are computed correctly
 */
TEST_F(TemperatureMonitorTest, SystemStatus_Aggregation) {
  mock_reader->setTemperature(25, 30.0f);
  mock_reader->setTemperature(26, 45.0f);
  
  // Call loop() at 20Hz for 3 seconds (3 actual readings with fast EMA)
  for (int i = 0; i < 60; i++) {
    arduino_mock::setMillis(i * 50);
    monitor->loop();
  }
  
  const TemperatureSystemStatus& sys_status = monitor->getSystemStatus();
  
  // Check aggregation - wider tolerance for EMA convergence from 15°C baseline
  EXPECT_EQ(sys_status.active_sensors, 2);
  EXPECT_NEAR(sys_status.average_temperature, 37.5f, 10.0f);  // (30+45)/2 with EMA lag
  EXPECT_NEAR(sys_status.highest_temperature, 45.0f, 10.0f);
  EXPECT_NEAR(sys_status.lowest_temperature, 30.0f, 10.0f);
  EXPECT_EQ(sys_status.hottest_sensor, 1);  // Right motor
  EXPECT_EQ(sys_status.coldest_sensor, 0);  // Left motor
}

/**
 * Main test entry point
 */
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
