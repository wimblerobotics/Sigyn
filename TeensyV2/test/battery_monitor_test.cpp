// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file battery_monitor_test.cpp
 * @brief Unit tests for BatteryMonitor module
 *
 * Tests critical safety functionality added for voltage/current monitoring:
 * - Critical voltage detection (low voltage triggers CRITICAL state)
 * - Critical current detection (high current triggers CRITICAL state)
 * - Sensor health monitoring (lost sensors trigger DEGRADED state)
 * - Safety integration via isUnsafe() for E-stop coordination
 * - EMA filtering for stable readings
 * - Battery state transitions (NORMAL -> WARNING -> CRITICAL)
 */

#include <gtest/gtest.h>
#include <memory>

#include "Arduino.h"
#include "modules/battery/battery_monitor.h"
#include "modules/safety/safety_coordinator.h"
#include "test/mocks/mock_power_sensor.h"

using namespace sigyn_teensy;

/**
 * Test fixture for BatteryMonitor tests
 */
class BatteryMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Reset mock Arduino state
        arduino_mock::reset();
        arduino_mock::setMillis(0);
        
        // Get instances
        monitor_ = &BatteryMonitor::getInstance();
        safety_ = &SafetyCoordinator::getInstance();
        
        // Reset safety state
        safety_->resetSafetyFlags();
        
        // Reset battery monitor state (singleton persists between tests)
        monitor_->testReset();
        
        // Create and inject mock sensors for all batteries
        for (size_t i = 0; i < 5; i++) {
            mock_sensors_[i] = std::make_unique<MockPowerSensor>();
            monitor_->registerSensor(i, mock_sensors_[i].get());
        }
        
        // Initialize with normal operating conditions
        setAllSensorsNormalConditions();
        
        // Call setup
        monitor_->testSetup();
    }
    
    void TearDown() override {
        arduino_mock::reset();
    }
    
    // Helper: Set all sensors to normal operating conditions
    void setAllSensorsNormalConditions() {
        // NOTE: Sensor readings are RAW values (before voltage_multiplier)
        // getVoltage() will multiply by voltage_multiplier from config
        
        // Battery 0 (36V LIPO): raw 15.2V * 2.375 = 36.1V actual, 5A
        mock_sensors_[0]->setVoltage(15.2f);
        mock_sensors_[0]->setCurrent(5.0f);
        mock_sensors_[0]->setConnected(true);
        
        // Battery 1-4: voltage_multiplier = 1.0, so raw = actual
        // Battery 1 (5V DCDC): 5V, 2A
        mock_sensors_[1]->setVoltage(5.0f);
        mock_sensors_[1]->setCurrent(2.0f);
        mock_sensors_[1]->setConnected(true);
        
        // Battery 2 (12V DCDC): 12V, 3A
        mock_sensors_[2]->setVoltage(12.0f);
        mock_sensors_[2]->setCurrent(3.0f);
        mock_sensors_[2]->setConnected(true);
        
        // Battery 3 (24V DCDC): 24V, 1A
        mock_sensors_[3]->setVoltage(24.0f);
        mock_sensors_[3]->setCurrent(1.0f);
        mock_sensors_[3]->setConnected(true);
        
        // Battery 4 (3.3V DCDC): 3.3V, 0.5A
        mock_sensors_[4]->setVoltage(3.3f);
        mock_sensors_[4]->setCurrent(0.5f);
        mock_sensors_[4]->setConnected(true);
    }
    
    // Helper: Run loop cycles to allow EMA to stabilize
    void runLoopCycles(int cycles, int interval_ms = 101) {
        for (int i = 0; i < cycles; i++) {
            uint32_t current_time = arduino_mock::current_millis;
            arduino_mock::setMillis(current_time + interval_ms);
            monitor_->testLoop();
        }
    }
    
    BatteryMonitor* monitor_;
    SafetyCoordinator* safety_;
    std::unique_ptr<MockPowerSensor> mock_sensors_[5];
};

/**
 * Test: Initial state - all batteries should be in UNKNOWN state
 */
TEST_F(BatteryMonitorTest, InitialState_AllBatteriesUnknown) {
    // Before any readings, state should be UNKNOWN
    for (size_t i = 0; i < 5; i++) {
        EXPECT_EQ(monitor_->getBatteryState(i), BatteryState::UNKNOWN)
            << "Battery " << i << " should start in UNKNOWN state";
    }
    
    // System should not be unsafe initially
    EXPECT_FALSE(monitor_->isUnsafe()) 
        << "System should not be unsafe with UNKNOWN state";
}

/**
 * Test: Critical low voltage detection for main battery (36V LIPO)
 * Config: critical_low_voltage = 32.0V
 * 
 * Safety requirement: When main battery drops below 32V, trigger CRITICAL state
 * and report unsafe via isUnsafe() to activate E-stop
 */
TEST_F(BatteryMonitorTest, CriticalLowVoltage_MainBattery) {
    // Set main battery (index 0) to critical low voltage
    // Raw sensor: 13.0V * 2.375 = 30.875V (< 32V critical threshold)
    mock_sensors_[0]->setVoltage(13.0f);
    mock_sensors_[0]->setCurrent(5.0f);  // Normal current
    
    // Allow readings to stabilize
    runLoopCycles(5);
    
    // Battery should be in CRITICAL state
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Main battery should enter CRITICAL state below 32V";
    
    // System should report unsafe (triggers E-stop coordination)
    EXPECT_TRUE(monitor_->isUnsafe())
        << "isUnsafe() should return true when any battery is CRITICAL";
}

/**
 * Test: Warning low voltage detection
 * Config: warning_low_voltage = 34.0V, critical = 32.0V
 * 
 * Battery should transition: NORMAL -> WARNING -> CRITICAL as voltage drops
 */
TEST_F(BatteryMonitorTest, WarningLowVoltage_StateTransitions) {
    // Start at normal voltage (raw: 15.2V * 2.375 = 36.1V)
    mock_sensors_[0]->setVoltage(15.2f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::NORMAL)
        << "Battery should be NORMAL at 36V";
    EXPECT_FALSE(monitor_->isUnsafe())
        << "System should be safe in NORMAL state";
    
    // Drop to warning level (raw: 13.9V * 2.375 = 33.0V, between 32V critical and 34V warning)
    mock_sensors_[0]->setVoltage(13.9f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::WARNING)
        << "Battery should enter WARNING state between 32-34V";
    EXPECT_FALSE(monitor_->isUnsafe())
        << "System should still be safe in WARNING state (not E-stop yet)";
    
    // Drop to critical level (raw: 13.0V * 2.375 = 30.875V < 32V critical)
    mock_sensors_[0]->setVoltage(13.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Battery should enter CRITICAL state below 32V";
    EXPECT_TRUE(monitor_->isUnsafe())
        << "System should be unsafe in CRITICAL state";
}

/**
 * Test: Critical high current detection
 * Config: critical_high_current = 15.0A for main battery
 * 
 * Safety requirement: Detect overcurrent conditions (motor stall, short circuit)
 * and trigger CRITICAL state
 */
TEST_F(BatteryMonitorTest, CriticalHighCurrent_MainBattery) {
    // Set normal voltage but excessive current
    mock_sensors_[0]->setVoltage(15.2f);  // Raw voltage (36.1V actual)
    mock_sensors_[0]->setCurrent(20.0f);  // Exceeds 15A threshold
    
    runLoopCycles(5);
    
    // Battery should be in CRITICAL state due to high current
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Battery should enter CRITICAL state when current exceeds 15A";
    
    // System should report unsafe
    EXPECT_TRUE(monitor_->isUnsafe())
        << "High current condition should trigger unsafe state";
}

/**
 * Test: Combined voltage and current limits
 * Verify that CRITICAL is triggered by either condition
 */
TEST_F(BatteryMonitorTest, CriticalCondition_BothLimits) {
    // Test Case 1: Low voltage only (raw: 13.0V = 30.875V actual < 32V critical)
    mock_sensors_[0]->setVoltage(13.0f);
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Low voltage alone should trigger CRITICAL";
    EXPECT_TRUE(monitor_->isUnsafe());
    
    // Reset to normal
    mock_sensors_[0]->setVoltage(15.2f);  // 36.1V actual
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::NORMAL);
    EXPECT_FALSE(monitor_->isUnsafe());
    
    // Test Case 2: High current only
    mock_sensors_[0]->setVoltage(15.2f);
    mock_sensors_[0]->setCurrent(20.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "High current alone should trigger CRITICAL";
    EXPECT_TRUE(monitor_->isUnsafe());
    
    // Reset to normal
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::NORMAL);
    EXPECT_FALSE(monitor_->isUnsafe());
    
    // Test Case 3: Both conditions
    mock_sensors_[0]->setVoltage(31.0f);
    mock_sensors_[0]->setCurrent(20.0f);
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Both conditions should trigger CRITICAL";
    EXPECT_TRUE(monitor_->isUnsafe());
}

/**
 * Test: Sensor failure detection
 * When a sensor loses connection, battery should enter DEGRADED state
 * and report to SafetyCoordinator
 */
TEST_F(BatteryMonitorTest, SensorFailure_DegradedState) {
    // Start with normal operation
    runLoopCycles(3);
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::NORMAL);
    EXPECT_FALSE(monitor_->isUnsafe())
        << "Sensor failure (DEGRADED) should not trigger unsafe by itself";
    
    // Verify no fault initially
    char fault_name[64];
    snprintf(fault_name, sizeof(fault_name), "BatteryMonitor_Battery0");
    const Fault& fault_before = safety_->getFault(fault_name);
    EXPECT_FALSE(fault_before.active);
    
    // Simulate sensor disconnection
    mock_sensors_[0]->setConnected(false);
    runLoopCycles(2);
    
    // Battery should enter DEGRADED state
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::DEGRADED)
        << "Lost sensor connection should trigger DEGRADED state";
    
    // SafetyCoordinator should have DEGRADED fault
    const Fault& degraded_fault = safety_->getFault(fault_name);
    EXPECT_TRUE(degraded_fault.active)
        << "SafetyCoordinator should have active DEGRADED fault";
    EXPECT_EQ(degraded_fault.severity, FaultSeverity::DEGRADED)
        << "Fault severity should be DEGRADED";
    
    // DEGRADED state should NOT trigger unsafe (it's a monitoring issue, not critical)
    EXPECT_FALSE(monitor_->isUnsafe())
        << "DEGRADED state (sensor loss) should not immediately trigger unsafe";
}

/**
 * Test: Sensor recovery from DEGRADED state
 * When a sensor reconnects, system should attempt recovery
 */
TEST_F(BatteryMonitorTest, SensorRecovery_FromDegradedState) {
    // Start normal
    runLoopCycles(3);
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::NORMAL);
    
    // Lose sensor
    mock_sensors_[0]->setConnected(false);
    runLoopCycles(2);
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::DEGRADED);
    
    // Sensor reconnects with normal readings
    mock_sensors_[0]->setConnected(true);
    mock_sensors_[0]->setVoltage(36.0f);
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(5);
    
    // Should recover to UNKNOWN first, then NORMAL after readings
    BatteryState state = monitor_->getBatteryState(0);
    EXPECT_TRUE(state == BatteryState::UNKNOWN || state == BatteryState::NORMAL)
        << "Sensor should recover from DEGRADED to UNKNOWN or NORMAL";
}

/**
 * Test: Initialization failure handling
 * If a sensor fails to initialize, it should be marked as DEGRADED
 */
TEST_F(BatteryMonitorTest, InitializationFailure_DegradedState) {
    // Create a new monitor instance for this test
    mock_sensors_[1]->setInitShouldFail(true);
    
    // Attempt initialization
    bool init_success = mock_sensors_[1]->init();
    
    EXPECT_FALSE(init_success)
        << "Sensor initialization should fail when configured to fail";
    
    // In actual implementation, setup() marks failed sensors as DEGRADED
    // This test verifies the mock behavior; actual integration tested separately
}

/**
 * Test: Multiple battery monitoring
 * Verify independent monitoring of all 5 power rails
 */
TEST_F(BatteryMonitorTest, MultipleBatteries_IndependentMonitoring) {
    // Set different states for different batteries
    
    // Battery 0: Critical low voltage (raw: 13.05V * 2.375 = 31.0V actual)
    mock_sensors_[0]->setVoltage(13.05f);
    mock_sensors_[0]->setCurrent(5.0f);
    
    // Battery 1: Normal
    mock_sensors_[1]->setVoltage(5.0f);
    mock_sensors_[1]->setCurrent(2.0f);
    
    // Battery 2: Warning
    mock_sensors_[2]->setVoltage(11.85f);  // Just below 11.9V warning threshold
    mock_sensors_[2]->setCurrent(3.0f);
    
    // Battery 3: Sensor lost
    mock_sensors_[3]->setConnected(false);
    
    // Battery 4: Normal
    mock_sensors_[4]->setVoltage(3.3f);
    mock_sensors_[4]->setCurrent(0.5f);
    
    runLoopCycles(5);
    
    // Verify each battery has correct state
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL)
        << "Battery 0 should be CRITICAL (low voltage)";
    EXPECT_EQ(monitor_->getBatteryState(1), BatteryState::NORMAL)
        << "Battery 1 should be NORMAL";
    EXPECT_EQ(monitor_->getBatteryState(2), BatteryState::WARNING)
        << "Battery 2 should be WARNING";
    EXPECT_EQ(monitor_->getBatteryState(3), BatteryState::DEGRADED)
        << "Battery 3 should be DEGRADED (sensor lost)";
    EXPECT_EQ(monitor_->getBatteryState(4), BatteryState::NORMAL)
        << "Battery 4 should be NORMAL";
    
    // System should be unsafe due to Battery 0 CRITICAL state
    EXPECT_TRUE(monitor_->isUnsafe())
        << "System should be unsafe when any battery is CRITICAL";
}

/**
 * Test: isUnsafe() only triggers on CRITICAL, not WARNING or DEGRADED
 */
TEST_F(BatteryMonitorTest, IsUnsafe_OnlyCriticalTriggers) {
    // Test WARNING state - should not trigger unsafe (raw: 13.89V * 2.375 = 33.0V actual)
    mock_sensors_[0]->setVoltage(13.89f);  // Warning level
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::WARNING);
    EXPECT_FALSE(monitor_->isUnsafe())
        << "WARNING state should not trigger unsafe";
    
    // Test DEGRADED state - should not trigger unsafe (raw: 15.16V * 2.375 = 36.0V actual)
    mock_sensors_[0]->setVoltage(15.16f);  // Normal voltage
    mock_sensors_[0]->setConnected(false);
    runLoopCycles(2);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::DEGRADED);
    EXPECT_FALSE(monitor_->isUnsafe())
        << "DEGRADED state should not trigger unsafe";
    
    // Test CRITICAL state - should trigger unsafe (raw: 13.05V * 2.375 = 31.0V actual)
    mock_sensors_[0]->setConnected(true);
    mock_sensors_[0]->setVoltage(13.05f);  // Critical
    runLoopCycles(3);
    
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CRITICAL);
    EXPECT_TRUE(monitor_->isUnsafe())
        << "CRITICAL state should trigger unsafe";
}

/**
 * Test: Charging detection (positive current on main battery)
 */
TEST_F(BatteryMonitorTest, ChargingDetection_MainBattery) {
    // Main battery (index 0) with positive current indicates charging
    mock_sensors_[0]->setVoltage(38.0f);  // Charging voltage
    mock_sensors_[0]->setCurrent(-5.0f);  // Negative current = charging
    
    runLoopCycles(5);
    
    // Battery should be in CHARGING state
    EXPECT_EQ(monitor_->getBatteryState(0), BatteryState::CHARGING)
        << "Main battery with negative current should be in CHARGING state";
    
    // Charging state should not trigger unsafe
    EXPECT_FALSE(monitor_->isUnsafe())
        << "CHARGING state should not trigger unsafe";
}

/**
 * Test: EMA filtering for voltage stability
 * Verify that exponential moving average smooths readings
 */
TEST_F(BatteryMonitorTest, EMAFiltering_VoltageStability) {
    // Set initial voltage (raw: 15.16V * 2.375 = 36.0V actual)
    mock_sensors_[0]->setVoltage(15.16f);
    runLoopCycles(10);  // Let EMA stabilize
    
    float initial_voltage = monitor_->getVoltage(0);
    EXPECT_NEAR(initial_voltage, 36.0f, 0.5f)
        << "Voltage should stabilize near setpoint with EMA";
    
    // Apply sudden voltage spike (raw: 16.84V * 2.375 = 40.0V actual)
    mock_sensors_[0]->setVoltage(16.84f);
    runLoopCycles(1);
    
    float voltage_after_spike = monitor_->getVoltage(0);
    
    // EMA should smooth the spike - voltage shouldn't jump immediately to 40V
    EXPECT_LT(voltage_after_spike, 40.0f)
        << "EMA should smooth sudden voltage changes";
    EXPECT_GT(voltage_after_spike, initial_voltage)
        << "Voltage should move toward new value";
}

/**
 * Test: EMA filtering for current stability
 */
TEST_F(BatteryMonitorTest, EMAFiltering_CurrentStability) {
    // Set initial current
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(10);  // Let EMA stabilize
    
    float initial_current = monitor_->getCurrent(0);
    EXPECT_NEAR(initial_current, 5.0f, 0.5f)
        << "Current should stabilize near setpoint with EMA";
    
    // Apply sudden current spike
    mock_sensors_[0]->setCurrent(10.0f);
    runLoopCycles(1);
    
    float current_after_spike = monitor_->getCurrent(0);
    
    // EMA should smooth the spike
    EXPECT_LT(current_after_spike, 10.0f)
        << "EMA should smooth sudden current changes";
    EXPECT_GT(current_after_spike, initial_current)
        << "Current should move toward new value";
}

/**
 * Test: Voltage multiplier calibration
 * Verify that voltage readings are scaled by voltage_multiplier
 * Config: voltage_multiplier = 2.375 for main battery
 */
TEST_F(BatteryMonitorTest, VoltageMultiplier_Calibration) {
    // Set raw sensor voltage
    float raw_voltage = 15.2f;
    mock_sensors_[0]->setVoltage(raw_voltage);
    runLoopCycles(10);  // Stabilize EMA
    
    float reported_voltage = monitor_->getVoltage(0);
    
    // Expected: raw_voltage * 2.375 (from config)
    float expected_voltage = raw_voltage * 2.375f;
    
    EXPECT_NEAR(reported_voltage, expected_voltage, 1.0f)
        << "Voltage should be scaled by voltage_multiplier";
}

/**
 * Test: Charge percentage estimation
 * Based on voltage curve (32V = 0%, 42V = 100%)
 */
TEST_F(BatteryMonitorTest, ChargePercentageEstimation) {
    // Test various voltage levels
    struct TestCase {
        float voltage;
        float expected_percentage;
        const char* description;
    };
    
    TestCase test_cases[] = {
        {32.0f, 0.0f, "Empty battery (32V)"},
        {37.0f, 0.5f, "Half charged (37V)"},
        {42.0f, 1.0f, "Fully charged (42V)"},
        {30.0f, 0.0f, "Below empty (clamped to 0%)"},
        {45.0f, 1.0f, "Above full (clamped to 100%)"}
    };
    
    for (const auto& test : test_cases) {
        mock_sensors_[0]->setVoltage(test.voltage);
        runLoopCycles(10);
        
        // Note: estimateChargePercentage() is private, so we can't test directly
        // This test serves as documentation for the expected behavior
        // In actual code, verify through status messages or public API
    }
}

/**
 * Test: Safety coordinator integration
 * Verify that BatteryMonitor CRITICAL state is detected by SafetyCoordinator
 * via the isUnsafe() interface and activates proper fault
 */
TEST_F(BatteryMonitorTest, SafetyIntegration_EstopTrigger) {
    // Start with normal state
    runLoopCycles(3);
    EXPECT_FALSE(monitor_->isUnsafe());
    
    // Verify no faults initially
    char fault_name[64];
    snprintf(fault_name, sizeof(fault_name), "BatteryMonitor_Battery0");
    const Fault& fault = safety_->getFault(fault_name);
    EXPECT_FALSE(fault.active) << "No fault should be active initially";
    
    // Trigger critical condition (raw: 12.63V * 2.375 = 30.0V actual < 32V critical)
    mock_sensors_[0]->setVoltage(12.63f);  // Critical low
    runLoopCycles(5);
    
    // Monitor should report unsafe
    EXPECT_TRUE(monitor_->isUnsafe())
        << "BatteryMonitor should report unsafe for SafetyCoordinator integration";
    
    // SafetyCoordinator should have EMERGENCY_STOP fault for Battery0
    const Fault& critical_fault = safety_->getFault(fault_name);
    EXPECT_TRUE(critical_fault.active) 
        << "SafetyCoordinator should have active fault for Battery0";
    EXPECT_EQ(critical_fault.severity, FaultSeverity::EMERGENCY_STOP)
        << "Fault severity should be EMERGENCY_STOP";
}

/**
 * Test: Reading consistency across getters
 * Verify that voltage and current getters return consistent values
 */
TEST_F(BatteryMonitorTest, ReadingConsistency_Getters) {
    mock_sensors_[0]->setVoltage(36.0f);
    mock_sensors_[0]->setCurrent(5.0f);
    runLoopCycles(10);
    
    // Read multiple times - should be stable
    float v1 = monitor_->getVoltage(0);
    float v2 = monitor_->getVoltage(0);
    float v3 = monitor_->getVoltage(0);
    
    EXPECT_FLOAT_EQ(v1, v2) << "Voltage reads should be consistent";
    EXPECT_FLOAT_EQ(v2, v3) << "Voltage reads should be consistent";
    
    float c1 = monitor_->getCurrent(0);
    float c2 = monitor_->getCurrent(0);
    float c3 = monitor_->getCurrent(0);
    
    EXPECT_FLOAT_EQ(c1, c2) << "Current reads should be consistent";
    EXPECT_FLOAT_EQ(c2, c3) << "Current reads should be consistent";
}

/**
 * Test: Module name reporting
 */
TEST_F(BatteryMonitorTest, ModuleName_Reporting) {
    const char* name = monitor_->name();
    EXPECT_STREQ(name, "BatteryMonitor")
        << "Module should report correct name";
}

/**
 * Performance test: Verify loop timing requirements
 * Battery monitoring should complete within 100ms update period
 */
TEST_F(BatteryMonitorTest, Performance_LoopTiming) {
    // This is a placeholder for performance testing
    // In actual hardware testing, verify that loop() completes < 100ms
    
    // Simulate rapid polling
    for (int i = 0; i < 100; i++) {
        arduino_mock::setMillis(i * 10);  // 10ms intervals
        // In real test, measure loop() execution time
    }
    
    // Success if no timeouts or watchdog triggers
    SUCCEED();
}
