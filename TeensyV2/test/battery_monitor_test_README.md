# BatteryMonitor Test Suite

## Overview

Comprehensive unit tests for the `BatteryMonitor` module, focusing on critical safety functionality for voltage/current monitoring and E-stop integration.

## Test Coverage

### Safety-Critical Features

1. **Critical Voltage Detection**
   - Low voltage threshold enforcement (32V for main battery)
   - CRITICAL state activation below threshold
   - E-stop integration via `isUnsafe()`

2. **Critical Current Detection**
   - Overcurrent protection (15A threshold for main battery)
   - Detection of motor stall, short circuits
   - CRITICAL state activation on overcurrent

3. **Sensor Health Monitoring**
   - Sensor disconnection detection
   - DEGRADED state for lost sensors
   - Sensor recovery handling

4. **Battery State Management**
   - State transitions: UNKNOWN → NORMAL → WARNING → CRITICAL
   - CHARGING state detection (negative current)
   - DEGRADED state for sensor failures

### Functional Features

5. **Multi-Battery Monitoring**
   - Independent monitoring of 5 power rails
   - Individual state tracking
   - System-wide safety aggregation

6. **EMA Filtering**
   - Voltage reading stability
   - Current reading stability
   - Spike suppression

7. **Calibration**
   - Voltage multiplier verification
   - Charge percentage estimation

8. **Integration**
   - SafetyCoordinator integration
   - Module interface compliance

## Test Organization

### Test Fixture
```cpp
BatteryMonitorTest
```
- Sets up BatteryMonitor singleton
- Injects MockPowerSensor instances for all 5 batteries
- Provides helper methods for common scenarios

### Mock Infrastructure
```cpp
MockPowerSensor
```
- Implements `IPowerSensor` interface
- Allows test control of:
  - Voltage readings
  - Current readings
  - Connection state
  - Initialization success/failure

## Key Test Cases

### Safety Tests

**CriticalLowVoltage_MainBattery**
- Verifies CRITICAL state activation when voltage drops below 32V
- Confirms `isUnsafe()` returns true for E-stop coordination

**CriticalHighCurrent_MainBattery**
- Verifies CRITICAL state activation when current exceeds 15A
- Confirms overcurrent detection for motor faults

**CriticalCondition_BothLimits**
- Tests that either voltage OR current limit triggers CRITICAL
- Verifies independent limit enforcement

**WarningLowVoltage_StateTransitions**
- Tests state progression: NORMAL → WARNING → CRITICAL
- Verifies WARNING state (34V threshold) doesn't trigger E-stop
- Confirms only CRITICAL triggers `isUnsafe()`

### Sensor Health Tests

**SensorFailure_DegradedState**
- Verifies DEGRADED state on sensor disconnection
- Confirms DEGRADED doesn't immediately trigger unsafe

**SensorRecovery_FromDegradedState**
- Tests recovery path when sensor reconnects
- Verifies transition DEGRADED → UNKNOWN → NORMAL

**InitializationFailure_DegradedState**
- Tests handling of sensor init failures
- Confirms graceful degradation

### Monitoring Tests

**MultipleBatteries_IndependentMonitoring**
- Verifies independent state tracking for 5 batteries
- Tests mixed states (CRITICAL, NORMAL, WARNING, DEGRADED)
- Confirms system unsafe if ANY battery is CRITICAL

**IsUnsafe_OnlyCriticalTriggers**
- Validates that only CRITICAL state triggers `isUnsafe()`
- Confirms WARNING and DEGRADED do not trigger unsafe

**ChargingDetection_MainBattery**
- Tests charging state detection (negative current)
- Verifies charging doesn't trigger unsafe

### Filtering Tests

**EMAFiltering_VoltageStability**
- Verifies exponential moving average smooths voltage
- Tests spike suppression

**EMAFiltering_CurrentStability**
- Verifies EMA filtering for current readings
- Tests transient suppression

### Calibration Tests

**VoltageMultiplier_Calibration**
- Verifies voltage scaling by `voltage_multiplier` (2.375x for main battery)
- Tests calibration accuracy

**ChargePercentageEstimation**
- Documents charge estimation algorithm (32V=0%, 42V=100%)
- Validates boundary conditions

### Integration Tests

**SafetyIntegration_EstopTrigger**
- Tests BatteryMonitor → SafetyCoordinator integration
- Verifies E-stop trigger path via `isUnsafe()`

## Running Tests

### Quick Test Run
```bash
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2
pio test -e test
```

### Verbose Output
```bash
pio test -e test -v
```

### Run Specific Test
```bash
pio test -e test --filter "BatteryMonitorTest.CriticalLowVoltage_MainBattery"
```

### Run All Teensy Tests (Including Battery Monitor)
```bash
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2
test_teensy  # Alias defined in bashrc
```

## Test Configuration

Tests use the production battery configuration from `BatteryConfig`:

| Battery | Name | Critical V | Warning V | Critical I | Multiplier |
|---------|------|-----------|-----------|-----------|------------|
| 0 | 36VLIPO | 32.0V | 34.0V | 15.0A | 2.375x |
| 1 | 5VDCDC | 4.9V | 4.94V | 9.0A | 1.0x |
| 2 | 12VDCDC | 11.8V | 11.9V | 19.0A | 1.0x |
| 3 | 24VDCDC | 23.0V | 23.5V | 9.0A | 1.0x |
| 4 | 3.3VDCDC | 3.1V | 3.2V | 9.0A | 1.0x |

## Dependencies

- GoogleTest framework
- MockPowerSensor (test/mocks/mock_power_sensor.h)
- Arduino mock (test/mocks/Arduino.h)

## Safety Verification

These tests verify the safety requirements from:
- `Documentation/Requirements.md`
- `TODO_list.txt` (Safety System Critical Gaps)

### Verified Safety Properties

✅ **[CRITICAL] BatteryMonitor isUnsafe() Implementation**
- Low voltage detection (< 32V main battery)
- High current detection (> 15A main battery)  
- State aggregation across all batteries
- E-stop coordination via `isUnsafe()`

✅ **Sensor Health Monitoring**
- Sensor timeout detection
- Graceful degradation on sensor loss
- Recovery path when sensor reconnects

✅ **Battery State Tracking**
- CRITICAL state for safety violations
- WARNING state for early warnings
- DEGRADED state for sensor failures

## Future Enhancements

### Test Coverage Gaps
- [ ] I2C multiplexer interaction testing
- [ ] Actual hardware sensor timing verification
- [ ] Long-duration stability testing
- [ ] Temperature compensation testing

### Additional Test Scenarios
- [ ] Rapid voltage/current oscillations
- [ ] Multiple simultaneous sensor failures
- [ ] Battery hot-swap scenarios
- [ ] Power-on self-test verification

### Performance Testing
- [ ] Loop execution timing verification
- [ ] Memory usage profiling
- [ ] Interrupt latency measurement

## Related Documentation

- [BatteryMonitor Implementation](../modules/battery/battery_monitor.cpp)
- [IPowerSensor Interface](../modules/battery/interfaces/i_power_sensor.h)
- [Safety System Overview](../modules/safety/README.md)
- [Safety Requirements](../../Documentation/Requirements.md)
