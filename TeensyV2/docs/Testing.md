# TeensyV2 Testing & Debugging Guide

**Table of Contents**
- [Automated Unit Tests](#automated-unit-tests)
- [Dependency Injection Pattern](#dependency-injection-pattern)
- [Manual Verification](#manual-verification)
- [Debugging Guide](#debugging-guide)

---

## Automated Unit Tests

The repository contains a comprehensive suite of automated unit tests using **Google Test** and **Google Mock**. These tests run on the host machine (not the Teensy) to verify logic, safety state machines, and fail-safe behaviors without requiring physical hardware.

### Running Tests
Tests are built and run using PlatformIO's native test environment.

```bash
# Run all tests
cd src/Sigyn/TeensyV2
pio test -e test

# Run specific test suite
pio test -e test -f test_battery_monitor
pio test -e test -f test_roboclaw_monitor
```

### Test Coverage
| Suite | File | Key Scenarios Tested |
|-------|------|----------------------|
| **BatteryMonitor** | `test/battery_monitor_test.cpp` | Critical voltage/current faults, EMA filtering, sensor disconnect recovery, state transitions (NORMAL→CRITICAL). |
| **RoboClawMonitor** | `test/roboclaw_monitor_test.cpp` | Motor Runaway detection, Overcurrent triggers, E-stop latching behavior, Setup failure handling. |
| **SafetyCoordinator** | `test/safety_coordinator_test.cpp` | Fault priority logic, E-stop propagation, Warning vs Critical severity handling. |
| **TemperatureMonitor** | `test/temperature_monitor_test.cpp` | Over-temperature thresholds, Thermal Runaway detection, Sensor disconnect logic. |

---

## Dependency Injection Pattern

The firmware uses **Dependency Injection (DI)** to decouple business logic from hardware drivers. This allows unit tests to inject Mock objects that simulate hardware behavior (including failures that are dangerous or impossible to reproduce physically).

### How it Works

1. **Interfaces**: Hardware drivers implement abstract interfaces (e.g., `IPowerSensor`, `RoboClawInterface`).
2. **Injection**: Modules accept pointers to these interfaces during setup or via specific setters, rather than instantiating hardware drivers directly.
3. **Mocks**: Tests provide `MockPowerSensor` or `MockRoboClaw` to control "sensor" readings precisely.

### Example: BatteryMonitor Injection

**In Firmware (`battery_monitor.h`):**
```cpp
// Module accepts any generic IPowerSensor
void registerSensor(size_t index, IPowerSensor* sensor);
```

**In Test (`battery_monitor_test.cpp`):**
```cpp
// 1. Create a mock sensor
auto mock_sensor = std::make_unique<MockPowerSensor>();

// 2. Inject it into the singleton
BatteryMonitor::getInstance().registerSensor(0, mock_sensor.get());

// 3. Simulate a critical low voltage
mock_sensor->setVoltage(31.0f); 

// 4. Verify system reaction
monitor_->loop();
EXPECT_TRUE(SafetyCoordinator::getInstance().isUnsafe());
```

### Example: RoboClaw Injection

**In Firmware (`roboclaw_monitor.h`):**
```cpp
// Allow swapping the real RoboClaw driver for a mock
void setRoboClawForTesting(RoboClawInterface* roboclaw);
```

---

## Manual Verification

For integration testing on actual hardware:

1. **RoboClawMonitor**
   - Send `cmd_vel` via host; wheels should spin.
   - Disconnect RoboClaw or block a wheel: expect `CRITICAL` fault from SafetyCoordinator citing overcurrent or comm failure.
   
2. **BatteryMonitor**
   - **Warning**: Connect variable power supply, drop voltage to 33.5V → Expect WARNING.
   - **Critical**: Drop voltage to 31.5V → Expect EMERGENCY_STOP.

3. **Combined Safety**
   - Send `setEstopCommand("trigger=true")` via host path.
   - Verify E-stop asserts (motors disable, red LED logic).
   - Verify system recovers only after `reset=true` AND conditions are clear.

---

## Debugging Guide

### Serial Monitor
Connect via USB serial to watch real-time `DIAG` and `FAULT` messages.

```json
DIAG1:{"level":"INFO","module":"SafetyCoordinator","message":"Fault activated: source=BatteryMonitor, severity=CRITICAL"}
FAULT1:{"active_fault":"true","source":"BatteryMonitor","severity":"CRITICAL"}
```

### Retrieving Logs
If SD card logging is enabled:
1. Remove MicroSD card.
2. Read `LOGxxxxx.TXT` files.
3. Filter for errors: `grep '"level":"ERROR"' LOG*.TXT`.

### Common Config Issues
- **Thresholds**: Verify `config.h` matches your hardware (e.g., shunt resistor values, voltage divider ratios).
- **Board ID**: Ensure `BOARD_ID` is set correctly for the target Teensy (Main vs Elevator).
