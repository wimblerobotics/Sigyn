# TeensyV2 Unit Testing Infrastructure

## Overview
This directory contains the unit testing infrastructure for the TeensyV2 embedded firmware. The testing system allows us to compile and test TeensyV2 code on a PC using Google Test, without requiring actual Teensy hardware.

## Architecture

### Dual Build System
The TeensyV2 codebase supports two build systems:

1. **PlatformIO** (Production): Builds firmware for Teensy 4.1 hardware
   - Uses real Arduino SDK and Teensy libraries
   - Compiles to ARM Cortex-M7 binary
   - Uploads via USB to physical boards

2. **CMake + ROS2** (Testing): Builds unit tests for PC execution
   - Uses mocked Arduino API
   - Compiles to x86_64 native code
   - Runs with Google Test framework

### Conditional Compilation
Source files use preprocessor directives to switch between real and mocked hardware:

```cpp
#ifdef UNIT_TEST
#include "Arduino.h"  // Mock from test/mocks/
#else
#include <Arduino.h>  // Real Teensy SDK
#endif
```

When `UNIT_TEST` is defined (automatically by CMakeLists.txt), the mock Arduino API is used instead of the real hardware.

## Directory Structure

```
TeensyV2/
├── platformio.ini          # PlatformIO config (production builds)
├── CMakeLists.txt          # CMake config (PC test builds)
├── package.xml             # ROS2 package manifest (test dependencies)
├── src/                    # Board-specific entry points (board1_main.cpp, etc.)
├── common/                 # Core infrastructure (Module, SerialManager)
├── modules/                # Feature modules (sensors, safety, motors, etc.)
└── test/                   # Unit testing infrastructure
    ├── mocks/              # Mock hardware APIs
    │   ├── Arduino.h       # Mock Arduino API
    │   ├── Arduino.cpp     # Mock Arduino implementation
    │   └── ...             # Future: Wire, Serial, etc.
    ├── temperature_monitor_test.cpp
    ├── safety_coordinator_test.cpp
    └── ...
```

## Mock Arduino API

The mock Arduino implementation (`test/mocks/Arduino.h`) provides:

- **Time Control**: `millis()`, `micros()`, `delay()` with injectable values
- **Digital I/O**: `pinMode()`, `digitalWrite()`, `digitalRead()` with state tracking
- **Analog I/O**: `analogRead()`, `analogWrite()` with injectable values
- **String Class**: Minimal Arduino String using std::string internally
- **Serial/Wire**: Stub implementations (not yet functional)

### Test Control Functions
Tests can control the mock hardware state:

```cpp
arduino_mock::setMillis(1000);       // Set current time to 1000ms
arduino_mock::setAnalogValue(A0, 2048);  // Set pin A0 to 2048 (12-bit ADC)
arduino_mock::reset();                // Reset all mock state
```

## Writing Tests

### Basic Test Structure

```cpp
#include <gtest/gtest.h>
#include "Arduino.h"  // Mock Arduino
#include "modules/sensors/temperature_monitor.h"

class MyModuleTest : public ::testing::Test {
protected:
  void SetUp() override {
    arduino_mock::reset();  // Clean slate for each test
  }
  
  void TearDown() override {
    arduino_mock::reset();
  }
};

TEST_F(MyModuleTest, TestSomething) {
  // Arrange: Set up mock hardware state
  arduino_mock::setMillis(0);
  arduino_mock::setAnalogValue(A0, 2048);
  
  // Act: Call code under test
  // ... 
  
  // Assert: Verify behavior
  EXPECT_EQ(result, expected);
}
```

### Current Test Coverage

**Phase 1** (Current):
- Temperature trend calculation logic
- Arduino mock infrastructure validation

**Phase 2** (Planned):
- Temperature monitor with injected sensor values
- Safety coordinator fault detection

## Building and Running Tests

### Prerequisites
```bash
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2
rosdep install --from-paths . --ignore-src -y
```

### Build Tests
```bash
cd /home/ros/sigyn_ws
colcon build --packages-select teensy_v2 --cmake-args -DBUILD_TESTING=ON
```

### Run All Tests
```bash
colcon test --packages-select teensy_v2
colcon test-result --verbose
```

### Run Specific Test
```bash
./install/teensy_v2/lib/teensy_v2/test_temperature_monitor
```

## Design Constraints

### No Heap Allocation in Production Code
The TeensyV2 firmware **must not use heap allocation** (no `new`, `malloc`, `std::vector`, etc.) in code that runs on the Teensy. This is a hard safety requirement for real-time embedded systems.

✅ **Allowed**:
- Static allocation
- Stack allocation
- Fixed-size arrays
- Singleton pattern with static storage

❌ **Forbidden**:
- `new` / `delete`
- `malloc` / `free`
- `std::vector`, `std::map`, `std::string` (in production code)
- Arduino `String` (uses heap internally, avoid in production)

**Exception**: Test code can use heap freely. The mock infrastructure uses `std::map` and `std::string` because it only runs on PC.

## Dependency Injection Strategy

For Phase 2+, we will add interfaces for hardware components to enable dependency injection:

```cpp
// Interface (pure virtual)
class ITemperatureSensor {
public:
  virtual ~ITemperatureSensor() = default;
  virtual float readTemperature() = 0;
};

// Real hardware implementation
class TMP36Sensor : public ITemperatureSensor {
  float readTemperature() override {
    return convertAnalogToTemperature(analogRead(pin_));
  }
};

// Mock implementation for testing
class MockTemperatureSensor : public ITemperatureSensor {
  float readTemperature() override {
    return test_temperature_;  // Injected by test
  }
  void setTemperature(float temp) {
    test_temperature_ = temp;
  }
};
```

This pattern will be applied to:
- Temperature sensors (TMP36)
- Battery monitors (INA226, analog voltage)
- IMU sensors (BNO055)
- Motor controller (RoboClaw)
- Distance sensors (VL53L0X)

## Verifying PlatformIO Still Works

After adding `#ifdef UNIT_TEST` directives, always verify the production build:

```bash
cd /home/ros/sigyn_ws/src/Sigyn/TeensyV2
pio run -e board1
```

The PlatformIO build **must not** define `UNIT_TEST`, so it will use the real Arduino SDK.

## FAQ

**Q: Why not use PlatformIO's native unit testing?**
A: PlatformIO's testing requires either uploading tests to hardware or using their limited desktop testing. ROS2+CMake+GTest provides better integration with our workspace and more powerful mocking.

**Q: Can I run tests in a debugger?**
A: Yes! Since tests compile to native x86_64 code, you can use GDB or any standard debugger:
```bash
gdb ./install/teensy_v2/lib/teensy_v2/test_temperature_monitor
```

**Q: How do I add a new test?**
A:
1. Create `test/my_module_test.cpp`
2. Add to `CMakeLists.txt`:
   ```cmake
   ament_add_gtest(test_my_module test/my_module_test.cpp)
   target_link_libraries(test_my_module ${PROJECT_NAME}_core)
   ```
3. Rebuild and run tests

**Q: What about performance/timing tests?**
A: The mock Arduino lets you control time precisely. You can test timing logic without waiting:
```cpp
arduino_mock::setMillis(0);
module.loop();  // First iteration
arduino_mock::setMillis(100);
module.loop();  // 100ms later
```

## Next Steps

**Phase 2**: Add hardware interfaces and mock implementations
- Refactor TemperatureMonitor to use ITemperatureSensor
- Create comprehensive temperature monitor tests
- Add safety coordinator test hooks

**Phase 3**: Safety coordinator tests
- Fault injection API
- Multi-fault scenarios
- Fault clearing and recovery

**Phase 4**: RoboClaw and motor safety
- Mock RoboClaw hardware layer
- Test motor runaway detection
- Test overcurrent protection
- Test odometry calculation

## References
- [Google Test Documentation](https://google.github.io/googletest/)
- [ROS2 Testing Guide](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Testing/Testing-Main.html)
- [Teensy 4.1 Reference](https://www.pjrc.com/teensy/teensy41.html)
