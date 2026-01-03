# Sigyn Safety System TODO

## Overview

This document outlines the architectural faults, missing implementations, and recommended improvements for the Sigyn safety system. It is derived from a deep code analysis of `TeensyV2` and `sigyn_to_sensor_v2`.

## 1. Critical Fixes (Immediate Action Required)

### 1.1 Implement Battery Safety
**Problem**: `BatteryMonitor` monitors voltage and current but `isUnsafe()` returns `false`.
**Fix**:
*   Modify `TeensyV2/modules/battery/battery_monitor.cpp`.
*   Implement `isUnsafe()` to check:
    *   `voltage_ema_ < config_.critical_low_voltage`
    *   `current_ema_ > config_.critical_high_current`
    *   `state_ == BatteryState::CRITICAL`
*   Ensure `resetSafetyFlags()` clears the error only if voltage recovers (with hysteresis).

### 1.2 Enable Safety on Board 3 (Gripper)
**Problem**: `config.h` sets `BOARD_HAS_SAFETY 0` for Board 3. The gripper has no safety monitoring.
**Fix**:
*   In `TeensyV2/common/core/config.h`, set `BOARD_HAS_SAFETY 1` for Board 3.
*   In `TeensyV2/src/elevator_board.cpp`, instantiate `SafetyCoordinator`.
*   Modify `TeensyV2/modules/motors/stepper_motor.cpp` to implement `isUnsafe()`.
    *   Check for stall conditions (if detectable).
    *   Check for limit switch inconsistencies (e.g., both up and down active).
    *   Check for timeout on movements.

### 1.3 Fix Software E-Stop
**Problem**: `SerialManager::handleCommand` has a `TODO` for "ESTOP".
**Fix**:
*   In `TeensyV2/common/core/serial_manager.cpp`:
    *   Parse the `ESTOP` command args (trigger/reset).
    *   Call `SafetyCoordinator::getInstance().activateEstop(...)` or `deactivateEstop()`.
*   Ensure this propagates to the hardware E-stop pin so other boards are notified.

## 2. High Priority Improvements

### 2.1 Map Safety Sources
**Problem**: `SafetyCoordinator` uses `EstopSource::UNKNOWN` for module errors.
**Improvement**:
*   Extend `EstopSource` enum to include `BATTERY`, `MOTOR`, `SENSOR`, `GRIPPER`.
*   Update `checkModuleSafety()` to map the module name to a source, or pass the source from the module.

### 2.2 Inter-Board Communication
**Problem**: `fault_handler` in `board1_main.cpp` has commented out code for inter-board signaling.
**Improvement**:
*   Implement the `digitalWrite` to the inter-board safety pin in `fault_handler`.
*   Ensure `SafetyCoordinator` on all boards monitors this pin and triggers E-stop if it goes HIGH.

### 2.3 ROS 2 Command Propagation
**Problem**: `TeensyBridge` sends E-stop only to Board 1.
**Improvement**:
*   If Board 1 is the "master" for safety, it should assert the hardware line.
*   Verify wiring: Does Board 1's E-stop output pin connect to Board 2 and 3's input pins? If so, the current software approach is fine (once 1.3 is fixed). If not, `TeensyBridge` needs to send the command to all connected boards.

## 3. Medium Priority / Architectural Improvements

### 3.1 Temperature Safety
**Problem**: `TemperatureMonitor` implements `isUnsafe` but enforcement is unclear.
**Improvement**:
*   Audit `TemperatureMonitor` to ensure `critical_high` thresholds are set correctly in config.
*   Test thermal runaway detection logic.

### 3.2 Sensor Timeout Safety
**Problem**: If a sensor (e.g., VL53L0X) stops responding, it might not trigger `isUnsafe`.
**Improvement**:
*   Add `last_reading_time` to all sensor monitors.
*   In `isUnsafe()`, check `millis() - last_reading_time > timeout`.

### 3.3 System-Wide "Ready" State
**Problem**: The robot might start moving before all boards are fully initialized and safe.
**Improvement**:
*   Implement a "System Ready" handshake.
*   `sigyn_to_sensor_v2` should wait for "STATUS:OK" from all 3 boards before allowing `cmd_vel` to pass through.

## 4. Code Consistency

*   **Standardize Logging**: Some modules use `Serial.println`, others use `SerialManager`. Move all logging to `SerialManager` for consistent JSON formatting.
*   **Config Validation**: Add `static_assert` or runtime checks in `setup()` to ensure critical config values (like battery thresholds) are sane.

## 5. Future Considerations

*   **Watchdog Timer**: Enable the hardware Watchdog Timer (WDT) on the Teensy. If the code hangs (infinite loop), the WDT should reset the board.
*   **Black Box Logging**: Use the SD card to log the last 60 seconds of state in a circular buffer, saved only on E-stop or fault.
