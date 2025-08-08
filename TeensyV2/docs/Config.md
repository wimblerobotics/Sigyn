# TeensyV2 Configuration Guide (config.h)

This guide explains how board- and feature-configuration is organized in TeensyV2 and how to add new boards safely.

## Overview

Configuration is centralized in `common/core/config.h` and driven by the compile-time define `BOARD_ID` (set per PlatformIO environment). Each board declares which features it has using `BOARD_HAS_*` flags. Convenience macros `ENABLE_*` are derived from those to keep conditionals readable across the codebase.

Key goals:
- Single source of truth for all board capabilities
- No scattered `#ifdef`s in modules
- Safe defaults and compile-time validation

## Key Macros

- `BOARD_ID` (required): integer ID selecting board config block
- Capability flags (per board):
  - `BOARD_HAS_SD_LOGGING`
  - `BOARD_HAS_MOTOR_CONTROL`
  - `BOARD_HAS_VL53L0X`
  - `BOARD_HAS_TEMPERATURE`
  - `BOARD_HAS_PERFORMANCE`
  - `BOARD_HAS_SAFETY`
  - `BOARD_HAS_ROBOCLAW`
  - `BOARD_HAS_BATTERY`
  - `BOARD_HAS_IMU`
- Convenience macros (global):
  - `ENABLE_SD_LOGGING`, `ENABLE_MOTOR_CONTROL`, `ENABLE_VL53L0X`, `ENABLE_TEMPERATURE`, `ENABLE_PERFORMANCE`, `ENABLE_SAFETY`, `ENABLE_ROBOCLAW`, `ENABLE_BATTERY`, `ENABLE_IMU`

## Board Parameters

Each `BOARD_ID` also defines:
- Serial config: `BOARD_SERIAL_BAUD_RATE`, `BOARD_SERIAL_TIMEOUT_MS`, `BOARD_SERIAL_WAIT_MS`
- Safety pins: `INTER_BOARD_SIGNAL_{INPUT,OUTPUT}_PIN`, `HARDWARE_ESTOP_INPUT_PIN`, `ESTOP_OUTPUT_PIN`
- Performance thresholds: `BOARD_MAX_MODULE_TIME_MS`, `BOARD_MIN_LOOP_FREQUENCY_HZ`, `BOARD_CRITICAL_*`
- Module-specific polling intervals (e.g., `SAFETY_CHECK_INTERVAL_US`, `VL53L0X_CHECK_INTERVAL_US`)

## How Features Are Consumed

Modules should use `#if ENABLE_<FEATURE>` guards to include/compile code paths for features that exist on the target board. Avoid probing hardware in code paths that are known to be disabled.

Example:
```cpp
#if ENABLE_BATTERY
  BatteryMonitor::getInstance();
#endif
```

## Adding a New Board

1) Create a PlatformIO environment in `platformio.ini`:
- Set `-DBOARD_ID=<N>` and a distinct `build_src_filter` selecting the proper `boardN_main.cpp` (or reuse an existing main) and needed modules

2) Extend `config.h` with a new `#elif BOARD_ID == <N>` block:
- Define all `BOARD_HAS_*` flags
- Fill in serial, safety GPIOs, and performance thresholds

3) Select modules in the env’s `build_src_filter`:
- Only include modules that the board actually has to keep code size and init cost low

4) Validate:
- Build both release and debug variants
- Power-on test: verify pins and safety paths
- Run with serial monitor and confirm PERF and safety messages

## Best Practices

- Keep `config.h` compile-time only; do not include runtime state or Arduino calls
- Prefer `ENABLE_*` over `BOARD_HAS_*` in module code
- Use descriptive `BOARD_NAME` in `platformio.ini` for diagnostics
- Document board expectations in `docs/Architecture.md` or a board-specific note

## Troubleshooting

- “Unknown BOARD_ID” error: Add the board block in `config.h` and set `-DBOARD_ID` in `platformio.ini`
- Wrong pins/levels at runtime: confirm the env used for upload matches the hardware; check `upload_port` mapping
- Unexpected module compiled in/out: inspect `build_src_filter` and `ENABLE_*` guards
