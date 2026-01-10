# TeensyV2 Safety System

This document describes the safety architecture of the TeensyV2 firmware (modules in `modules/safety`, `modules/roboclaw`, `modules/battery`, `modules/performance`, `modules/sensors`).

## Core Concept

- Safety is implemented by the `SafetyCoordinator` module (`modules/safety/safety_coordinator.*`).
- Modules raise or clear faults via `activateFault(FaultSeverity, source, description)` and `deactivateFault(source)`.
- Up to 16 faults are tracked by source name; each fault carries a severity (`NORMAL`, `WARNING`, `DEGRADED`, `EMERGENCY_STOP`, `SYSTEM_SHUTDOWN`).
- E-stop behavior: faults raised at `EMERGENCY_STOP`/`SYSTEM_SHUTDOWN` increment an internal counter. When the counter rises from 0→1 the RoboClaw E-stop line is asserted (only if `CONTROLS_ROBOCLAW_ESTOP_PIN` is compiled in). Clearing the last E-stop fault releases the line.
- Reporting: once per second the coordinator publishes `FAULT` JSON messages via `SerialManager` for each active fault (or a single `{\"active_fault\":\"false\"}` when clear).
- Commands: `setEstopCommand("trigger=true")` raises an E-stop fault; `setEstopCommand("reset=true")` calls `Module::resetAllSafetyFlags()`.
- Recovery: the coordinator does not auto-clear faults; callers must clear them explicitly.

## Fault Sources

- **RoboClawMonitor** (`modules/roboclaw/roboclaw_monitor.*`)
  - Motor overcurrent: > `max_current_m1/m2` (defaults 100 A). Latched; manual reset required.
  - Runaway detection: speed exceeds `runaway_speed_threshold_qpps` (always active).
  - RoboClaw fatal error bits: any of the `kFatalRoboclawErrorMask` bits set.
  - RoboClaw over-temperature: `temperature_c >= roboclaw_temp_fault_c` (default 100°C).
  - Communication failures: repeated read failures mark communication not OK and raise a fault.
  - Commands older than `cmd_vel_timeout_ms` (200 ms default) zero motor commands.

- **BatteryMonitor** (`modules/battery/battery_monitor.*`)
  - Critical low voltage: < `critical_low_voltage` (default 32.0 V) → `EMERGENCY_STOP`.
  - Warning low voltage: < `warning_low_voltage` (default 34.0 V) → `WARNING`.
  - Critical high current: > `critical_high_current` (default 20.0 A) → `EMERGENCY_STOP`.
  - Each battery index reports separately (fault source is `BatteryMonitor_BatteryN`).

- **TemperatureMonitor** (`modules/sensors/temperature_monitor.*`)
  - Thermal Runaway: Temperature rising faster than `thermal_runaway_rate` (default 100°C/min) → `EMERGENCY_STOP`.
  - Per-sensor warning/critical defaults: 70°C / 85°C.
  - System warning/critical defaults: 65°C / 80°C.
  - Sensor stale/timeout: no valid readings for > `fault_timeout_ms` (default 5s) → `DEGRADED`.
  - Raises faults through `SafetyCoordinator` when thresholds exceeded.

- **PerformanceMonitor** (`modules/performance/performance_monitor.*`)
  - Tracks loop frequency and module execution times against config (`BOARD_MIN_LOOP_FREQUENCY_HZ`, `BOARD_MAX_MODULE_TIME_MS`).
  - Currently `isUnsafe()` returns `false`; it produces diagnostics only.

- **Software commands**
  - `setEstopCommand("trigger=true")` from host software explicitly asserts an E-stop fault.

## Timing & Threshold References

- Board-specific minimum loop frequency: from `config.h` (`BOARD_MIN_LOOP_FREQUENCY_HZ`; defaults: Board1 50 Hz, Board2 20 Hz, Board3 10 Hz).
- Max module execution time: from `config.h` (`BOARD_MAX_MODULE_TIME_MS`; defaults: Board1 2 ms, Board2 3 ms, Board3 5 ms).
- RoboClaw communication timeout: `timeout_us = 100000` (100 ms) per `RoboClawConfig`.
- RoboClaw status reporting cadences:
  - Critical encoder/odometry: ~67 Hz (15 ms cadence)
  - Safety checks: 10 Hz
  - System status: ~3 Hz
  - Diagnostics: ~1 Hz

## Recovery Expectations

- Overcurrent and RoboClaw fatal errors: latched; require host reset or power intervention.
- Battery critical/high current: clears when voltage/current return within thresholds; charging is external.
- Temperature: clears after cooling below configured thresholds.
- Runaway: clears when speeds drop below threshold and faults are deactivated by the module.
- Software E-stop: clears only after `reset=true` command or `resetAllSafetyFlags()`.

## ROS2 / Host Visibility

- Faults are emitted as `FAULT` JSON messages by `SerialManager` once per second when active (or clear message when inactive).
- Other diagnostics are emitted by each module (`DIAG1` style messages) and can be logged via `SDLogger`.

## Notes

- Faults are stored individually by source string; there is no global error bitmask.
- Inter-board GPIO safety signaling is not implemented in `SafetyCoordinator`; only the RoboClaw E-stop line is driven (when compiled with control pin enabled).
- Verify thresholds in `config.h` and per-module configs before relying on any specific numbers in this document.
