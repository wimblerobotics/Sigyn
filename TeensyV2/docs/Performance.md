# TeensyV2 Performance Monitoring

TeensyV2 enforces tight real-time constraints. This document explains how performance is measured, reported, and enforced by the core Module system.

## Loop Targets
- Board 1: 85 Hz target
- Board 2: 80 Hz target
- Minimum acceptable frequency is set per-board in `config.h` via `BOARD_MIN_LOOP_FREQUENCY_HZ`

## Module Budget
- Default cap: ≤2.0 ms per module `loop()` (configurable)
- Critical modules may have separate budgets; see `config.h`

## Statistics
Collected in `Module::PerformanceStats`:
- `duration_min_us`, `duration_max_us`, `duration_sum_us`, `loop_count`

Available at runtime via:
- `Module::getPerformanceStats(char* json, size_t size)` (aggregated)
- `module.getPerformanceStats()` (per-module)

## Violation Detection
- Module loop time > `kMaxLoopTimeUs` (2 ms) → timing violation
- Overall loop frequency < `kTargetLoopFrequencyHz` (80 Hz default) → frequency violation
- Board-specific critical thresholds in `config.h` escalate to safety actions

## Reporting
- PERF JSON messages (see README) provide:
  - `freq`, `tfreq`, `mviol`, `fviol`
  - Per-module array with min/max/avg/last

## Tuning Tips
- Split slow operations across cycles using state machines
- Push IO to lower-frequency tiers when possible (e.g., diagnostics at 3 Hz)
- Avoid dynamic allocation in `loop()`
- Use integer math in hot paths; cache trigonometry if needed

## Debug Builds
- Use `env:board*_debug` for instrumentation and `-DTEENSY_V2_DEBUG_LEVEL`
- Monitor with `/path/to/venv/bin/pio device monitor --baud 115200`
