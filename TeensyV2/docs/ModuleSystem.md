# TeensyV2 Module System

This document explains the core module framework found in `common/core/module.h/.cpp`.

## Purpose
- Provide automatic registration and lifecycle management of all functional blocks (sensors, actuators, services)
- Enforce real-time performance budgeting and safety checks for each module

## Lifecycle
- `Module::setupAll()` runs once at startup in registration order
- `Module::loopAll()` runs each cycle and measures each module’s execution time

## Key APIs (for module authors)
- Derive from `sigyn_teensy::Module`
- Implement:
  - `void setup()` — blocking init OK
  - `void loop()` — non-blocking, ≤2 ms target
  - `const char* name() const`
  - Optional: `bool isUnsafe()` and `void resetSafetyFlags()`
- Provide a singleton accessor that constructs the module (auto-registers)

## Performance
- Default per-module budget: 2.0 ms (see `kMaxLoopTimeUs`)
- Target loop rate: 80–100 Hz (see `kTargetLoopFrequencyHz`)
- Stats available per-module and aggregated as JSON via `getPerformanceStats(...)`

## Safety Integration
- Any module can report unsafe state via `isUnsafe()`
- Safety coordinator can query `Module::isAnyModuleUnsafe()` and drive E-stop

## Best Practices
- Split heavy work over multiple cycles using state machines
- Avoid dynamic allocation in `loop()`
- Prefer integer math in hot paths
- Guard feature-specific code with `#if ENABLE_*` from `config.h`
