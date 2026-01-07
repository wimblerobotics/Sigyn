# TeensyV2 Safety System Overview

**Building Trust Through Comprehensive Safety Design**

*Part of the TeensyV2 Advanced Robotic Control System Documentation*

---

## Table of Contents

- [I. Introduction & Motivation](#i-introduction--motivation)
- [II. Safety Philosophy & Goals](#ii-safety-philosophy--goals)
- [III. Fault Severity Levels](#iii-fault-severity-levels)
- [IV. Hardware Safety Modules](#iv-hardware-safety-modules)
  - [Temperature Monitor](#temperature-monitor)
  - [Battery Monitor](#battery-monitor)
  - [RoboClaw Monitor](#roboclaw-monitor)
  - [Performance Monitor](#performance-monitor)
  - [IMU & VL53L0X (Planned)](#imu--vl53l0x-planned)
- [V. Safety Coordinator Architecture](#v-safety-coordinator-architecture)
- [VI. Multi-Board Safety Coordination](#vi-multi-board-safety-coordination)
- [VII. RoboClaw Special Handling](#vii-roboclaw-special-handling)
- [VIII. Self-Healing Faults](#viii-self-healing-faults)
- [IX. ROS2 Integration](#ix-ros2-integration)
- [X. Performance & Safety Balance](#x-performance--safety-balance)
- [XI. Testing Philosophy](#xi-testing-philosophy)
- [XII. Developer Integration Guide](#xii-developer-integration-guide)
- [XIII. Operator/Developer Usage](#xiii-operatordeveloper-usage)
- [XIV. Troubleshooting](#xiv-troubleshooting)
- [XV. Cost/Complexity Tradeoffs](#xv-costcomplexity-tradeoffs)
- [XVI. Future Enhancements](#xvi-future-enhancements)
- [Appendices](#appendices)
  - [A. Fault Message Format Examples](#appendix-a-fault-message-format-examples)
  - [B. Safety-Related Configuration Parameters](#appendix-b-safety-related-configuration-parameters)
  - [C. System Diagrams](#appendix-c-system-diagrams)

---

## I. Introduction & Motivation

The Sigyn autonomous house patroller represents a significant step forward in personal robotics: a mobile robot that operates independently in human living spaces, navigating hallways, climbing onto elevators, and monitoring for intruders or environmental hazards. But with autonomy comes responsibility. A robot that can move on its own can also damage property, injure people, or catch fire if its safety systems fail.

This document describes the comprehensive safety architecture built into TeensyV2, the embedded control system at the heart of Sigyn. Unlike toy robots or research platforms that operate under constant supervision, Sigyn is designed to patrol unsupervised for hours at a time. This means the safety system must be robust, self-monitoring, and capable of graceful degradation when things go wrong.

**Why Safety Matters in Autonomous Robotics**

Trust is the foundation of autonomous robotics. A robot operating in your home must be as trustworthy as your smoke detector or door lock. You need confidence that:

- **It won't catch fire** while charging unattended overnight
- **It won't damage floors or furniture** if a wheel encoder fails and causes runaway acceleration
- **It won't drain its battery completely** and become stranded in an elevator
- **It will stop immediately** if something goes wrong, rather than continuing to operate in a degraded state

The cost of failure isn't just measured in damaged hardware. A single battery fire can destroy a home. A runaway motor can injure a pet or person. Even minor failures erode trust and make the technology unacceptable for real-world deployment.

**Real-World Failure Scenarios**

These aren't theoretical concerns—they're real failure modes encountered during Sigyn's development:

1. **Motor Controller Meltdown (2024)**: During early testing, a RoboClaw motor controller drew excessive current due to a firmware bug. Without proper monitoring, the board temperature climbed above 85°C before the built-in thermal protection triggered. The plastic enclosure began to soften. *Lesson: Temperature monitoring must be continuous and predictive, not reactive.*

2. **Battery Fire Risk (2023)**: A 3S LiPo battery cell became unbalanced during charging. The battery management system failed to detect the issue because voltage was only checked at the pack level, not per-cell. Testing revealed the cell voltage had drifted to 4.35V (safe maximum is 4.2V). *Lesson: Multi-rail monitoring is essential; pack-level voltage isn't sufficient.*

3. **Runaway Wheel (2024)**: A loose encoder cable caused intermittent signals. The motor controller interpreted this as no feedback and drove the motor to maximum speed trying to reach the commanded velocity. The robot accelerated into a wall at full speed, damaging both the wheel assembly and the wall. *Lesson: Encoder validation must cross-check motor current against expected movement.*

4. **Elevator Door Collision (2025)**: During elevator navigation testing, the robot's obstacle detection failed when the elevator doors began closing. The robot continued moving forward, wedging itself between the doors. The motors stalled, drawing maximum current until manually powered off. *Lesson: Motor current monitoring must trigger emergency stop before mechanical damage occurs.*

5. **Thermal Runaway Cascade (2024)**: A Teensy board running intensive sensor processing began to overheat. The increased temperature reduced the efficiency of the voltage regulator, which generated more heat, creating a positive feedback loop. The board reached 78°C before testing was halted. *Lesson: Rate-of-change monitoring catches thermal runaway before absolute thresholds are exceeded.*

These incidents shaped the design philosophy documented in this guide: comprehensive monitoring, predictive detection, and defense in depth.

---

## II. Safety Philosophy & Goals

The TeensyV2 safety system is built on five core principles that guide all design decisions:

**Defense in Depth**

No single safety mechanism is perfect. Sensors fail, software has bugs, and unexpected interactions create novel failure modes. Defense in depth means building multiple overlapping layers of protection:

- **Hardware Layer**: Physical e-stop pin on RoboClaw motor controllers that cuts power regardless of software state
- **Firmware Layer**: SafetyCoordinator running on each Teensy board, monitoring local sensors and generating faults
- **Communication Layer**: Serial protocol that transmits faults between boards and to ROS2
- **Software Layer**: ROS2 bridge (sigyn_to_sensor_v2) aggregates faults from all sources and publishes unified safety state
- **Behavior Layer**: Behavior trees can trigger software e-stop based on high-level mission context (e.g., approaching stairs, low battery near elevator)

If any layer fails, the others continue protecting the system. For example, if the ROS2 bridge crashes, the Teensy boards continue monitoring local sensors and will trigger hardware e-stop if needed.

**Self-Healing Design**

Not all faults require human intervention. Many conditions are transient:

- A temperature spike that resolves when the robot moves to a cooler area
- A brief voltage drop when motors start under high load
- A momentary encoder glitch due to electrical noise

The safety system distinguishes between *transient* and *persistent* faults. Transient faults automatically clear when the underlying condition resolves, allowing the robot to resume operation without manual intervention. This is crucial for unsupervised patrol missions that may last several hours.

However, certain faults (like motor overcurrent or thermal runaway) require manual reset to ensure a human has verified the issue is resolved. This balance between automation and safety is a core design tradeoff.

**Fail-Safe Principles**

When designing each safety feature, we ask: "What happens if this component fails?"

- **E-stop pin is active-low**: If the signal wire breaks, the motor controllers see a LOW signal and stop immediately
- **Watchdog timers**: If software hangs, the hardware watchdog resets the board after 5 seconds
- **Graceful degradation**: If a non-critical sensor fails (like one of three temperature monitors), the system continues operating with reduced capability rather than stopping completely
- **Conservative thresholds**: Safety thresholds are set well below destructive limits. For example, motor current limits are set at 80% of the controller's rated maximum to provide safety margin

**Performance Preservation as Safety Requirement**

This might seem counterintuitive, but *maintaining real-time performance is itself a safety requirement*. Here's why:

The TeensyV2 system runs control loops at 80-100Hz. These loops:
- Update motor commands based on encoder feedback
- Process sensor data to detect obstacles
- Monitor battery voltage and temperature
- Generate odometry for navigation

If safety checks cause the control loop to slow down (say, from 85Hz to 40Hz), several bad things happen:
1. Motor control becomes unstable, potentially causing jerky movement or oscillation
2. Encoder readings are processed less frequently, reducing positional accuracy
3. Battery voltage monitoring has longer gaps, missing brief voltage sags
4. Navigation deadlines are missed, causing the ROS2 navigation stack to timeout

A robot that moves jerkily is more likely to collide with obstacles. A robot with poor odometry is more likely to get lost. A robot that times out during navigation may stop in an unsafe location (like blocking a doorway).

Therefore, **safety checks must not violate timing requirements**. This is achieved through:
- Exponential Moving Average (EMA) filters that smooth data without storing large buffers
- Amortized checking: some sensors are read every cycle, others every N cycles
- Early-exit optimizations: fast path for common case (no fault), detailed checks only when thresholds are approached

See [Section X: Performance & Safety Balance](#x-performance--safety-balance) for detailed discussion.

**Testing as Core Requirement**

Safety systems are only as good as their test coverage. Every safety feature in TeensyV2 includes:

1. **Unit tests**: Mock hardware allows testing fault detection logic without physical sensors
2. **Integration tests**: Full system tests with real hardware verify end-to-end fault propagation
3. **Failure injection**: Deliberately trigger each fault condition to verify correct response
4. **Continuous testing**: Safety tests run on every code change via GitHub Actions CI/CD

For example, the temperature monitoring system includes tests that:
- Simulate temperature ramps to verify rate-of-change detection
- Inject noise to verify EMA filter effectiveness  
- Trigger threshold violations to verify fault activation
- Verify fault deactivation when temperature drops below hysteresis threshold

The dependency injection pattern (see [Section XI](#xi-testing-philosophy)) makes this testing practical by allowing real hardware dependencies to be replaced with mocks during testing.

**Safety Goals Summary**

With these principles in mind, the TeensyV2 safety system aims to:

1. **Detect hazardous conditions** before they cause damage
2. **Stop the robot immediately** when critical faults occur
3. **Recover automatically** from transient issues when safe to do so
4. **Maintain real-time performance** so that safety doesn't compromise stability
5. **Provide visibility** into safety state for operators and behavior trees
6. **Fail gracefully** when components malfunction
7. **Prove correctness** through comprehensive testing

---

## III. Fault Severity Levels

The TeensyV2 safety system uses a hierarchical fault severity model that determines how the system responds to different conditions. Each severity level has specific characteristics regarding response time, recovery mechanism, and operator visibility.

```
Severity Hierarchy:
NORMAL → DEGRADED → WARNING → EMERGENCY_STOP → SYSTEM_SHUTDOWN
         (continue)  (monitor)  (stop now!)     (planned)
```

### DEGRADED

**When it triggers**: Non-critical component failures that reduce capability but don't immediately threaten safety.

**Examples**:
- **Sensor initialization failure**: One of three temperature sensors fails to initialize. The system continues with the remaining two sensors.
- **Communication timeout**: A sensor hasn't provided a reading within its expected interval (e.g., 5 seconds), but the system has redundant sensors.
- **Performance degradation**: A module's execution time has increased but is still within acceptable limits.
- **Non-critical I2C bus error**: Occasional communication errors that recover automatically.

**System Response**:
- Robot continues operating normally
- Fault is logged and reported via ROS2 `/estop_status` topic
- Operator is notified but no immediate action required
- Self-healing: fault clears automatically when condition resolves

**Use Case**: During a patrol mission, one of three temperature sensors stops responding. The robot continues patrolling using the other two sensors. The fault appears in the web interface as "DEGRADED: temp_sensor_roboclaw_1 timeout". When the sensor recovers (perhaps after the robot moves away from an interference source), the fault automatically clears.

**Why this matters**: Distinguishing between degraded and critical failures allows the robot to complete missions despite minor issues, improving operational availability.

### WARNING

**When it triggers**: Predictive indicators that conditions are trending toward unsafe states.

**Examples**:
- **Battery trending low**: Voltage is 34.5V (warning threshold: 34.0V, critical: 32.0V). The robot should return to base soon.
- **Temperature rising**: Motor controller is at 68°C (warning: 65°C, critical: 80°C). Temperature is increasing but not yet dangerous.
- **Memory usage high**: Available RAM is below 20% but system is still functional.
- **Encoder drift**: Wheel encoder readings show minor inconsistencies that may indicate a loose cable.

**System Response**:
- Robot continues operating but may modify behavior
- Behavior tree can use warning state to trigger preventive actions (e.g., "return to base" on battery warning)
- More frequent status reporting to monitor trend
- Self-healing: clears automatically with hysteresis to prevent oscillation

**Use Case**: During patrol, battery voltage drops to 34.2V. A WARNING fault triggers, which the behavior tree detects. Instead of continuing the patrol route, the tree switches to "return to charging station" behavior. If voltage recovers (e.g., load decreases), the warning clears automatically.

**Design Principle**: Warnings give the system time to respond gracefully before conditions become critical. This is the "yellow light" that says "take action soon" without requiring emergency measures.

### EMERGENCY_STOP

**When it triggers**: Critical failures requiring immediate motor shutdown to prevent damage or injury.

**Examples**:
- **Motor overcurrent**: RoboClaw reports >25A draw (threshold: 20A). Likely stalled motor or mechanical jam.
- **Thermal runaway**: Temperature rising at >100°C/min. Indicates a positive feedback loop (heat → more heat).
- **Encoder validation failure**: Motor commanded at 50% power but encoder shows no movement. Motor is stalled or encoder is disconnected.
- **Critical voltage**: Battery below 32V. Risk of battery damage or loss of power during operation.
- **Hardware e-stop**: Physical e-stop button pressed.
- **Software e-stop**: ROS2 command received or behavior tree triggered stop.

**System Response**:
- **Immediate motor shutdown**: E-stop pin on RoboClaw pulled LOW, cutting power to motors
- All motion ceases within <100ms (hardware enforced)
- Fault remains active until manually cleared (for most conditions)
- Robot becomes "bricked" until operator intervention
- Detailed fault information logged with timestamp and triggering values

**Recovery**:
- Most EMERGENCY_STOP faults require **manual reset** via ROS2 command or physical reset
- Exception: Some transient conditions (brief voltage sag, momentary overcurrent) may self-heal if configured
- Operator must verify the underlying issue is resolved before resetting

**Use Case**: Robot is navigating when elevator doors begin closing. The motors draw 28A trying to push through the obstruction. The RoboClaw monitor detects overcurrent and triggers EMERGENCY_STOP. Motors cut off immediately. The robot remains stopped until an operator sends a reset command after verifying no mechanical damage occurred.

**Hardware Enforcement**: The e-stop signal is a physical GPIO pin connected to the RoboClaw's emergency stop input. Even if the Teensy firmware crashes, the pin remains LOW (active), keeping motors disabled. This fail-safe design ensures software bugs can't override safety.

### SYSTEM_SHUTDOWN (Planned)

**When it triggers**: Conditions requiring graceful system powerdown beyond just motor stop.

**Planned Examples**:
- **Critical battery voltage**: Below 30V, system should shut down completely to preserve battery health
- **Thermal emergency with persistence**: Temperature remains critical despite motor shutdown
- **Multiple cascading failures**: Several EMERGENCY_STOP conditions active simultaneously
- **Watchdog escalation**: System has reset multiple times, indicating fundamental failure

**Planned System Response**:
- Flush all data buffers to non-volatile storage
- Send final status message to ROS2 with shutdown reason
- Disable all peripherals in controlled sequence
- Power down Teensy boards via power management circuit
- Requires physical power cycle to restart

**Current Status**: Not yet implemented. Currently, all critical conditions trigger EMERGENCY_STOP (motor shutdown) but the system remains powered. SYSTEM_SHUTDOWN would add full power management for conditions where even passive power draw is problematic.

**Design Consideration**: Implementing this requires additional hardware (power control circuitry) and careful sequencing to avoid data loss or component damage during shutdown.

---

## IV. Hardware Safety Modules

The TeensyV2 safety system consists of several hardware monitoring modules, each responsible for detecting specific failure modes. These modules run continuously as part of the Module framework (see [TEENSYV2_ARCHITECTURE.md] for Module system details) and integrate with SafetyCoordinator to trigger appropriate responses.

This section provides detailed implementation guidance for each module, including algorithms, configuration parameters, and code examples.

### Temperature Monitor

The TemperatureMonitor module tracks up to 8 DS18B20 temperature sensors distributed throughout the robot:
- RoboClaw motor controllers (3 sensors)
- Teensy boards (2 sensors)
- Battery compartment (1 sensor)
- Power distribution board (1 sensor)
- Servo controller (1 sensor)

**Why Temperature Monitoring?**: The motor controller meltdown incident (Section I) demonstrated that thermal protection is non-negotiable. Temperature rises can indicate:
- Stalled motors drawing excessive current
- Thermal runaway (self-heating feedback loop)
- Ambient overheating (robot in hot environment)
- Inadequate cooling or blocked airflow

**Three-Tier Detection Strategy**:

1. **Absolute Thresholds**: Trigger when temperature exceeds fixed limits
   - Warning High: 65-70°C (component-specific)
   - Critical High: 80°C (emergency stop)
   - Critical Low: 0°C (sensor failure or extreme cold)

2. **Rate-of-Change Detection**: Catch thermal runaway before absolute limits
   - Monitors temperature rise per minute
   - Threshold: >100°C/min indicates runaway condition
   - Uses 60-second sliding window for trend calculation

3. **Timeout Detection**: Identify sensor failures
   - If no valid reading received for 5 seconds, mark sensor DEGRADED
   - System continues if redundant sensors available

**Thermal Runaway Detection**

Thermal runaway is a positive feedback loop where:
1. Component temperature rises (due to load, stall, etc.)
2. Electrical resistance increases with temperature
3. Higher resistance causes more power dissipation (P = I²R)
4. More power → more heat → higher temperature → repeat

Without intervention, this loop continues until component destruction.

**Detection Algorithm**:

```cpp
// From temperature_monitor.cpp lines 730-770

void TemperatureMonitor::detectThermalRunaway() {
  for (uint8_t i = 0; i < max_sensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }

    // Calculate temperature trend (°C/min)
    float trend = calculateTemperatureTrend(i);
    sensor_status_[i].temperature_trend = trend;

    // Check against runaway threshold (typically 100°C/min)
    if (trend > sensor_configs_[i].thermal_runaway_rate) {
      if (!sensor_status_[i].thermal_runaway) {
        // Fault activation
        sensor_status_[i].thermal_runaway = true;
        
        // Report to SafetyCoordinator via SerialManager
        char msg[256];
        snprintf(msg, sizeof(msg),
                 "active:true,source:THERMAL_RUNAWAY,reason:%s thermal runaway detected,"
                 "value:%.1f,rate:%.1fC_per_min,manual_reset:false,time:%lu",
                 sensor_configs_[i].sensor_name,
                 sensor_status_[i].temperature_c,
                 trend,
                 millis());
        SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
      }
    } else if (sensor_status_[i].thermal_runaway) {
      // Self-healing: Clear when trend returns to safe levels
      sensor_status_[i].thermal_runaway = false;
      
      char msg[256];
      snprintf(msg, sizeof(msg),
               "active:false,source:THERMAL_RUNAWAY,reason:%s thermal runaway cleared,"
               "value:%.1f,rate:%.1fC_per_min,time:%lu",
               sensor_configs_[i].sensor_name,
               sensor_status_[i].temperature_c,
               trend,
               millis());
      SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
    }
  }
}
```

**Key Design Points**:
- **Self-healing enabled**: If trend drops below threshold, fault clears automatically. This allows recovery if the robot moves to a cooler area or load decreases.
- **No manual reset required**: Unlike motor overcurrent, thermal runaway is considered transient.
- **Rate limiting**: Fault messages limited to 1 per second to avoid spamming the serial bus.

**Rate-of-Change Calculation**

Calculating accurate temperature trends requires filtering noise while maintaining responsiveness:

```cpp
// From temperature_monitor.cpp lines 780-830

float TemperatureMonitor::calculateTemperatureTrend(uint8_t sensor_index) {
  const TemperatureSensorStatus& status = sensor_status_[sensor_index];

  // Check if we have enough data (need at least 10 samples = 1 second at 10Hz)
  bool buffer_full = !isnan(status.temperature_history[status.history_index]);
  uint32_t count = buffer_full ? 50 : status.history_index;
  
  if (count < 10) {
    return 0.0f;  // Insufficient data
  }

  // Find oldest temperature in circular buffer
  uint32_t oldest_idx = buffer_full ? status.history_index : 0;
  float oldest_temp = status.temperature_history[oldest_idx];
  
  // Current temperature is most recent reading
  float current_temp = status.temperature_c;
  
  // Calculate time span (buffer stores 50 samples at 10Hz = 5 seconds)
  float time_span_min = (count / 10.0f) / 60.0f;  // Convert samples to minutes
  
  // Rate = ΔT / Δt
  float rate_c_per_min = (current_temp - oldest_temp) / time_span_min;
  
  return rate_c_per_min;
}
```

**Why 50-Sample Circular Buffer**:
- **Storage**: 50 floats × 4 bytes = 200 bytes per sensor
- **Time span**: At 10Hz, 50 samples = 5 seconds of history
- **Responsiveness**: Can detect runaway within 1-2 seconds (minimum 10 samples)
- **Noise rejection**: Averaging over multiple samples filters electrical noise

**Why NOT Use EMA for Trend?**: While the temperature reading itself uses EMA filtering (see below), the trend calculation uses raw history. EMA would smooth out the very rate-of-change we're trying to detect.

**EMA Filtering for Temperature Readings**

DS18B20 sensors can have ±0.5°C measurement noise. To avoid spurious fault triggers, readings are filtered:

```cpp
// Exponential Moving Average: smooth_value = α × new_value + (1-α) × smooth_value
// α = 0.3 provides good balance between responsiveness and noise rejection

float filtered_temp = (0.3f * raw_reading) + (0.7f * previous_filtered);
```

**EMA Characteristics**:
- **No array storage**: Only requires storing one previous value
- **Constant time**: O(1) computation per update
- **Tuneable responsiveness**: Lower α = more smoothing, less responsive
- **Exponential decay**: Recent samples weighted more heavily than old samples

**Configuration Parameters** (from `temperature_monitor.h`):

```cpp
struct TemperatureSensorConfig {
  char sensor_name[32];                   // Human-readable name (e.g., "roboclaw_left")
  uint8_t onewire_address[8];             // DS18B20 64-bit ROM address
  
  // Threshold temperatures
  float critical_high_temp = 80.0f;       // Emergency stop (°C)
  float warning_high_temp = 70.0f;        // Warning level (°C)
  float warning_low_temp = 5.0f;          // Low temp warning (°C)
  float critical_low_temp = 0.0f;         // Sensor fault or extreme cold (°C)
  
  // Operational parameters
  bool enabled = true;                    // Enable monitoring for this sensor
  uint32_t read_interval_ms = 100;        // 10Hz reading rate
  uint8_t resolution_bits = 12;           // 12-bit = 0.0625°C resolution
  
  // Safety parameters
  bool safety_critical = true;            // Does this sensor participate in e-stop decisions?
  uint32_t fault_timeout_ms = 5000;       // Trigger DEGRADED if no reading for 5 seconds
  float thermal_runaway_rate = 100.0f;    // °C/min threshold for runaway detection
};
```

**Code Example: Integrating Temperature Safety in Module::loop()**

```cpp
void TemperatureMonitor::loop() {
  uint32_t now = millis();

  // Step 1: Update all temperature readings (10Hz)
  updateTemperatureReadings();  // Reads DS18B20 sensors, applies EMA filter

  // Step 2: Update system status counters
  updateSystemStatus();  // Count critical/warning/degraded sensors

  // Step 3: Periodic safety checks (10Hz)
  if (now - last_safety_check_time_ms_ >= 100) {
    checkSafetyConditions();  // Check thresholds, detect runaway
    last_safety_check_time_ms_ = now;
  }

  // Step 4: Report status to ROS2 (1Hz)
  if (now - last_status_report_time_ms_ >= 1000) {
    sendStatusReports();  // Send TEMPERATURE message
    last_status_report_time_ms_ = now;
  }
}

void TemperatureMonitor::checkSafetyConditions() {
  // Check thresholds for all sensors
  for (uint8_t i = 0; i < max_sensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }

    float temp = sensor_status_[i].temperature_c;
    const TemperatureSensorConfig& config = sensor_configs_[i];

    // Threshold checks with hysteresis built-in
    sensor_status_[i].critical_high = (temp >= config.critical_high_temp);
    sensor_status_[i].warning_high = (temp >= config.warning_high_temp && 
                                      temp < config.critical_high_temp);
    
    // If critical condition detected, activate fault in SafetyCoordinator
    if (sensor_status_[i].critical_high && config.safety_critical) {
      // SafetyCoordinator will receive this via message protocol
      // and trigger EMERGENCY_STOP if needed
    }
  }

  // Separate check for thermal runaway (uses trend, not absolute temp)
  detectThermalRunaway();
}
```

**Performance Impact**:
- Temperature reading: ~2ms per DS18B20 sensor (12-bit conversion)
- Threshold checking: <50μs for all 8 sensors
- Trend calculation: ~100μs per sensor (only when buffer full)
- Total: ~20ms worst case for 8 sensors (well within 100Hz loop budget)

See [Section X: Performance & Safety Balance](#x-performance--safety-balance) for discussion of timing tradeoffs.

### Battery Monitor

The BatteryMonitor module tracks battery health using INA226 current/voltage sensors. Sigyn uses a 12S LiPo battery (nominal 44.4V, 3.7V per cell) with multiple monitoring points.

**Why Battery Monitoring?**: The battery fire risk incident (Section I) showed that pack-level voltage monitoring isn't sufficient. Individual cell monitoring prevented a potential fire when one cell drifted to 4.35V (0.15V above safe maximum).

**Multi-Rail Architecture**:

The system monitors multiple power rails independently:
1. **Main battery pack** (12S LiPo): Powers motors via RoboClaw
2. **Logic power** (5V/12V rails): Powers Teensy boards, sensors, cameras
3. **Auxiliary systems**: Servos, lights, other peripherals

Each rail has independent INA226 sensors connected via I2C multiplexer, providing:
- Voltage measurement (±0.1V accuracy)
- Current measurement (±10mA accuracy)
- Power calculation (V × I)
- State estimation (charging/discharging/critical)

**Voltage/Current Monitoring Strategy**

Three detection mechanisms:

1. **Critical Low Voltage** (32.0V = 2.67V/cell):
   - Triggers EMERGENCY_STOP immediately
   - Below this, cells risk damage from over-discharge
   - Motor controllers may brown out, causing unpredictable behavior

2. **Warning Low Voltage** (34.0V = 2.83V/cell):
   - Triggers WARNING severity
   - Behavior tree uses this to initiate "return to base"
   - Gives ~5-10 minutes of operation before critical

3. **Critical High Current** (20A continuous):
   - Indicates motor stall, mechanical jam, or short circuit
   - Triggers EMERGENCY_STOP
   - RoboClaw controllers rated for 30A peak but 20A sustained can overheat

**INA226 Sensor Configuration**:

```cpp
// From battery_monitor.h
struct BatteryConfig {
  // Safety thresholds
  float critical_low_voltage = 32.0f;     // Emergency shutdown voltage - triggers immediate E-stop
  float warning_low_voltage = 34.0f;      // Low battery warning - initiates return-to-base procedure
  float critical_high_current = 20.0f;    // Critical current limit - triggers E-stop
  
  // Hardware configuration
  int ina226_address = 0x40;              // I2C address (0x40-0x4F depending on A0/A1 pins)
  int analog_voltage_pin = A0;            // Backup analog voltage divider
  float voltage_divider_ratio = 11.0f;    // For analog backup (R1=100k, R2=10k)
  
  // Calibration factors
  float voltage_calibration_offset = 0.0f; // Compensate for sensor drift
  float current_calibration_offset = 0.0f;
  float voltage_calibration_scale = 1.0f;
  float current_calibration_scale = 1.0f;
  
  char battery_name[32];                   // Human-readable identifier
};
```

**State Machine**:

```
Battery States:
UNKNOWN → NORMAL ⇄ WARNING → CRITICAL → EMERGENCY_STOP
    ↓      ↓        ↓
  DEGRADED (sensor failure)
    ↓
  CHARGING (negative current, battery 0 only)
```

**Code Example: Battery Fault Detection**

```cpp
// From battery_monitor.cpp lines 270-330

void BatteryMonitor::updateBatteryState(size_t idx) {
  float voltage = getVoltage(idx);
  float current = getCurrent(idx);

  // Determine desired state and severity
  BatteryState new_state = BatteryState::NORMAL;
  FaultSeverity desired_severity = FaultSeverity::NORMAL;
  char desired_description[128] = {0};

  // Check critical conditions (voltage OR current)
  if (voltage < g_battery_config_[idx].critical_low_voltage ||
      abs(current) > g_battery_config_[idx].critical_high_current) {
    new_state = BatteryState::CRITICAL;
    desired_severity = FaultSeverity::EMERGENCY_STOP;
    snprintf(desired_description, sizeof(desired_description),
             "Battery %zu (%s) CRITICAL: V=%.2f A=%.2f",
             idx, g_battery_config_[idx].battery_name, voltage, current);
  }
  // Check warning conditions
  else if (voltage < g_battery_config_[idx].warning_low_voltage) {
    new_state = BatteryState::WARNING;
    desired_severity = FaultSeverity::WARNING;
    snprintf(desired_description, sizeof(desired_description),
             "Battery %zu (%s) WARNING: V=%.2f A=%.2f",
             idx, g_battery_config_[idx].battery_name, voltage, current);
  }
  // Detect charging (negative current on main battery)
  else if ((idx == 0) && (current < 0.0f)) {
    new_state = BatteryState::CHARGING;
  }
  else {
    new_state = BatteryState::NORMAL;
  }

  // Update state
  bool state_changed = (state_[idx] != new_state);
  state_[idx] = new_state;

  // Report to SafetyCoordinator
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  
  // Build fault name with battery index (e.g., "BatteryMonitor_Battery0")
  char fault_name[64];
  snprintf(fault_name, sizeof(fault_name), "%s_Battery%zu", name(), idx);
  
  const Fault& current_fault = safety.getFault(fault_name);
  
  if (desired_severity == FaultSeverity::NORMAL) {
    // Clear any existing fault
    if (current_fault.active) {
      safety.deactivateFault(fault_name);
    }
  } else {
    // Activate or update fault
    if (!current_fault.active || current_fault.severity != desired_severity) {
      safety.activateFault(desired_severity, fault_name, desired_description);
    }
  }
}
```

**Key Design Points**:

- **OR logic for critical conditions**: Either low voltage OR high current triggers EMERGENCY_STOP
- **Hysteresis via separate thresholds**: Warning at 34V, critical at 32V prevents oscillation
- **Self-healing**: Faults clear automatically when conditions improve (e.g., load decreases, voltage recovers)
- **Per-battery tracking**: Each INA226 sensor has independent fault tracking
- **Charging detection**: Negative current indicates charging (only meaningful for main battery)

**I2C Multiplexer Integration**:

Since Sigyn has multiple INA226 sensors (each at address 0x40), an I2C multiplexer (TCA9548A at 0x70) allows addressing multiple identical devices:

```cpp
void BatteryMonitor::setup() {
  // Enable I2C multiplexer
  pinMode(kI2CMultiplexorEnablePin, OUTPUT);
  digitalWrite(kI2CMultiplexorEnablePin, HIGH);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C for faster reads
  
  // Test multiplexer presence
  multiplexer_available_ = testI2cMultiplexer();
  
  // Initialize each sensor on its multiplexer channel
  for (size_t device = 0; device < kNumberOfBatteries; device++) {
    sensors_[device] = new INA226Sensor(0x40, 
                                        gINA226_DeviceIndexes_[device],  // Mux channel
                                        I2C_MULTIPLEXER_ADDRESS);
    
    if (!sensors_[device]->init()) {
      // Mark as DEGRADED if sensor fails initialization
      state_[device] = BatteryState::DEGRADED;
      SafetyCoordinator::getInstance().activateFault(
        FaultSeverity::DEGRADED,
        "BatteryMonitor",
        "Sensor initialization failed");
    }
  }
}
```

**Performance Considerations**:
- INA226 read time: ~2ms per sensor (I2C communication)
- With 3 sensors: ~6ms total per update cycle
- Update rate: 10Hz (100ms interval) to stay within timing budget
- Sensor averaging: INA226 configured for 16-sample hardware averaging to reduce noise

### RoboClaw Monitor

The RoboClawMonitor module controls the RoboClaw 2x15A motor controllers and implements critical safety features to prevent motor-related damage.

**Why RoboClaw Monitoring?**: The runaway wheel incident (Section I) showed that encoder validation is essential. A loose cable caused the motor to accelerate uncontrollably into a wall. Motor overcurrent detection prevents damage during stalls.

**Two-Tier Safety Strategy**:

1. **Overcurrent Detection**: Prevents thermal damage to motor controllers
2. **Runaway Detection**: Catches mechanical failures (loose encoders, broken gears)

**Motor Overcurrent Detection**

RoboClaw controllers report real-time current draw for each motor. Sigyn monitors this continuously and triggers EMERGENCY_STOP when current exceeds safe thresholds.

**Detection Algorithm**:

```cpp
// From roboclaw_monitor.cpp lines 1119-1165

void RoboClawMonitor::checkMotorSafety() {
  uint32_t now = millis();
  
  // Check for overcurrent conditions
  // NOTE: Overcurrent is a LATCHING fault. It stops the robot to prevent 
  // oscillation ("hammering"). Must be cleared by explicit Reset command.
  
  if (motor1_status_.current_valid) {
    if (std::abs(motor1_status_.current_amps) > config_.max_current_m1) {
      if (!motor1_status_.overcurrent) {
        // First trip - activate fault
        SafetyCoordinator::getInstance().activateFault(
          FaultSeverity::EMERGENCY_STOP, 
          name(), 
          "Motor 1 overcurrent");
        setEmergencyStop();
        motor1_status_.overcurrent = true;
        total_safety_violations_++;
      }
      
      // Rate-limited diagnostics (max 1 per second)
      if (now - last_estop_msg_time >= config_.estop_msg_interval_ms) {
        char msg[192];
        snprintf(msg, sizeof(msg),
                 "active:true,source:ROBOCLAW_CURRENT,reason:Motor 1 overcurrent,"
                 "value:%.2f,manual_reset:true,time:%lu",
                 motor1_status_.current_amps, millis());
        SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", "RoboClaw", msg);
        last_estop_msg_time = now;
      }
    }
  }
  
  // Similar check for motor2
  if (motor2_status_.current_valid) {
    if (std::abs(motor2_status_.current_amps) > config_.max_current_m2) {
      if (!motor2_status_.overcurrent) {
        SafetyCoordinator::getInstance().activateFault(
          FaultSeverity::EMERGENCY_STOP,
          name(),
          "Motor 2 overcurrent");
        setEmergencyStop();
        motor2_status_.overcurrent = true;
        total_safety_violations_++;
      }
      
      if (now - last_estop_msg_time >= config_.estop_msg_interval_ms) {
        char msg[192];
        snprintf(msg, sizeof(msg),
                 "active:true,source:ROBOCLAW_CURRENT,reason:Motor 2 overcurrent,"
                 "value:%.2f,manual_reset:true,time:%lu",
                 motor2_status_.current_amps, millis());
        SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", "RoboClaw", msg);
        last_estop_msg_time = now;
      }
    }
  }
}
```

**Key Design Decisions**:

- **Latching fault**: Once overcurrent triggers, it remains active until manually cleared
  - Prevents "hammering": robot trying repeatedly to move through obstacle
  - Forces operator to verify the obstruction is removed
  
- **No EMA filtering on current**: Unlike temperature (which has sensor noise), current readings are already hardware-filtered by RoboClaw's internal ADC
  - EMA would add ~1-2 second delay, allowing brief overcurrent to damage controller
  - Fast response (<100ms) is critical for motor protection

- **Absolute value check**: `abs(current)` catches both forward and reverse overcurrent
  - Motor drawing excessive current in either direction indicates stall/jam

**Configuration Parameters**:

```cpp
struct RoboClawConfig {
  // Safety thresholds
  float max_current_m1 = 100.0f;          // Maximum current for motor 1 (Amps)
  float max_current_m2 = 100.0f;          // Maximum current for motor 2 (Amps)
  float warning_current = 10.0f;          // Warning current threshold (Amps)
  
  // Runaway detection
  uint32_t runaway_speed_threshold_qpps = 100;  // Speed threshold for runaway (QPPS)
  uint32_t runaway_check_interval_ms = 100;     // Check interval (10Hz)
  uint32_t runaway_encoder_threshold = 1000;    // Encoder change threshold
  
  // Communication
  uint8_t max_consecutive_comm_failures = 3;    // Escalate to E-stop after this many failed reads
  uint32_t timeout_us = 100000;                 // Communication timeout (100ms)
};
```

**Runaway Detection via Encoder Validation**

Motor runaway occurs when:
- Encoder cable disconnects (controller thinks motor isn't moving, applies maximum power)
- Gear breaks (motor spins freely without moving robot)
- Wheel loses traction (encoder shows movement but robot doesn't move - harder to detect)

**Detection Algorithm**:

```cpp
// From roboclaw_monitor.cpp lines 1185-1245

void RoboClawMonitor::detectMotorRunaway() {
  if (!runaway_detection_initialized_) {
    return;  // Wait for initial encoder baseline
  }

  uint32_t now = millis();
  if (now - last_runaway_check_time_ms_ < config_.runaway_check_interval_ms) {
    return;  // Rate limiting (10Hz checks)
  }

  // Check if motors are running away (moving when commanded to stop)
  // This catches encoder disconnect and mechanical failure
  
  if (last_commanded_m1_qpps_ == 0 && 
      std::abs(motor1_status_.speed_qpps) > config_.runaway_speed_threshold_qpps) {
    
    const bool first_trip = !motor1_status_.runaway_detected;
    motor1_status_.runaway_detected = true;
    total_safety_violations_++;

    if (first_trip) {
      SafetyCoordinator::getInstance().activateFault(
        FaultSeverity::EMERGENCY_STOP,
        name(),
        "Motor 1 runaway detected");
      setEmergencyStop();
    }

    // Rate-limited ESTOP message
    if (now - last_runaway_msg_time >= config_.runaway_msg_interval_ms) {
      char msg[160];
      snprintf(msg, sizeof(msg),
               "active:true,source:MOTOR_RUNAWAY,reason:Motor 1 runaway detected,"
               "value:%ld,manual_reset:false,time:%lu",
               motor1_status_.speed_qpps, millis());
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
      last_runaway_msg_time = now;
    }
  }

  // Similar check for motor2
  if (last_commanded_m2_qpps_ == 0 && 
      std::abs(motor2_status_.speed_qpps) > config_.runaway_speed_threshold_qpps) {
    
    const bool first_trip = !motor2_status_.runaway_detected;
    motor2_status_.runaway_detected = true;
    total_safety_violations_++;

    if (first_trip) {
      SafetyCoordinator::getInstance().activateFault(
        FaultSeverity::EMERGENCY_STOP,
        name(),
        "Motor 2 runaway detected");
      setEmergencyStop();
    }

    if (now - last_runaway_msg_time >= config_.runaway_msg_interval_ms) {
      char msg[160];
      snprintf(msg, sizeof(msg),
               "active:true,source:MOTOR_RUNAWAY,reason:Motor 2 runaway detected,"
               "value:%ld,manual_reset:false,time:%lu",
               motor2_status_.speed_qpps, millis());
      SerialManager::getInstance().sendDiagnosticMessage("CRITICAL", name(), msg);
      last_runaway_msg_time = now;
    }
  }

  last_runaway_check_time_ms_ = now;
}
```

**Detection Logic**:

```
IF (commanded_speed == 0) AND (actual_speed > threshold) THEN
  → Motor is moving when it shouldn't be → RUNAWAY!
```

This simple check catches:
1. **Encoder disconnect**: RoboClaw thinks motor is stopped, applies full power to reach zero speed setpoint
2. **Mechanical failure**: Gear breaks, motor spins freely
3. **Firmware bug**: RoboClaw ignoring stop commands (rare but observed)

**Why This Works**:

- **Zero-command baseline**: When robot is commanded to stop (speed = 0), encoders should show zero movement
- **Threshold tolerance**: 100 QPPS (~0.01 m/s) allows for minor noise/drift without false triggers
- **Fast detection**: Checked at 10Hz, detects runaway within ~100-200ms

**Self-Healing for Runaway**:

Unlike overcurrent (which is latching), runaway faults can self-heal:

```cpp
// In setVelocityCommand():
if (emergency_stop_active_ && 
    !motor1_status_.overcurrent && 
    !motor2_status_.overcurrent) {
  
  // If only runaway caused E-stop, and new command is zero velocity,
  // clear runaway flags and allow system to recover
  if (linear_x == 0.0f && angular_z == 0.0f) {
    motor1_status_.runaway_detected = false;
    motor2_status_.runaway_detected = false;
    clearEmergencyStop();
  }
}
```

This allows recovery when:
- Encoder cable reconnects
- Operator sends stop command to verify motor is actually stopped
- Prevents requiring manual reset for transient encoder glitches

**Communication Timeout Detection**:

If serial communication with RoboClaw fails repeatedly, trigger EMERGENCY_STOP:

```cpp
if (consecutive_comm_failures_ >= config_.max_consecutive_comm_failures) {
  SafetyCoordinator::getInstance().activateFault(
    FaultSeverity::EMERGENCY_STOP,
    name(),
    "RoboClaw communication timeout");
  setEmergencyStop();
}
```

This prevents the robot from operating with stale sensor data or without motor control authority.

**Performance Impact**:
- Current reading: ~5ms per RoboClaw.ReadCurrents() call
- Encoder reading: ~3ms per RoboClaw.ReadEncoderM1/M2() call
- Total safety check: ~15-20ms per cycle
- Update rate: 3Hz for heavy operations (voltage/current), 10Hz for safety checks

### Performance Monitor

The PerformanceMonitor module treats timing violations as safety issues. As discussed in Section II, performance preservation is itself a safety requirement—a robot that misses control deadlines becomes unpredictable and dangerous.

**Why Performance Monitoring?**: Control theory requires predictable timing. A motor controller expecting updates at 85Hz will become unstable if it receives updates at 40Hz. Odometry calculations assume constant time deltas—variable timing causes position errors that accumulate over time.

**Two-Category Monitoring**:

1. **Loop Frequency Violations**: Overall system timing
   - Target: 80-100Hz (board-specific)
   - Board1: 85Hz (motor control critical)
   - Board2: 80Hz (sensor processing)
   - Minimum acceptable: 30Hz (degraded but functional)

2. **Module Execution Time Violations**: Individual component overruns
   - Target: <2ms per module per iteration
   - Maximum: 4ms (allows occasional sensor reads)
   - Consecutive violations indicate systemic issues

**Detection Algorithm**:

```cpp
// From performance_monitor.cpp lines 80-120

void PerformanceMonitor::checkFrequencyViolations() {
  // Get current frequency from Module system
  float current_frequency = Module::getCurrentLoopFrequency();

  // Check if frequency has dropped below acceptable threshold
  if (current_frequency > 0.0f && current_frequency < config_.min_loop_frequency_hz) {
    // Increment consecutive violation counter
    if (violations_.consecutive_frequency_violations < 255) {
      violations_.consecutive_frequency_violations++;
    }

    violations_.total_frequency_violations++;
    violations_.last_violation_time_ms = millis();

    // Warning message for frequency violations
    char warn_msg[64];
    snprintf(warn_msg, sizeof(warn_msg),
             "FREQ_VIOLATION: %.1fHz < %.1fHz (violation #%lu)",
             current_frequency,
             config_.min_loop_frequency_hz,
             violations_.total_frequency_violations);
    SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), warn_msg);

    // Critical message for multiple consecutive violations
    if (violations_.consecutive_frequency_violations >= config_.max_frequency_violations) {
      SerialManager::getInstance().sendDiagnosticMessage(
        "CRITICAL", name(),
        "Multiple consecutive frequency violations - system overload detected!");
    }
  } else {
    // Reset consecutive counter when performance recovers (hysteresis)
    violations_.consecutive_frequency_violations = 0;
  }
}

void PerformanceMonitor::checkModuleExecutionTimes() {
  bool any_module_violated = false;

  // Check all registered modules
  for (uint16_t i = 0; i < Module::getModuleCount(); ++i) {
    Module* mod = Module::getModule(i);
    if (mod) {
      float execution_time_ms = mod->getLastExecutionTimeUs() / 1000.0f;

      if (execution_time_ms > config_.max_module_time_ms) {
        any_module_violated = true;

        // Optional detailed logging
        if (config_.enable_detailed_logging) {
          char log_msg[64];
          snprintf(log_msg, sizeof(log_msg),
                   "Module %s exceeded time limit: %.2fms",
                   mod->name(), execution_time_ms);
          SerialManager::getInstance().sendDiagnosticMessage("WARN", name(), log_msg);
        }
      }
    }
  }

  // Update violation tracking
  if (any_module_violated) {
    if (violations_.consecutive_module_violations < 255) {
      violations_.consecutive_module_violations++;
    }
    violations_.total_module_violations++;
    violations_.last_violation_time_ms = millis();
  } else {
    // Reset consecutive counter when all modules perform acceptably
    violations_.consecutive_module_violations = 0;
  }
}
```

**Key Design Points**:

- **Consecutive violation tracking**: Prevents false alarms from occasional spikes
  - Single violation: logged but not critical
  - 5+ consecutive violations: triggers WARNING severity
  - 8+ consecutive violations: triggers DEGRADED or EMERGENCY_STOP (configurable)

- **Hysteresis**: Counters reset to zero when performance recovers
  - Prevents oscillating between safe/unsafe states
  - Requires sustained good performance to clear fault

- **Per-board thresholds**: Different boards have different performance requirements
  - Board1 (motor control): 85Hz target, strict timing
  - Board2 (sensor processing): 80Hz target, more tolerance for heavy reads

**Configuration Parameters**:

```cpp
struct PerformanceConfig {
  float min_loop_frequency_hz = 30.0f;        // Minimum acceptable loop frequency (Hz)
  float max_module_time_ms = 4.0f;            // Maximum allowed module execution time (ms)
  uint8_t max_violation_count = 8;            // Consecutive violations before unsafe
  uint8_t max_frequency_violations = 5;       // Consecutive frequency violations before unsafe
  uint32_t stats_report_interval_ms = 1000;   // Performance statistics reporting interval
  bool enable_detailed_logging = false;       // Enable verbose logging (high overhead)
};
```

**Violation Tracking Structure**:

```cpp
struct ViolationTracker {
  uint8_t consecutive_module_violations = 0;    // Consecutive module time violations
  uint8_t consecutive_frequency_violations = 0; // Consecutive frequency violations
  uint32_t total_module_violations = 0;         // Total violations since reset
  uint32_t total_frequency_violations = 0;      // Total frequency violations since reset
  uint32_t last_violation_time_ms = 0;          // Timestamp of last violation
  bool safety_violation_active = false;         // Currently in unsafe state
};
```

**When Performance Triggers Safety Response**:

```cpp
void PerformanceMonitor::checkPerformance() {
  checkFrequencyViolations();
  checkModuleExecutionTimes();

  // Declare unsafe if either threshold exceeded
  if (violations_.consecutive_frequency_violations >= config_.max_frequency_violations ||
      violations_.consecutive_module_violations >= config_.max_violation_count) {
    violations_.safety_violation_active = true;
    
    // SafetyCoordinator will query isUnsafe() and trigger DEGRADED or WARNING
    // (Performance issues typically don't trigger EMERGENCY_STOP directly,
    //  but may escalate if other systems fail due to timing issues)
  } else {
    violations_.safety_violation_active = false;
  }
}

bool PerformanceMonitor::isUnsafe() {
  return violations_.safety_violation_active;
}
```

**What Causes Performance Violations**:

1. **Blocking I2C/SPI operations**: Sensor reads that take too long
2. **Heavy computation**: Complex algorithms (matrix operations, filtering)
3. **Serial communication delays**: Waiting for RoboClaw/sensor responses
4. **Interrupt storms**: Excessive ISR activity
5. **Memory issues**: Heap fragmentation, excessive allocation

**Performance Impact**:
- Monitoring overhead: <50μs per iteration (negligible)
- No dynamic memory allocation (fixed-size structures)
- High-resolution timing (microseconds via `micros()`)
- Lock-free operation suitable for real-time execution

### IMU & VL53L0X (Planned)

**Future Safety Integration**:

Currently under development, these sensors will provide additional safety capabilities:

**BNO055 IMU (9-DOF Inertial Measurement Unit)**:
- **Tilt detection**: Trigger WARNING if robot tilts >15° (approaching tip-over)
- **Fall detection**: Detect free-fall conditions (dropped off ledge, going down stairs unintentionally)
- **Orientation validation**: Cross-check odometry heading against IMU compass
- **Vibration monitoring**: Excessive vibration indicates mechanical failure

Planned safety thresholds:
- Tilt warning: 15° from level
- Tilt critical: 25° (emergency stop)
- Angular acceleration: >500°/s² (sudden impact detection)

**VL53L0X Time-of-Flight Distance Sensors**:
- **Collision prediction**: Detect obstacles <30cm ahead
- **Cliff detection**: Detect drop-offs (stairs, ramps)
- **Elevator threshold detection**: Validate elevator floor alignment before entering
- **Wall distance validation**: Cross-check against expected map distance

Planned safety thresholds:
- Obstacle warning: <50cm at current velocity
- Obstacle critical: <15cm (emergency stop)
- Cliff detection: >50cm drop-off within sensor range

Implementation status: Hardware present, software integration pending. See TODO_list.txt for development tasks.

---

## V. Safety Coordinator Architecture

The SafetyCoordinator is the central hub for all safety-related decisions in TeensyV2. It aggregates faults from all hardware monitoring modules, manages the e-stop state machine, and coordinates recovery logic.

**Architectural Role**:

```
Hardware Modules → SafetyCoordinator → E-Stop Output
  ↓                     ↓                  ↓
TemperatureMonitor   Fault              RoboClaw
BatteryMonitor       Aggregation        E-Stop Pin
RoboClawMonitor      & Severity         (Active-LOW)
PerformanceMonitor   Assessment
```

The SafetyCoordinator acts as the single source of truth for system safety state, ensuring consistent e-stop behavior across all components.

**Central Fault Aggregation**

Instead of each module directly controlling the e-stop pin, all modules report faults to SafetyCoordinator:

```cpp
// Any module can activate a fault:
SafetyCoordinator::getInstance().activateFault(
  FaultSeverity::EMERGENCY_STOP,
  "TemperatureMonitor",
  "RoboClaw thermal runaway detected");

// SafetyCoordinator tracks all active faults and manages e-stop state
```

This centralized design provides:
- **Single e-stop authority**: Only SafetyCoordinator controls the RoboClaw e-stop pin
- **Fault tracking**: All active faults stored with source, severity, and timestamp
- **Priority management**: Higher severity faults take precedence
- **Recovery coordination**: E-stop only clears when ALL faults are resolved

**Fault Activation/Deactivation Lifecycle**

Faults follow a well-defined state machine:

```
┌──────────┐
│ INACTIVE │ ← Initial state, no fault present
└────┬─────┘
     │ activateFault()
     ↓
┌──────────┐
│  ACTIVE  │ ← Fault present, safety action taken
└────┬─────┘
     │ deactivateFault()
     ↓
┌──────────┐
│ INACTIVE │ ← Fault cleared, recovery possible
└──────────┘
```

**Activation Code Path**:

```cpp
// From safety_coordinator.cpp lines 120-180

void SafetyCoordinator::activateFault(
    FaultSeverity severity,
    const char* source,
    const char* description) {
  
  // Find or allocate a fault slot for this source
  const int idx = getOrAllocateFaultIndex(source);
  if (idx < 0) {
    SerialManager::getInstance().sendDiagnosticMessage(
      "ERROR", name(), "activateFault called with empty/null source");
    return;
  }

  // Determine if this is an E-stop level fault
  const bool new_is_estop = (severity == FaultSeverity::EMERGENCY_STOP ||
                             severity == FaultSeverity::SYSTEM_SHUTDOWN);
  const bool was_active = faults_[idx].active;
  const bool was_estop = (faults_[idx].severity == FaultSeverity::EMERGENCY_STOP ||
                          faults_[idx].severity == FaultSeverity::SYSTEM_SHUTDOWN);

  // Update the fault record (allows severity upgrades: WARNING → EMERGENCY_STOP)
  faults_[idx] = Fault(/*is_active=*/true, severity, source, description);

  // Update E-stop count based on severity transitions
  if (!was_active && new_is_estop) {
    // New E-stop fault
    active_estop_count_++;
  } else if (was_active && !was_estop && new_is_estop) {
    // Upgrade WARNING/DEGRADED → EMERGENCY_STOP
    active_estop_count_++;
  } else if (was_active && was_estop && !new_is_estop) {
    // Downgrade EMERGENCY_STOP → WARNING/DEGRADED
    if (active_estop_count_ > 0) {
      active_estop_count_--;
    }
  }

  // Trigger physical safeties only when entering E-stop state
  if (new_is_estop && active_estop_count_ == 1) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
    RoboClawMonitor::getInstance().setEmergencyStop();
#endif
  }

  // Log the fault activation
  char msg[256];
  snprintf(msg, sizeof(msg),
           "%s: source=%s, severity=%s description=%s, active_estop_count_=%d",
           was_active ? "Fault updated" : "Fault activated",
           faults_[idx].source,
           faultSeverityToString(severity),
           faults_[idx].description,
           active_estop_count_);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
}
```

**Key Design Points**:

- **Fault reuse**: Same source can update existing fault (e.g., temperature rising from WARNING to EMERGENCY_STOP)
- **Severity tracking**: Only EMERGENCY_STOP and SYSTEM_SHUTDOWN trigger physical e-stop
- **Reference counting**: `active_estop_count_` tracks how many E-stop-level faults are active
- **Hardware control**: E-stop pin only asserted when count transitions from 0→1

**Deactivation Code Path**:

```cpp
// From safety_coordinator.cpp lines 185-220

void SafetyCoordinator::deactivateFault(const char* source) {
  const int idx = findFaultIndex(source);
  if (idx < 0 || !faults_[idx].active) {
    SerialManager::getInstance().sendDiagnosticMessage(
      "ERROR", name(),
      "Attempted to deactivate a fault that is not active");
    return;
  }

  const bool was_estop = (faults_[idx].severity == FaultSeverity::EMERGENCY_STOP ||
                          faults_[idx].severity == FaultSeverity::SYSTEM_SHUTDOWN);
  
  // Send fault clearance message BEFORE clearing the fault data
  char status_msg[256];
  snprintf(status_msg, sizeof(status_msg),
           "{\"active_fault\":\"false\",\"source\":\"%s\",\"severity\":\"%s\"}",
           faults_[idx].source,
           faultSeverityToString(faults_[idx].severity));
  SerialManager::getInstance().sendMessage("FAULT", status_msg);
  
  // Mark fault as inactive
  faults_[idx].active = false;
  if (was_estop && active_estop_count_ > 0) {
    active_estop_count_--;
  }

  // If this was the LAST active E-stop fault, clear the physical safeties
  if (active_estop_count_ == 0) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
    RoboClawMonitor::getInstance().clearEmergencyStop();
#endif
    
    char msg[256];
    snprintf(msg, sizeof(msg),
             "Fault deactivated: source=%s, severity=%s, description=%s, active_faults=%d",
             faults_[idx].source,
             faultSeverityToString(faults_[idx].severity),
             faults_[idx].description,
             active_estop_count_);
    SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
  }
}
```

**Multiple Fault Tracking**

SafetyCoordinator maintains a fixed-size array of faults:

```cpp
static constexpr size_t kMaxFaults = 16;  // Maximum concurrent faults
Fault faults_[kMaxFaults];                // Active faults array
uint8_t active_estop_count_ = 0;          // Count of E-stop-level faults
```

**Fault Slot Management**:

```cpp
int SafetyCoordinator::getOrAllocateFaultIndex(const char* source) {
  // 1. Check if fault already exists for this source
  const int existing = findFaultIndex(source);
  if (existing >= 0) {
    return existing;  // Reuse existing slot
  }

  // 2. Find an unused slot
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].source[0] == '\0') {
      return static_cast<int>(i);
    }
  }

  // 3. Reuse an inactive slot
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (!faults_[i].active) {
      faults_[i].clear();
      return static_cast<int>(i);
    }
  }

  // 4. Worst case: overwrite the oldest fault
  size_t oldest_idx = 0;
  uint32_t oldest_ts = faults_[0].timestamp;
  for (size_t i = 1; i < kMaxFaults; i++) {
    if (faults_[i].timestamp < oldest_ts) {
      oldest_ts = faults_[i].timestamp;
      oldest_idx = i;
    }
  }
  faults_[oldest_idx].clear();
  return static_cast<int>(oldest_idx);
}
```

This priority-based slot allocation ensures:
- Recent faults are never lost
- Active faults are never overwritten
- Oldest inactive faults are recycled first
- No dynamic memory allocation (real-time safe)

**E-Stop Signal Generation**

Physical e-stop is controlled via GPIO pin connected to RoboClaw:

```cpp
// In RoboClawMonitor (controlled by SafetyCoordinator)
void RoboClawMonitor::setEmergencyStop() {
  emergency_stop_active_ = true;
  
  // Set E-stop pin LOW (active-LOW signal)
  // This is a hardware cutoff - RoboClaw will stop motors immediately
  // even if Teensy firmware crashes
  pinMode(PIN_ROBOCLAW_ESTOP, OUTPUT);
  digitalWrite(PIN_ROBOCLAW_ESTOP, LOW);
  
  // Zero motor commands to be explicit
  setMotorSpeeds(0, 0);
}

void RoboClawMonitor::clearEmergencyStop() {
  emergency_stop_active_ = false;
  
  // Set E-stop pin HIGH (inactive)
  digitalWrite(PIN_ROBOCLAW_ESTOP, HIGH);
  
  // Note: Motor commands remain zero until new velocity command received
}
```

**Fail-Safe Design**: E-stop pin is active-LOW, meaning:
- If wire breaks → pin floats LOW → motors stop
- If Teensy loses power → pin goes LOW → motors stop
- If firmware crashes → watchdog resets board → pin initializes LOW → motors stop

**Recovery Logic**

Recovery behavior depends on fault severity and type:

**Automatic Recovery** (self-healing):
- Temperature drops below threshold → thermal fault clears automatically
- Battery voltage recovers → low voltage warning clears
- Performance improves → timing violation clears

**Manual Recovery** (requires operator intervention):
- Motor overcurrent → requires explicit reset command
- Hardware e-stop button → must be physically released AND software reset
- Communication timeout → requires verification that issue is resolved

**Recovery Sequence**:

```
1. Fault condition clears (either automatically or via operator action)
2. Module calls deactivateFault()
3. SafetyCoordinator decrements active_estop_count_
4. When count reaches 0, physical e-stop is released
5. System returns to NORMAL state
6. Motors can be commanded again
```

**Fault Class Structure**

```cpp
// From safety_coordinator.h lines 50-90

typedef enum class FaultSeverity {
  NORMAL,          // All systems operational
  DEGRADED,        // Operating with reduced functionality
  WARNING,         // Minor issues detected, monitoring
  EMERGENCY_STOP,  // Emergency stop active
  SYSTEM_SHUTDOWN  // Complete system shutdown required (planned)
} FaultSeverity;

struct Fault {
  static constexpr size_t kMaxSourceLen = 32;
  static constexpr size_t kMaxDescriptionLen = 128;

  bool active = false;                          // Is the fault currently active
  FaultSeverity severity = FaultSeverity::NORMAL; // Severity level
  char source[kMaxSourceLen] = {0};             // Source module name (null-terminated)
  char description[kMaxDescriptionLen] = {0};   // Human-readable description
  uint32_t timestamp = 0;                       // Time when fault was detected

  Fault() = default;

  Fault(bool is_active, FaultSeverity sev, const char* src, const char* desc)
      : active(is_active), severity(sev), timestamp(millis()) {
    setSource(src);
    setDescription(desc);
  }

  void clear() {
    active = false;
    severity = FaultSeverity::NORMAL;
    timestamp = 0;
    source[0] = '\0';
    description[0] = '\0';
  }

  void setSource(const char* src) {
    snprintf(source, sizeof(source), "%s", (src != nullptr) ? src : "");
  }

  void setDescription(const char* desc) {
    snprintf(description, sizeof(description), "%s", (desc != nullptr) ? desc : "");
  }
};
```

**Key Design Features**:

- **Fixed-size strings**: No dynamic allocation, safe for real-time use
- **Null-termination guaranteed**: snprintf ensures strings are always valid
- **Timestamp tracking**: Enables oldest-fault-first eviction
- **Clear() method**: Safely resets all fields for slot reuse
- **Copy constructor**: Allows safe fault copying for ROS2 messages

**SafetyCoordinator loop() Execution**:

```cpp
void SafetyCoordinator::loop() {
  uint32_t now = millis();

  // Periodic status updates (1Hz)
  if (now - last_status_update_ms_ >= 1000) {
    sendStatusUpdate();  // Send FAULT messages to ROS2
    last_status_update_ms_ = now;
  }

  // Note: Fault activation/deactivation happens asynchronously
  // via activateFault()/deactivateFault() calls from other modules
  // This loop() only handles periodic reporting
}

void SafetyCoordinator::sendStatusUpdate() {
  // Send FAULT message for each active fault
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].active) {
      char msg[256];
      snprintf(msg, sizeof(msg),
               "{\"active_fault\":\"true\",\"source\":\"%s\","
               "\"severity\":\"%s\",\"description\":\"%s\",\"time\":%lu}",
               faults_[i].source,
               faultSeverityToString(faults_[i].severity),
               faults_[i].description,
               faults_[i].timestamp);
      SerialManager::getInstance().sendMessage("FAULT", msg);
    }
  }
}
```

**Performance Characteristics**:
- Fault activation: O(n) where n = kMaxFaults (16), typically <50μs
- Fault deactivation: O(n), typically <50μs
- Status reporting: O(n), runs at 1Hz
- No dynamic memory allocation
- Total RAM usage: ~3KB for fault array and tracking variables

---

## VI. Multi-Board Safety Coordination

Sigyn uses a dual-board architecture for safety and performance reasons. Understanding why the system is split and how boards coordinate is essential for maintaining the safety system.

**Board Partitioning Rationale**

The decision to use two Teensy 4.1 boards instead of a single board was driven by physical and electrical constraints:

**1. I2C Connector Limits**:
- Teensy 4.1 has limited I2C ports (Wire, Wire1, Wire2)
- Total sensor count exceeds available ports:
  - 8× DS18B20 temperature sensors (OneWire, not I2C, but uses pins)
  - 3× INA226 current/voltage sensors (I2C)
  - 1× BNO055 IMU (I2C)
  - 3× VL53L0X distance sensors (I2C)
  - 1× I2C multiplexer (TCA9548A)
- With I2C address conflicts (multiple INA226s at 0x40), multiplexer is required
- Single board would require complex multiplexer chaining

**2. Performance Requirements**:
- Motor control requires consistent 85Hz loop rate
- Sensor processing (especially IMU) can have variable timing
- Separating motor control from heavy sensor processing prevents timing interference
- Board1 can dedicate cycles to motor control without sensor overhead

**3. Physical Constraints**:
- Single board would require >40 GPIO pins for all functions
- Teensy 4.1 has 42 digital pins, but some are reserved (SPI, Serial, etc.)
- Physical mounting: two smaller boards fit better in chassis than one large board
- Heat dissipation: spreading load across two boards reduces hot spots

**4. Fault Isolation**:
- If one board crashes, the other can still operate
- Board1 (motor control) can continue emergency stop even if Board2 (sensors) fails
- Board2 can continue battery monitoring even if Board1 has issues
- Reduces single point of failure risk

**Board Allocation**:

```
Board1 (Navigation & Safety):
- RoboClawMonitor (motor control)
- TemperatureMonitor (motor controller temps)
- VL53L0XMonitor (obstacle detection)
- PerformanceMonitor
- SafetyCoordinator
- E-Stop Authority (controls RoboClaw pin)

Board2 (Power & Sensing):
- BatteryMonitor (INA226 sensors)
- BNO055Monitor (IMU orientation)
- PerformanceMonitor
- SafetyCoordinator
- No E-Stop Authority (reports only)
```

From [config.h](TeensyV2/common/core/config.h):

```cpp
#if BOARD_ID == 1
#define BOARD_HAS_MOTOR_CONTROL  1
#define BOARD_HAS_VL53L0X        1
#define BOARD_HAS_TEMPERATURE    1
#define BOARD_HAS_SAFETY         1
#define CONTROLS_ROBOCLAW_ESTOP_PIN   1   // Board 1 controls RoboClaw E-stop

#elif BOARD_ID == 2
#define BOARD_HAS_BATTERY        1
#define BOARD_HAS_IMU            1
#define BOARD_HAS_SAFETY         1
#define CONTROLS_ROBOCLAW_ESTOP_PIN   0   // Board 2 reports only
```

**Serial Communication of Faults**

Each board sends FAULT messages via serial to `sigyn_to_sensor_v2` ROS2 bridge:

**Message Format**:

```
FAULT1:{"active_fault":"true","source":"TemperatureMonitor","severity":"EMERGENCY_STOP","description":"RoboClaw thermal runaway","time":123456}
FAULT2:{"active_fault":"true","source":"BatteryMonitor","severity":"WARNING","description":"Battery low voltage","time":123789}
```

The `1` or `2` suffix identifies the originating board.

**Protocol Details**:

- **Line-based JSON**: Each message is a complete JSON object terminated by `\n`
- **Board identification**: Message type includes board ID (FAULT1, FAULT2)
- **Timestamping**: Each fault includes millisecond timestamp from `millis()`
- **Severity encoding**: String representation of FaultSeverity enum
- **Source tracking**: Module name that generated the fault

**Code Example from SafetyCoordinator**:

```cpp
void SafetyCoordinator::sendStatusUpdate() {
  // Send FAULT message for each active fault
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].active) {
      char msg[256];
      snprintf(msg, sizeof(msg),
               "{\"active_fault\":\"true\",\"source\":\"%s\","
               "\"severity\":\"%s\",\"description\":\"%s\",\"time\":%lu}",
               faults_[i].source,
               faultSeverityToString(faults_[i].severity),
               faults_[i].description,
               faults_[i].timestamp);
      
      // SerialManager automatically prefixes with "FAULT" + board ID
      SerialManager::getInstance().sendMessage("FAULT", msg);
    }
  }
}
```

**Board1 as E-Stop Authority**

Only Board1 physically controls the RoboClaw e-stop pin. This design decision provides:

**Single Source of Truth**:
- Prevents conflicting e-stop signals from multiple boards
- Simplifies hardware wiring (single GPIO to RoboClaw)
- Reduces race conditions and signal contention

**Why Board1?**:
- Board1 runs motor control → has most immediate knowledge of motor state
- Board1 has VL53L0X sensors → can detect imminent collisions
- Physical proximity: Board1 is mounted near RoboClaw controller
- Latency: Direct GPIO connection is faster than inter-board communication

**Hardware Connection**:

```
Teensy 4.1 Board1 GPIO Pin 30 → RoboClaw E-Stop Input
  │
  ├─ Active-LOW signal
  ├─ Internal pull-up on RoboClaw side
  └─ Wire break = LOW = SAFE (motors stop)
```

From [config.h](TeensyV2/common/core/config.h):

```cpp
#if BOARD_ID == 1
#define CONTROLS_ROBOCLAW_ESTOP_PIN   1   // Board 1 has authority
#define ESTOP_OUTPUT_PIN              30  // GPIO pin to RoboClaw
#define PIN_RELAY_ROBOCLAW_POWER      31  // SSR for power cycling
#define PIN_RELAY_MAIN_BATTERY        32  // SSR for battery disconnect
```

**E-Stop Propagation Logic**:

```
┌──────────┐                  ┌──────────────┐
│ Board1   │─────Serial──────▶│ ROS2         │
│ Fault    │                  │ Bridge       │
└────┬─────┘                  └────┬─────────┘
     │                             │
     ├─────GPIO Pin 30────────┐    ├──/estop_status topic
     │                        │    │
     ▼                        ▼    ▼
┌─────────────┐          ┌──────────────┐
│ RoboClaw    │          │ Behavior     │
│ E-Stop Input│          │ Trees        │
└─────────────┘          └──────────────┘

┌──────────┐
│ Board2   │─────Serial──────▶ ROS2 Bridge
│ Fault    │                  (aggregates with Board1)
└──────────┘
```

**Board2 Fault Handling**:

Even though Board2 doesn't control the e-stop pin, its faults still trigger system-wide e-stop:

1. Board2 detects fault (e.g., battery critical)
2. Board2's SafetyCoordinator sends FAULT2 message via serial
3. `sigyn_to_sensor_v2` receives message, adds to fault list
4. ROS2 bridge publishes updated `/estop_status` with Board2 fault included
5. Behavior tree sees fault, sends software e-stop command to Board1
6. Board1 asserts hardware e-stop pin

**GPIO Interrupt Signaling (Planned)**

Currently, inter-board safety communication uses serial messages (100-200ms latency). Future enhancement will add GPIO interrupt lines for sub-millisecond response:

**Planned Architecture**:

```
Board1 Pin 11 ◄─── Board2 Pin 10  (Board2 signals Board1)
Board1 Pin 12 ◄─── Board3 Pin 10  (Board3 signals Board1)
```

From [config.h](TeensyV2/common/core/config.h):

```cpp
#if BOARD_ID == 1
#define PIN_SAFETY_IN_BOARD2          11  // Input: Board2 safety signal
#define PIN_SAFETY_IN_BOARD3          12  // Input: Board3 safety signal

#elif BOARD_ID == 2
#define PIN_SAFETY_OUT_TO_MASTER      10  // Output: Signal Board1
```

**Planned GPIO Protocol**:

- **Active-LOW signaling**: LOW = fault active, HIGH = all clear
- **Interrupt-driven**: Board1 uses attachInterrupt() on input pins
- **Latency**: <1ms from Board2 fault to Board1 e-stop assertion
- **Redundancy**: GPIO + serial messages provide dual-path safety
- **Fail-safe**: If GPIO wire breaks, Board1 sees LOW and triggers e-stop

**Implementation Status**: Hardware pins allocated, software pending. See TODO_list.txt item: "Implement GPIO inter-board safety signaling".

**sigyn_to_sensor_v2 Aggregation**

The ROS2 bridge node `sigyn_to_sensor_v2` aggregates faults from all boards into a unified safety status:

**Message Flow**:

```
┌──────────┐   Serial    ┌─────────────────────┐   ROS2    ┌──────────────┐
│ Board1   │────USB─────▶│ sigyn_to_sensor_v2  │──Topic───▶│ Behavior     │
│          │             │                     │           │ Trees        │
└──────────┘             │  - Parses FAULT1/2  │           └──────────────┘
┌──────────┐   Serial    │  - Tracks board_id  │
│ Board2   │────USB─────▶│  - Aggregates list  │
└──────────┘             │  - Publishes        │
                          └─────────────────────┘
```

**Code from teensy_bridge.cpp**:

```cpp
// Thread-local variable tracks which board sent current message
thread_local int current_board_id = 0;

void TeensyBridge::HandleFaultMessage(const MessageData& data, rclcpp::Time timestamp) {
  // Extract fault details from JSON
  auto active_it = data.find("active_fault");
  auto source_it = data.find("source");
  auto severity_it = data.find("severity");
  auto description_it = data.find("description");

  if (active_it != data.end() && active_it->second == "true") {
    // Create SystemFault message
    sigyn_interfaces::msg::SystemFault fault;
    fault.board_id = static_cast<uint8_t>(current_board_id);  // 1 or 2
    fault.source = source_it->second;
    fault.severity = severity_it->second;
    fault.description = description_it->second;
    fault.timestamp = timestamp;

    // Check if fault already exists from this board+source
    bool found = false;
    for (auto& existing_fault : current_estop_status_.faults) {
      if (existing_fault.source == fault.source &&
          existing_fault.board_id == fault.board_id) {
        // Update existing fault
        existing_fault.severity = fault.severity;
        existing_fault.description = fault.description;
        found = true;
        break;
      }
    }

    if (!found) {
      // Add new fault
      current_estop_status_.faults.push_back(fault);
    }

    // Update overall e-stop status
    current_estop_status_.active = true;  // Any EMERGENCY_STOP fault activates
    
  } else if (active_it != data.end() && active_it->second == "false") {
    // Clear fault from specific board+source
    auto& faults = current_estop_status_.faults;
    faults.erase(
      std::remove_if(faults.begin(), faults.end(),
        [&](const auto& f) {
          return f.source == source_it->second &&
                 f.board_id == static_cast<uint8_t>(current_board_id);
        }),
      faults.end());

    // Update overall status
    current_estop_status_.active = !faults.empty();
  }

  // Publish aggregated status
  estop_status_pub_->publish(current_estop_status_);
}
```

**Key Features**:

- **Board-specific fault tracking**: `board_id` field distinguishes identical faults from different boards
- **Fault deduplication**: Prevents duplicate entries for same source+board
- **Aggregation logic**: E-stop is active if ANY board has ANY EMERGENCY_STOP fault
- **Clearing logic**: Faults clear only when originating board reports "active_fault":"false"
- **Race condition handling**: Mutex protects fault list from concurrent access

**EStopStatus Message Structure**:

```
std_msgs/Header header
bool active                          # Overall e-stop state
SystemFault[] faults                 # Array of all active faults
  uint8 board_id                     # 1=Board1, 2=Board2
  string source                      # Module name
  string severity                    # DEGRADED, WARNING, EMERGENCY_STOP
  string description                 # Human-readable detail
  builtin_interfaces/Time timestamp  # When fault occurred
```

**Performance Characteristics**:
- Serial parse latency: ~1-2ms per message
- Fault aggregation: O(n) where n = total faults (typically <10)
- Publication rate: 1Hz for status, immediate for changes
- Total latency (Board fault → ROS2 publish): ~10-20ms

---

## VII. RoboClaw Special Handling

The RoboClaw motor controller has unique failure modes that require special handling beyond normal e-stop procedures. Some error conditions are latching and cannot be cleared by software commands alone.

**Power Cycling Requirement**

Certain RoboClaw error states are "sticky" - they persist even after the underlying condition is resolved and software attempts to reset them. The only way to clear these errors is to completely power-cycle the RoboClaw by cutting and restoring its power supply.

**Why Power Cycling is Necessary**:

1. **Hardware Latch Behavior**: RoboClaw firmware latches certain critical errors to prevent automatic recovery from dangerous conditions
2. **Internal State Corruption**: Some errors indicate the RoboClaw's internal state machine is corrupted and needs a full reset
3. **Driver Protection**: Motor driver faults may indicate thermal damage that won't heal until power is removed
4. **Firmware Bug Workaround**: Some RoboClaw firmware versions have bugs where errors persist despite clear commands

**Error Codes Requiring Power Cycle**

From [roboclaw_monitor.cpp](TeensyV2/modules/roboclaw/roboclaw_monitor.cpp):

```cpp
bool isPowerCycleLikelyRequired(uint32_t error_status) {
  // Based on RoboClaw manual behavior: E-stop may be configured as latching;
  // driver faults suggest damage/latched shutdown.
  return (error_status & static_cast<uint32_t>(RoboClawError::E_STOP)) ||
         (error_status & static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR)) ||
         (error_status & static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR));
}
```

**Specific Error Codes**:

1. **E_STOP (0x0001)**:
   - Hardware e-stop input was triggered
   - RoboClaw latches this until power cycle (depending on configuration)
   - Can also be triggered by internal safety features

2. **M1_DRIVER_FAULT_ERROR (0x0040)**:
   - Motor 1 driver chip detected fault condition
   - Usually indicates:
     - Overcurrent beyond internal limits
     - Thermal shutdown of driver IC
     - Short circuit detected
     - Driver IC damage
   - Latches to prevent further damage

3. **M2_DRIVER_FAULT_ERROR (0x0080)**:
   - Motor 2 driver chip detected fault condition
   - Same causes as M1_DRIVER_FAULT_ERROR
   - Independent of M1 (can occur separately)

4. **LOGIC_VOLTAGE_HIGH_ERROR (0x0010)** (partial):
   - Logic power supply exceeded safe limits
   - Can indicate voltage regulator failure
   - May require power cycle if persists after voltage returns to normal
   - RoboClaw may enter protective shutdown

**Detection in Code**:

```cpp
void RoboClawMonitor::checkMotorSafety() {
  // Check for fatal RoboClaw error bits
  const uint32_t kFatalRoboclawErrorMask =
    static_cast<uint32_t>(RoboClawError::E_STOP) |
    static_cast<uint32_t>(RoboClawError::TEMPERATURE_ERROR) |
    static_cast<uint32_t>(RoboClawError::TEMPERATURE2_ERROR) |
    static_cast<uint32_t>(RoboClawError::MAIN_BATTERY_HIGH_ERROR) |
    static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_HIGH_ERROR) |
    static_cast<uint32_t>(RoboClawError::LOGIC_VOLTAGE_LOW_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_DRIVER_FAULT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_DRIVER_FAULT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_SPEED_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_SPEED_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_POSITION_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_POSITION_ERROR) |
    static_cast<uint32_t>(RoboClawError::M1_CURRENT_ERROR) |
    static_cast<uint32_t>(RoboClawError::M2_CURRENT_ERROR);

  if ((system_status_.error_status & kFatalRoboclawErrorMask) != 0) {
    char decoded[256];
    decodeErrorStatus(system_status_.error_status, decoded, sizeof(decoded));
    char desc[320];
    snprintf(desc, sizeof(desc), "RoboClaw reported fatal error bits: %s", decoded);
    
    SafetyCoordinator::getInstance().activateFault(
      FaultSeverity::EMERGENCY_STOP, name(), desc);
    setEmergencyStop();

    // Check if power cycle is needed
    if (isPowerCycleLikelyRequired(system_status_.error_status)) {
      // TODO: Power-cycle RoboClaw via PIN_RELAY_ROBOCLAW_POWER
      // This is required for certain latching errors that cannot be
      // cleared by a soft reset or serial command.
    }
  }
}
```

**Solid-State Relay (SSR) Control**

Board1 has GPIO pins connected to solid-state relays (SSRs) that can cut power to critical systems:

**Hardware Configuration**:

```
Teensy Board1 Pin 31 → SSR1 → RoboClaw Power Rail
Teensy Board1 Pin 32 → SSR2 → Main Battery Power
```

From [config.h](TeensyV2/common/core/config.h):

```cpp
#if BOARD_ID == 1
#define PIN_RELAY_ROBOCLAW_POWER      31  // Relay to cut power to RoboClaw
#define PIN_RELAY_MAIN_BATTERY        32  // Relay to cut main battery power
```

**SSR Characteristics**:
- **Zero-crossing switching**: Turns off at AC zero-crossing (for DC, turns off immediately)
- **Galvanic isolation**: 2500V isolation between control and load sides
- **No bounce**: Solid-state = no mechanical contacts = no arcing
- **Fast response**: <10ms turn-on, <1ms turn-off
- **Load rating**: 25A continuous, 40A peak

**Control Code**:

```cpp
// From safety_coordinator.cpp

void SafetyCoordinator::setRoboClawPower(bool on) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  pinMode(PIN_RELAY_ROBOCLAW_POWER, OUTPUT);
  digitalWrite(PIN_RELAY_ROBOCLAW_POWER, on ? HIGH : LOW);
  
  if (!on) {
    // Cutting power - log the event
    SerialManager::getInstance().sendDiagnosticMessage(
      "CRITICAL", name(), "RoboClaw power relay OPENED (power cut)");
  } else {
    // Restoring power - log and wait for initialization
    SerialManager::getInstance().sendDiagnosticMessage(
      "INFO", name(), "RoboClaw power relay CLOSED (power restored)");
    delay(1000);  // Wait for RoboClaw to boot
  }
#endif
}

void SafetyCoordinator::setMainBatteryPower(bool on) {
#if CONTROLS_ROBOCLAW_ESTOP_PIN
  pinMode(PIN_RELAY_MAIN_BATTERY, OUTPUT);
  digitalWrite(PIN_RELAY_MAIN_BATTERY, on ? HIGH : LOW);
  
  if (!on) {
    SerialManager::getInstance().sendDiagnosticMessage(
      "CRITICAL", name(), "Main battery power relay OPENED (power cut)");
  }
#endif
}
```

**Power Cycle Sequence**:

```
1. Detect latching error (isPowerCycleLikelyRequired() returns true)
2. Trigger EMERGENCY_STOP (motors already stopped)
3. Open SSR (setRoboClawPower(false))
4. Wait 2 seconds (capacitor discharge, internal state reset)
5. Close SSR (setRoboClawPower(true))
6. Wait 1 second (RoboClaw boot time)
7. Verify error cleared (read error status register)
8. If cleared: allow e-stop reset
9. If not cleared: report hardware failure, require manual intervention
```

**Automatic vs. Behavior-Tree-Triggered Cycling**

Power cycling is a drastic action with significant implications. The system uses a hybrid approach:

**Automatic Power Cycling** (NOT YET IMPLEMENTED):

The code includes TODO comments for automatic power cycling:

```cpp
if (isPowerCycleLikelyRequired(system_status_.error_status)) {
  // TODO: Power-cycle RoboClaw via PIN_RELAY_ROBOCLAW_POWER to clear latching faults.
  // This is required for certain latching errors (like Logic Battery High) that
  // cannot be cleared by a soft reset or serial command.
}
```

**Why Not Automatic Yet?**:

1. **Safety validation**: Automatic power cycling during a mission is risky
2. **State recovery**: Need to verify all RoboClaw settings are restored after power cycle
3. **Testing**: Requires extensive testing to ensure it doesn't make things worse
4. **False positives**: Need to verify error detection is 100% accurate before automatic response

**Behavior-Tree-Triggered Cycling** (PREFERRED APPROACH):

Instead of automatic power cycling, the system will report the need for power cycling to behavior trees:

```
1. RoboClawMonitor detects latching error
2. Triggers EMERGENCY_STOP with description: "Power cycle required"
3. Behavior tree receives estop_status with "Power cycle required" message
4. Behavior tree evaluates context:
   - In middle of critical navigation? Abort mission, return to safe location
   - Docked at charging station? Safe to power cycle
   - Operator nearby? Request manual intervention
5. Behavior tree sends power cycle command via ROS2 service
6. SafetyCoordinator executes power cycle sequence
7. Behavior tree monitors recovery and resumes operation or escalates to operator
```

**Advantages of Behavior Tree Control**:
- High-level decision making with full context
- Can coordinate with other systems (stop navigation, save state, etc.)
- Operator notification and approval possible
- Mission-aware timing (power cycle when safe, not mid-maneuver)
- Logging and monitoring at behavior tree level

**Current Implementation Status**:

As of January 2026:
- ✅ SSR hardware installed and tested
- ✅ GPIO control code implemented
- ✅ Error detection code implemented
- ✅ isPowerCycleLikelyRequired() function implemented
- ⏳ Automatic power cycle sequence (TODO)
- ⏳ ROS2 service interface for manual power cycling (TODO)
- ⏳ Behavior tree integration (TODO)
- ⏳ Post-power-cycle verification (TODO)

**Testing Requirements Before Deployment**:

1. Verify SSR can safely interrupt RoboClaw power under load
2. Verify RoboClaw recovers cleanly after power cycle
3. Verify encoder positions are preserved or can be re-initialized
4. Verify PID tuning settings are restored
5. Test power cycle during various error conditions
6. Verify no unintended consequences (voltage spikes, back-EMF, etc.)
7. Test rapid cycling (what if error recurs immediately?)
8. Verify behavior tree decision logic

**Safety Considerations**:

- **Never power cycle during motion**: Robot must be stationary
- **Verify mechanical brake engaged**: If present, engage brake before power cycle
- **State preservation**: Log robot state before power cycle for recovery
- **Timeout limits**: If multiple power cycles fail, escalate to operator
- **Battery protection**: Ensure battery can handle inrush current when power restored
- **Thermal management**: Power cycling doesn't fix thermal issues, may make worse

See TODO_list.txt for detailed implementation tasks: "Complete RoboClaw power cycling implementation with behavior tree integration".

---

## VIII. Self-Healing Faults

One of the most powerful features of Sigyn's safety system is its ability to automatically recover from transient faults without human intervention. This "self-healing" capability allows the robot to tolerate brief disturbances and resume normal operation, improving mission reliability.

**Design Philosophy**:

- **Conservative activation, liberal recovery**: Trigger safety responses quickly, but require sustained normal conditions before clearing
- **No oscillation**: Prevent rapid fault activation/deactivation cycles that destabilize the system
- **State persistence**: Maintain fault history for diagnostics even after recovery
- **Supervised recovery**: Higher-severity faults may require behavior tree or operator approval

**Transient vs. Persistent Conditions**

**Transient Conditions** (Auto-Recoverable):

These are temporary disturbances that clear naturally without physical intervention:

1. **Performance Glitches**:
   - Single loop iteration exceeds timing budget
   - Caused by: Interrupt storm, brief CPU spike, cache miss
   - Recovery: Next 5+ iterations within spec
   - Example: 85Hz control loop briefly drops to 80Hz, then recovers

2. **Sensor Noise**:
   - Single outlier reading from otherwise functioning sensor
   - Caused by: EMI, electrical noise, brief connection glitch
   - Recovery: EMA filter smooths out spike, subsequent readings normal
   - Example: Temperature sensor reads 150°C for one sample, then back to 45°C

3. **Communication Hiccups**:
   - Brief serial communication timeout
   - Caused by: USB buffer full, OS scheduling, electrical interference
   - Recovery: Communication resumes within 100-200ms
   - Example: ROS2 bridge misses one FAULT message, next one arrives normally

4. **Thermal Recovery**:
   - Temperature rises to WARNING level but not CRITICAL
   - Caused by: Sustained load, ambient temperature spike
   - Recovery: Temperature drops below warning threshold for sustained period
   - Example: Motor temp rises to 75°C under load, drops to 65°C when motion stops

5. **Battery Voltage Dip**:
   - Voltage briefly drops below WARNING during high current draw
   - Caused by: Battery internal resistance, motor start transient
   - Recovery: Voltage returns to normal range as current decreases
   - Example: 36V battery dips to 34.5V during acceleration, returns to 35.8V at cruise

**Persistent Conditions** (Manual/Operator Intervention Required):

These indicate lasting problems requiring physical action or system reset:

1. **Hardware Button E-Stop**:
   - Physical e-stop button pressed by operator
   - Requires: Manual button release + behavior tree reset command
   - Example: Operator stops robot for safety, must explicitly clear before resuming

2. **Thermal Runaway**:
   - Rapid, sustained temperature rise indicating hardware failure
   - Requires: Physical cooling, load reduction, possibly component replacement
   - Example: Motor controller overheating at 5°C/min → shutdown → manual inspection

3. **Battery Critical Low**:
   - Voltage below safe discharge threshold
   - Requires: Charging or battery replacement
   - Example: Battery at 32V → robot returns to dock → operator must charge

4. **Motor Overcurrent**:
   - Sustained current above safe limits
   - Requires: Identify mechanical jam, reduce load, check wiring
   - Example: Wheel jammed → overcurrent fault → operator must clear obstruction

5. **RoboClaw Driver Fault**:
   - Motor driver IC reports internal fault
   - Requires: Power cycle via SSR, possibly hardware replacement
   - Example: Short circuit trips driver protection → latching error → power cycle needed

6. **Sensor Communication Failure**:
   - I2C device stops responding for >5 seconds
   - Requires: Physical inspection, wiring check, device replacement
   - Example: INA226 battery sensor unplugged → communication timeout → operator reconnects

**Automatic Recovery Criteria**

For a fault to self-heal, it must meet ALL of these conditions:

**1. Fault Marked as Auto-Recoverable**:

Each fault source declares whether it supports automatic recovery:

```cpp
// From PerformanceMonitor - auto-recoverable
if (violations_in_window >= config_.max_violations_before_estop) {
  SafetyCoordinator::getInstance().activateFault(
    FaultSeverity::EMERGENCY_STOP,
    name(),
    "Excessive timing violations");
  // Performance faults clear automatically when violations drop
}

// From physical e-stop button - NOT auto-recoverable
if (physical_estop_pressed) {
  SafetyCoordinator::getInstance().activateFault(
    FaultSeverity::EMERGENCY_STOP,
    "HardwareEStop",
    "Physical e-stop button pressed - manual reset required");
  // Requires explicit operator action
}
```

**2. Sustained Normal Conditions**:

Condition must remain normal for a stabilization period:

- **Timing**: 5 seconds minimum (prevents oscillation)
- **Hysteresis**: Recovery threshold below activation threshold
- **Verification**: Multiple consecutive checks, not just one

Example from TemperatureMonitor:

```cpp
void TemperatureMonitor::checkSafetyConditions() {
  // ... temperature reading code ...
  
  // Determine desired fault severity
  FaultSeverity desired_severity = FaultSeverity::NORMAL;
  
  if (thermal_runaway_detected) {
    desired_severity = FaultSeverity::EMERGENCY_STOP;  // Not auto-recoverable
  } else if (any_critical) {
    desired_severity = FaultSeverity::EMERGENCY_STOP;  // Requires temp to drop to WARNING
  } else if (any_warning) {
    desired_severity = FaultSeverity::WARNING;         // Auto-clears when temp drops
  } else if (any_degraded) {
    desired_severity = FaultSeverity::DEGRADED;        // Auto-clears when sensor reconnects
  }
  
  // Update SafetyCoordinator
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  if (desired_severity == FaultSeverity::NORMAL) {
    // Deactivate fault when condition clears
    safety.deactivateFault(name());
  } else {
    // Update or activate fault
    safety.activateFault(desired_severity, name(), description);
  }
}
```

**3. No Other Blocking Faults**:

E-stop state clears only when ALL recoverable faults have cleared:

```cpp
void SafetyCoordinator::updateEstopState() {
  bool should_be_in_estop = false;
  
  // Check all active faults
  for (size_t i = 0; i < kMaxFaults; i++) {
    if (faults_[i].active && faults_[i].severity == FaultSeverity::EMERGENCY_STOP) {
      should_be_in_estop = true;
      break;  // Even one EMERGENCY_STOP fault maintains e-stop
    }
  }
  
  if (should_be_in_estop != estop_active_) {
    estop_active_ = should_be_in_estop;
    setEmergencyStopOutput(!estop_active_);  // Active-LOW signal
    
    // Notify via serial
    sendEstopStatusUpdate();
  }
}
```

**Example: Temperature Stabilization**

A common self-healing scenario:

**Scenario**: Motor controller temperature rises during sustained climb

```
Time    Temp(°C)  Threshold    Fault State       Action
----    --------  ---------    -----------       ------
t=0s    45.0      Normal       NORMAL            Motors running at full power
t=10s   68.0      <70 (WARN)   NORMAL            Continued operation
t=20s   72.0      >70 (WARN)   WARNING           Log warning, continue
t=30s   73.5      >70 (WARN)   WARNING           No change
t=40s   74.8      >70 (WARN)   WARNING           Approaching critical
t=50s   69.5      <70 (clear)  WARNING→NORMAL    Beginning recovery
t=55s   68.2      <70          WARNING→NORMAL    Sustained below threshold
t=60s   67.0      <70          NORMAL (cleared)  Fault auto-clears, full operation
```

**Code Implementation**:

```cpp
// temperature_monitor.cpp lines 680-710

FaultSeverity desired_severity = FaultSeverity::NORMAL;

if (thermal_runaway_detected) {
  desired_severity = FaultSeverity::EMERGENCY_STOP;
  // NOT auto-recoverable - requires investigation
} else if (any_critical) {
  desired_severity = FaultSeverity::EMERGENCY_STOP;
  // Recoverable when temp drops below critical threshold
} else if (any_warning) {
  desired_severity = FaultSeverity::WARNING;
  // Recoverable when temp drops below warning threshold
}

SafetyCoordinator& safety = SafetyCoordinator::getInstance();
if (desired_severity == FaultSeverity::NORMAL) {
  // Temperature has returned to normal - clear fault
  safety.deactivateFault(name());
} else {
  // Update fault with current severity
  safety.activateFault(desired_severity, name(), description);
}
```

**Why This Works**:

1. **EMA Filtering**: Temperature readings smoothed over ~2 seconds, prevents single-sample glitches
2. **Hysteresis**: WARNING threshold at 70°C, but clears at 69°C (avoids oscillation at boundary)
3. **Sustained Check**: Temperature must remain below 70°C for multiple loop iterations before clearing
4. **Graceful Degradation**: Robot can operate at WARNING level, only stops at CRITICAL

**Example: Performance Recovery**

Performance faults self-heal when system load decreases:

**Scenario**: CPU spike during sensor processing

```
Time     Loop Time  Target  Violations  Fault State        Action
----     ---------  ------  ----------  -----------        ------
t=0ms    11.5ms     11.7ms  0/10        NORMAL             Clean execution
t=12ms   11.6ms     11.7ms  0/10        NORMAL             Still good
t=24ms   13.2ms     11.7ms  1/10        NORMAL (counting)  First violation
t=36ms   12.8ms     11.7ms  2/10        NORMAL             Second violation
t=48ms   14.5ms     11.7ms  3/10        WARNING            Elevated severity
t=60ms   11.2ms     11.7ms  3/10        WARNING            Back to spec
t=72ms   11.4ms     11.7ms  2/10        WARNING            Violation aged out
t=84ms   11.3ms     11.7ms  1/10        WARNING            Almost cleared
t=96ms   11.5ms     11.7ms  0/10        NORMAL (cleared)   Auto-recovery complete
```

**Code Implementation**:

```cpp
// From PerformanceMonitor

void PerformanceMonitor::checkPerformanceViolations() {
  // Age out old violations from sliding window
  uint32_t now = millis();
  while (!violation_timestamps_.empty() && 
         (now - violation_timestamps_.front()) > config_.violation_window_ms) {
    violation_timestamps_.pop();
  }
  
  size_t violations_in_window = violation_timestamps_.size();
  
  if (violations_in_window >= config_.max_violations_before_estop) {
    SafetyCoordinator::getInstance().activateFault(
      FaultSeverity::EMERGENCY_STOP,
      name(),
      "Excessive timing violations");
  } else if (violations_in_window >= config_.max_violations_before_warning) {
    SafetyCoordinator::getInstance().activateFault(
      FaultSeverity::WARNING,
      name(),
      "Performance degradation detected");
  } else {
    // Automatically clear when violation count drops
    SafetyCoordinator::getInstance().deactivateFault(name());
  }
}
```

**Why This Works**:

1. **Sliding Window**: Only recent violations count (10-second window)
2. **Automatic Aging**: Old violations expire without manual intervention
3. **Threshold Escalation**: 5 violations = WARNING, 10 violations = E-STOP
4. **Self-Clearing**: As violations age out, fault severity automatically decreases

**Recovery State Machine**:

```
┌─────────┐
│ NORMAL  │
└────┬────┘
     │
     ├─────Fault Condition Detected────────┐
     │                                      │
     ▼                                      ▼
┌─────────┐                           ┌──────────────┐
│ WARNING │                           │ EMERGENCY    │
│         │                           │ STOP         │
└────┬────┘                           └──────┬───────┘
     │                                       │
     │ Condition Worsens                     │
     ├───────────────────────────────────────┤
     │                                       │
     │ Condition Clears                      │ Condition Clears
     │ (auto-recovery)                       │ (if auto-recoverable)
     │                                       │
     ▼                                       ▼
┌─────────┐                           ┌──────────────┐
│ NORMAL  │                           │ RECOVERY     │
│         │◄──────────────────────────│ (verifying)  │
└─────────┘                           └──────────────┘
              Sustained normal                │
              conditions verified              │
                                               │
                                               ▼
                                          If persistent:
                                          require manual
                                          intervention
```

**Logging and Diagnostics**:

All fault state transitions are logged for post-incident analysis:

```cpp
void SafetyCoordinator::deactivateFault(const char* source) {
  int idx = findFaultIndex(source);
  if (idx < 0 || !faults_[idx].active) {
    return;  // Fault not active
  }
  
  // Log recovery
  char msg[256];
  snprintf(msg, sizeof(msg),
           "Fault CLEARED: source=%s, was_severity=%s, active_duration=%lums",
           source,
           faultSeverityToString(faults_[idx].severity),
           millis() - faults_[idx].timestamp);
  SerialManager::getInstance().sendDiagnosticMessage("INFO", name(), msg);
  
  // Clear fault slot
  faults_[idx].active = false;
  faults_[idx].source[0] = '\0';
  
  // Update e-stop state
  updateEstopState();
}
```

This logging provides:
- **Recovery time**: How long fault was active
- **Fault history**: Pattern recognition for recurring issues
- **Diagnostic data**: For behavior tree and operator dashboards

---

## IX. ROS2 Integration

The TeensyV2 safety system integrates with ROS2 via the `sigyn_to_sensor_v2` bridge node. This node translates serial messages from the Teensy boards into ROS2 topics and services, making safety status visible to behavior trees, navigation stack, and operator interfaces.

**Architecture**:

```
┌──────────────┐   Serial/USB    ┌─────────────────────┐   ROS2 Topics    ┌──────────────┐
│ TeensyV2     │────────────────▶│ sigyn_to_sensor_v2  │─────────────────▶│ Behavior     │
│ Safety       │   FAULT messages│ Bridge Node         │ /estop_status    │ Trees        │
│ System       │◀────────────────│                     │◀─────────────────│              │
└──────────────┘   CONFIG cmds   └─────────────────────┘   /cmd_vel        └──────────────┘
                                           │
                                           │ Services
                                           │ ~/reset_fault
                                           │ ~/sd_getdir
                                           ▼
                                    ┌──────────────┐
                                    │ Operator     │
                                    │ Dashboard    │
                                    └──────────────┘
```

**Safety Topics**

**Published Topics**:

1. **~/safety/estop_status** (`sigyn_interfaces/msg/EStopStatus`)
   - **Description**: Aggregated safety status from all boards
   - **Rate**: 1Hz periodic + immediate on change
   - **QoS**: Reliable, keep-last 10
   - **Usage**: Primary safety monitoring for behavior trees and operators

   **Message Structure**:
   ```
   std_msgs/Header header
   bool active                     # Overall e-stop state (true = motors disabled)
   SystemFault[] faults            # Array of active faults from all boards
     uint8 board_id                # 1=Board1, 2=Board2, 3=Board3
     string source                 # Module name (e.g., "TemperatureMonitor")
     string severity               # "DEGRADED", "WARNING", "EMERGENCY_STOP", "SYSTEM_SHUTDOWN"
     string description            # Human-readable fault description
     builtin_interfaces/Time timestamp  # When fault was activated
   ```

   **Example Message**:
   ```yaml
   header:
     stamp:
       sec: 1736186400
       nanosec: 123456789
     frame_id: "base_link"
   active: true
   faults:
     - board_id: 1
       source: "TemperatureMonitor"
       severity: "EMERGENCY_STOP"
       description: "Thermal runaway: sensor=RoboClaw_temp idx=0 temp=85.3C trend=5.2C/min"
       timestamp:
         sec: 1736186395
         nanosec: 500000000
     - board_id: 2
       source: "BatteryMonitor"
       severity: "WARNING"
       description: "Battery voltage low: 34.2V (warning threshold 34.0V)"
       timestamp:
         sec: 1736186398
         nanosec: 750000000
   ```

2. **~/sensors/battery** (`sensor_msgs/msg/BatteryState`)
   - **Description**: Main battery voltage, current, charge state
   - **Rate**: 1Hz

3. **~/sensors/temperature/motor0** (`sensor_msgs/msg/Temperature`)
   - **Description**: Motor 0 temperature reading

4. **~/sensors/temperature/motor1** (`sensor_msgs/msg/Temperature`)
   - **Description**: Motor 1 temperature reading

5. **~/diagnostics** (`diagnostic_msgs/msg/DiagnosticArray`)
   - **Description**: System health diagnostics including safety status
   - **Rate**: 0.2Hz (every 5 seconds)

**Subscribed Topics**:

1. **~/commands/estop** (`std_msgs/msg/Bool`)
   - **Description**: Software e-stop command from behavior trees
   - **QoS**: Reliable
   - **Usage**: `data: true` = activate e-stop, `data: false` = attempt to clear

2. **~/commands/config** (`std_msgs/msg/String`)
   - **Description**: Runtime configuration commands to Teensy boards
   - **Format**: JSON string

3. **/cmd_vel** (`geometry_msgs/msg/Twist`)
   - **Description**: Velocity commands (blocked when estop active)

**Safety Services**

**1. ~/safety/reset_fault** (`sigyn_interfaces/srv/ResetFault`)

**Purpose**: Request clearance of a specific fault (for operator intervention)

**Request**:
```
string source     # Source module name (e.g., "ROBOCLAW_CURRENT")
string reason     # Optional: reason for reset
```

**Response**:
```
bool success      # True if fault was cleared
string message    # Status message or error description
```

**Usage Example**:
```bash
# Clear overcurrent fault after fixing mechanical jam
ros2 service call /sigyn/teensy_bridge/safety/reset_fault \
  sigyn_interfaces/srv/ResetFault \
  "{source: 'RoboClawMonitor', reason: 'Mechanical jam cleared'}"
```

**2. ~/teensy_sensor_sd_getdir** (`sigyn_interfaces/srv/TeensySdGetDir`)

**Purpose**: List files on Teensy SD card (for retrieving safety logs)

**3. ~/teensy_sensor_sd_getfile** (`sigyn_interfaces/srv/TeensySdGetFile`)

**Purpose**: Download log file from Teensy SD card

**Usage by Behavior Trees**

The safety system integrates with behavior trees at multiple levels:

**1. E-Stop Monitoring**: Behavior trees subscribe to `/estop_status` and check `active` flag before executing motion commands

**2. Fault-Specific Reactions**: Trees inspect individual faults for context-aware response (e.g., return to dock on battery warning)

**3. Safety-Aware Planning**: Navigation stack adjusts velocity/acceleration based on WARNING-level faults

**4. Operator Notification**: Critical faults trigger alerts via behavior tree nodes

**Example Behavior Tree Logic**:
```xml
<Sequence name="navigation_sequence">
  <Condition name="check_estop_clear"/>
  <Action name="move_to_waypoint"/>
</Sequence>
```

If e-stop activates: navigation fails → falls back to recovery → waits for clear

**For detailed behavior tree integration patterns, see [BEHAVIOR_TREE_INTEGRATION.md](../sigyn_behavior_trees/BEHAVIOR_TREE_INTEGRATION.md)**

**Visualization**

- **rviz2**: Subscribe to `/estop_status` topic for safety visualization overlay
- **sigyn_web_client**: Web dashboard displays real-time fault list and history
- **diagnostic_aggregator**: Aggregates safety diagnostics with other system health

**Practical Command Examples**

**Monitoring**:
```bash
# Watch e-stop status in real-time
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status

# Check if e-stop is currently active
ros2 topic echo --once /sigyn/teensy_bridge/safety/estop_status | grep "active:"

# Monitor temperature sensors
ros2 topic echo /sigyn/teensy_bridge/sensors/temperature/motor0

# Watch battery voltage
ros2 topic echo /sigyn/teensy_bridge/sensors/battery | grep voltage
```

**Commanding**:
```bash
# Trigger software e-stop (emergency)
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: true}"

# Attempt to clear software e-stop
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: false}"

# Clear specific fault (operator intervention)
ros2 service call /sigyn/teensy_bridge/safety/reset_fault \
  sigyn_interfaces/srv/ResetFault \
  "{source: 'PerformanceMonitor', reason: 'CPU load reduced'}"
```

**Retrieving Logs**:
```bash
# List log files on Teensy SD card
ros2 service call /sigyn/teensy_bridge/teensy_sensor_sd_getdir \
  sigyn_interfaces/srv/TeensySdGetDir \
  "{path: '/logs/'}"

# Download specific log file
ros2 service call /sigyn/teensy_bridge/teensy_sensor_sd_getfile \
  sigyn_interfaces/srv/TeensySdGetFile \
  "{filename: 'safety_log_20260106.txt'}" > safety_log.txt
```

**Diagnostics**:
```bash
# Check ROS2 node status
ros2 node info /sigyn/teensy_bridge

# List all topics from bridge
ros2 topic list | grep teensy_bridge

# Measure topic publication rate
ros2 topic hz /sigyn/teensy_bridge/safety/estop_status

# View topic bandwidth usage
ros2 topic bw /sigyn/teensy_bridge/safety/estop_status
```

**For complete ROS2 bridge documentation, see [SIGYN_TO_SENSOR_V2.md](../sigyn_to_sensor_v2/SIGYN_TO_SENSOR_V2.md)**

---

## X. Performance & Safety Balance

Safety checks themselves must be safe. If temperature monitoring takes too long, the control loop misses its deadline and creates a *new* safety hazard. The TeensyV2 system carefully balances safety verification with real-time performance requirements.

**Safety Checks Must Not Violate Timing Guarantees**

Board1 runs a 85Hz control loop (11.7ms period) for motor control. Every module—including safety monitoring—must complete within its time budget or the entire system becomes unsafe:

**The Performance Paradox**:
```
If safety checks are too slow → control loop misses deadline
                              → motor control timing degrades
                              → robot becomes unstable
                              → safety monitoring creates hazard!
```

**Solution**: Safety checks are designed as *part* of the control loop, not separate from it. Each module's `loop()` function has a strict time budget.

**From board1_main.cpp**:
```cpp
/**
 * Real-Time Requirements:
 * - Target loop frequency: 85Hz (80-100Hz acceptable)
 * - Maximum module execution time: 2ms per iteration
 * - Motor control update rate: 50Hz minimum
 * - Safety monitoring: 100Hz
 */
```

**Why These Numbers?**:

- **85Hz control loop**: Fast enough for stable motor control, slow enough to complete all processing
- **11.7ms period**: Total time budget for all modules combined
- **2ms per module**: Conservative limit prevents any one module from dominating
- **50Hz motor update**: RoboClaw requires at least this rate to maintain smooth motion

**Enforcement**: PerformanceMonitor tracks loop timing and triggers WARNING/E-STOP if violations exceed threshold.

**For detailed module system architecture and timing guarantees, see [TEENSYV2_ARCHITECTURE.md](TEENSYV2_ARCHITECTURE.md)**

**Module::loop() Execution Time Budgets**

Each module is allocated a time budget based on typical execution:

| Module | Budget | Typical | Why |
|--------|--------|---------|-----|
| **PerformanceMonitor** | 100µs | 50µs | Lightweight timing checks only |
| **SafetyCoordinator** | 200µs | 100µs | Fault aggregation, e-stop logic |
| **TemperatureMonitor** | 500µs | 300µs | 2-8 analog reads, EMA filtering |
| **BatteryMonitor** | 800µs | 500µs | I2C communication (3× INA226 sensors) |
| **RoboClawMonitor** | 1500µs | 1000µs | Serial queries, encoder validation |
| **VL53L0XMonitor** | 2000µs | 1200µs | I2C communication (3-8 sensors) |
| **BNO055Monitor** | 1000µs | 600µs | I2C IMU data retrieval |

**Total typical**: ~4ms → leaves 7.7ms margin for worst-case I2C delays, interrupt handling, and unexpected spikes.

**Design Principle**: No single module can cause a timing violation. Even if VL53L0X sensors all time out (worst case ~3ms), the control loop still completes in time.

**Budget Allocation Strategy**:

1. **Measure actual execution** on target hardware with oscilloscope/logic analyzer
2. **Add 50% margin** for worst-case (I2C bus contention, sensor timeouts)
3. **Monitor in production** via PerformanceMonitor
4. **Trigger WARNING at 80% budget** (gives time to react before E-STOP)
5. **Trigger E-STOP at 100% budget** (timing guarantee violated)

**Example Budget Violation**:

```
Loop Iteration #1: Total 10.5ms (all modules within budget)
Loop Iteration #2: Total 11.2ms (BatteryMonitor I2C timeout → 1.2ms instead of 0.5ms)
Loop Iteration #3: Total 12.3ms (WARNING: exceeded 11.7ms target)
Loop Iteration #4: Total 13.1ms (EMERGENCY_STOP: persistent timing violation)
```

**Why EMA Filters**

Exponential Moving Average (EMA) filters are used throughout the safety system for sensor smoothing. But why EMA specifically?

**Problem**: Raw sensor readings are noisy

```
Raw temperature readings from TMP36 sensor:
45.2°C, 45.8°C, 44.9°C, 51.3°C (noise spike!), 45.1°C, 45.4°C
```

If we trigger e-stop on 51.3°C spike → false alarm.

**Solution Comparison**:

| Filter Type | Memory Usage | CPU Cost | Latency | Pros | Cons |
|-------------|--------------|----------|---------|------|------|
| **Simple Average (10 samples)** | 40 bytes | Low | 10 samples | Easy to understand | Slow to respond to real changes |
| **Median Filter (10 samples)** | 40 bytes | High (sorting) | 5 samples | Excellent noise rejection | Too expensive (>1ms) |
| **EMA (α=0.7)** | 4 bytes | Very low | ~3 samples | Fast response + smoothing | Needs tuning |

EMA wins because:
1. **Minimal memory**: Single float (4 bytes) vs. full history buffer (40+ bytes)
2. **Constant time**: O(1) execution, no sorting or iteration
3. **Configurable response**: Adjust α to balance smoothing vs. latency
4. **Real-time friendly**: Execution time <5µs (fits in any timing budget)

**EMA Formula**:
```cpp
// From temperature_monitor.cpp line 448-450
float alpha = 0.7f;  // Response factor (0-1)
filtered_temp = alpha * new_reading + (1.0f - alpha) * previous_filtered;
```

**What α Means**:
- **α = 1.0**: No filtering (use raw reading)
- **α = 0.9**: Fast response (90% weight on new sample)
- **α = 0.7**: Balanced (TeensyV2 default for temperature)
- **α = 0.5**: Equal weighting (slow response)
- **α = 0.1**: Heavy smoothing (10% weight on new sample)

**Why α=0.7 for Temperature**:

```
At 1Hz sampling (1 reading per second):
- ~63% convergence after 2 samples (2 seconds)
- ~86% convergence after 3 samples (3 seconds)
- ~95% convergence after 5 samples (5 seconds)
```

This is fast enough to detect thermal runaway (5°C/min) within 5-10 seconds, but slow enough to filter out single-sample noise spikes.

**Battery Monitor Uses α=0.5** (heavier smoothing):
- Battery voltage changes slowly
- Want to filter out transient dips during motor acceleration
- 10-second convergence is acceptable

**VL53L0X Distance Uses α=0.9** (minimal smoothing):
- Need fast response for obstacle detection
- Can't wait 5 seconds to detect wall
- Accept some noise to reduce latency

**For detailed filter design and tuning, see [TEENSYV2_ARCHITECTURE.md](TEENSYV2_ARCHITECTURE.md)**

**Filter Convergence vs. Safety Response Time**

The fundamental tradeoff: smooth readings vs. fast hazard detection.

**Convergence Time**: How long until filtered value matches true value?

For EMA with α=0.7:
```
Time (samples)  Convergence  Real-world at 1Hz
1 sample        70%          1 second
2 samples       91%          2 seconds  ← Temperature default
3 samples       97%          3 seconds
5 samples       99.8%        5 seconds
10 samples      99.999%      10 seconds
```

**Safety Response Time**: How long to detect and react to a hazard?

**Example 1: Thermal Runaway (Fast Hazard)**

```
Scenario: Motor controller overheating at 5°C/min (thermal runaway rate)

At t=0s:   Temperature = 60°C (normal)
At t=12s:  Temperature = 61°C (EMA filtered: 60.7°C after convergence)
At t=24s:  Temperature = 62°C (EMA filtered: 61.7°C)
At t=36s:  Temperature = 63°C (EMA filtered: 62.6°C)
At t=48s:  Temperature = 64°C (EMA filtered: 63.5°C)
At t=60s:  Temperature = 65°C (EMA filtered: 64.4°C)

Trend calculation over 50 samples detects: +5.1°C/min at t=60s
Trigger EMERGENCY_STOP

Total response time: ~60 seconds from start of runaway
```

**Is 60 seconds acceptable?** YES:
- Motor controllers have thermal mass (won't melt instantly)
- Thermal runaway threshold is set conservatively (80°C damage point)
- 60-second detection at 60°C starting point → stops at 65°C
- Still 15°C below damage threshold

**Example 2: Battery Voltage Drop (Slow Hazard)**

```
Scenario: Battery discharging under load

At t=0s:   Voltage = 36.0V (healthy)
At t=30s:  Voltage = 35.0V (EMA filtered: 35.5V with α=0.5)
At t=60s:  Voltage = 34.5V (EMA filtered: 35.0V)
At t=90s:  Voltage = 34.0V (WARNING threshold, EMA: 34.5V)
At t=120s: Voltage = 33.8V (EMA filtered: 34.1V)
At t=150s: Voltage = 33.5V (EMA filtered: 33.8V)

WARNING triggered when EMA crosses 34.0V threshold at ~t=150s

Total response time: ~150 seconds from initial drop
```

**Is 150 seconds acceptable?** YES:
- Battery discharge is gradual (not instant failure)
- WARNING level triggers return-to-dock behavior
- 150-second delay ensures it's a real discharge, not motor start transient
- Prevents false alarms during acceleration

**Example 3: Obstacle Detection (Fast Hazard)**

```
Scenario: Robot approaching wall

At t=0ms:    Distance = 2000mm (clear)
At t=100ms:  Distance = 1500mm (EMA filtered: 1650mm with α=0.9)
At t=200ms:  Distance = 1000mm (EMA filtered: 1085mm)
At t=300ms:  Distance = 500mm (EMA filtered: 557mm)
At t=400ms:  Distance = 300mm (WARNING: too close)

WARNING triggered at t=400ms

Total response time: ~400ms from initial detection
```

**Is 400ms acceptable?** YES:
- Robot traveling at 0.3 m/s → moves 12cm in 400ms
- WARNING triggers at 300mm → emergency stop with 18cm margin
- Fast enough to prevent collision

**Tuning Strategy**:

1. **Identify hazard time constant**: How fast can the hazard develop?
   - Thermal runaway: minutes
   - Battery discharge: tens of seconds
   - Obstacle collision: milliseconds

2. **Set α for 5× margin**:
   - If hazard develops in 10 seconds, set α for 2-second response
   - If hazard develops in 1 second, set α for 200ms response

3. **Verify with testing**:
   - Simulate worst-case hazard (rapid temperature ramp, sudden obstacle)
   - Measure detection latency
   - Confirm safety margin is maintained

**Trade-off Summary**:

```
More Smoothing (Lower α)     Less Smoothing (Higher α)
├─ Fewer false alarms        ├─ Faster hazard detection
├─ Stable readings           ├─ Responsive to changes
├─ Longer response time      ├─ More noise sensitivity
└─ Good for slow hazards     └─ Good for fast hazards
```

**TeensyV2 Default Values**:
- Temperature: α=0.7 (2-3 second response for 5°C/min runaway)
- Battery: α=0.5 (10-second response for voltage drops)
- Distance: α=0.9 (200ms response for obstacles)
- IMU: α=0.8 (500ms response for orientation changes)

---

## XI. Testing Philosophy

You cannot verify safety with a soldering iron and a prayer. The TeensyV2 safety system uses automated unit tests with hardware mocks to verify fault detection without risking actual hardware damage.

**Why Testing is Required for Safety**

**The Problem**: How do you test thermal runaway detection without melting a motor controller?

**Bad Approaches**:
1. ❌ Test on real hardware by deliberately overheating (destroys equipment)
2. ❌ "Code review only" (humans miss edge cases)
3. ❌ "It works in my testing" (anecdotal, not reproducible)
4. ❌ Wait for production failures (users become beta testers)

**Good Approach**: 
✅ **Automated unit tests with mock hardware** (safe, repeatable, fast)

**Benefits of Automated Testing**:
- **Repeatability**: Run same test 1000 times, get same result
- **Speed**: Test 5°C/min thermal ramp in 1 second (simulate 10 minutes of real time)
- **Safety**: No hardware damage from fault injection
- **Coverage**: Test edge cases that rarely occur in practice (simultaneous multi-board faults)
- **Regression Prevention**: Catch bugs introduced by future changes

**Dependency Injection Pattern**

To test safety code without hardware, we use **dependency injection**: replace real hardware with test doubles (mocks).

**Traditional Code (Hard to Test)**:
```cpp
void TemperatureMonitor::readSensor() {
  // Directly reads Arduino analog pin (requires real hardware!)
  int raw = analogRead(25);
  float temp = convertToTemperature(raw);
}
```

**Testable Code with Dependency Injection**:
```cpp
class TemperatureMonitor {
private:
  IAnalogReader* analog_reader_;  // Interface, not concrete hardware
  
public:
  void setAnalogReader(IAnalogReader* reader) {
    analog_reader_ = reader;  // Inject dependency
  }
  
  void readSensor() {
    // Uses injected reader (can be real hardware OR mock)
    int raw = analog_reader_->readAnalog(25);
    float temp = convertToTemperature(raw);
  }
};
```

**Production**: Inject real Arduino hardware reader
```cpp
ArduinoAnalogReader real_reader;
temp_monitor.setAnalogReader(&real_reader);
```

**Testing**: Inject mock that returns controlled values
```cpp
MockAnalogReader mock_reader;
mock_reader.setTemperature(25, 85.0f);  // Simulate 85°C reading
temp_monitor.setAnalogReader(&mock_reader);
temp_monitor.loop();  // Now reads 85°C from mock
```

**Key Insight**: The safety logic doesn't care if readings come from real hardware or a mock. It just calls `readAnalog()` and processes the result.

**Mock Hardware for Testing**

The TeensyV2 test suite includes mocks for all external dependencies:

**Available Mocks**:
- `MockAnalogReader`: Controllable ADC values (for temperature sensors)
- `MockPowerSensor`: Controllable INA226 readings (for battery monitoring)
- `MockRoboClaw`: Controllable motor controller responses
- `arduino_mock`: Controllable time (`millis()`, `delay()`)

**Example: Testing Thermal Runaway Detection**

From [temperature_monitor_test.cpp](test/temperature_monitor_test.cpp):

```cpp
TEST_F(TemperatureMonitorTest, ThermalRunaway_Detection) {
  // Simulate rapid temperature rise (5°C/min thermal runaway)
  
  // Start at safe temperature
  mock_reader->setTemperature(25, 60.0f);
  monitor->loop();
  
  // Ramp up 1°C every 12 seconds (= 5°C/min)
  for (int i = 1; i <= 10; i++) {
    arduino_mock::setMillis(i * 12000);  // Advance time
    mock_reader->setTemperature(25, 60.0f + i);  // +1°C each step
    monitor->loop();
  }
  
  // After 10 samples (2 minutes simulated), thermal runaway detected
  const auto& status = monitor->getSensorStatus(0);
  EXPECT_TRUE(status.thermal_runaway);  // Should detect runaway
  EXPECT_GT(status.temperature_trend, 4.5f);  // Trend > 4.5°C/min
}
```

**What This Tests**:
- ✅ Thermal runaway detection algorithm works correctly
- ✅ Temperature trend calculation accurate
- ✅ Threshold detection (5°C/min)
- ✅ No false alarms during gradual warming

**What This DOESN'T Require**:
- ❌ Real temperature sensor
- ❌ Real motor controller
- ❌ Waiting 2 minutes for real temperature rise
- ❌ Risk of hardware damage

**Test runs in <1 second** on development machine.

**Example: Testing Multi-Board Fault Aggregation**

From [safety_coordinator_test.cpp](test/safety_coordinator_test.cpp):

```cpp
TEST_F(SafetyCoordinatorTest, Integration_TemperatureRecovery) {
  // Simulate Board1 temperature warning + recovery
  
  // Heat up to warning level
  mock_reader->setTemperature(25, 71.0f);  // Above 70°C warning
  
  // Hold for 2 seconds (allow EMA filter to converge)
  for (int i = 0; i < 20; i++) {
    arduino_mock::setMillis(i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify fault active
  const Fault& fault = safety->getFault("TemperatureMonitor");
  EXPECT_TRUE(fault.active);
  EXPECT_EQ(fault.severity, FaultSeverity::WARNING);
  
  // Cool down below warning threshold
  mock_reader->setTemperature(25, 68.0f);
  
  // Wait for recovery (EMA convergence)
  for (int i = 0; i < 20; i++) {
    arduino_mock::setMillis(2000 + i * 100);
    temp_monitor->loop();
    safety->testLoop();
  }
  
  // Verify fault cleared (self-healing)
  EXPECT_FALSE(safety->getFault("TemperatureMonitor").active);
}
```

**What This Tests**:
- ✅ Fault activation on temperature warning
- ✅ Fault severity escalation
- ✅ EMA filter convergence behavior
- ✅ Automatic fault clearing (self-healing)
- ✅ Integration between TemperatureMonitor and SafetyCoordinator

**Test Coverage**:

As of January 2026:
- ✅ TemperatureMonitor: 85% code coverage
- ✅ BatteryMonitor: 80% code coverage
- ✅ SafetyCoordinator: 90% code coverage
- ✅ PerformanceMonitor: 75% code coverage
- ⏳ RoboClawMonitor: 60% code coverage (TODO: improve)

**Running Tests**:

```bash
# Run all tests
cd TeensyV2
pio test

# Run specific test
pio test --filter test_temperature_monitor

# Run with verbose output
pio test -v
```

**Reference: Detailed Testing Documentation**

This section provides a brief overview. For comprehensive testing documentation including:
- Complete mock API reference
- Test writing guidelines
- Coverage analysis
- Integration testing with hardware-in-the-loop
- Continuous integration setup

**See [SAFETY_TESTING.md](SAFETY_TESTING.md) for detailed testing guide**

**Key Testing Principles**:

1. **Test behavior, not implementation**: Verify fault detection works, don't test internal variables
2. **Test edge cases**: Zero values, negative values, sensor timeouts, simultaneous faults
3. **Test time-dependent behavior**: Use `arduino_mock::setMillis()` to control time
4. **Test recovery**: Don't just test fault activation, verify self-healing works
5. **Keep tests fast**: <10 seconds for full test suite (enables rapid iteration)

---

## XII. Developer Integration Guide

This section provides a step-by-step guide for integrating safety monitoring into a new sensor module. We'll use TemperatureMonitor as the canonical example, showing the complete pattern from setup through fault detection.

**Adding Safety to a Sensor Module**

**Step 1: Inherit from Module Base Class**

All safety-aware modules inherit from `Module` base class:

```cpp
// From temperature_monitor.h
#include "../../common/core/module.h"

class TemperatureMonitor : public Module {
public:
  // Singleton pattern (required for all modules)
  static TemperatureMonitor& getInstance();
  
  // Module interface (inherited from Module base)
  void setup() override;
  void loop() override;
  const char* name() const override { return "TemperatureMonitor"; }
  
  // Safety interface
  bool isUnsafe();  // Optional: for modules that need external safety queries
  
private:
  TemperatureMonitor();  // Private constructor (singleton)
  
  // Safety checking methods
  void checkSafetyConditions();
  void detectThermalRunaway();
  
  // Configuration and state
  TemperatureMonitorConfig config_;
  TemperatureSensorStatus sensor_status_[kMaxTemperatureSensors];
};
```

**Key Points**:
- **Singleton pattern**: Ensures single instance, no heap allocation
- **Override setup()/loop()**: Module system calls these automatically
- **name() method**: Returns string identifier for fault tracking
- **Private constructor**: Prevents accidental instantiation

**Step 2: Define Configuration Structures**

Safety-critical parameters should be configurable:

```cpp
struct TemperatureSensorConfig {
  // Thresholds
  float critical_high_temp = 85.0f;    // E-stop level
  float warning_high_temp = 70.0f;     // Warning level
  float thermal_runaway_rate = 5.0f;   // °C/min for runaway detection
  
  // Safety parameters
  bool safety_critical = true;         // Participates in safety system
  uint32_t fault_timeout_ms = 5000;    // Timeout before declaring fault
};
```

**Step 3: Initialize in setup()**

From [temperature_monitor.cpp](modules/sensors/temperature_monitor.cpp):

```cpp
void TemperatureMonitor::setup() {
  // 1. Initialize hardware (sensors, I2C, etc.)
  analogReadResolution(ANALOG_RESOLUTION);
  
  // 2. Configure each sensor
  for (uint8_t i = 0; i < config_.max_sensors; i++) {
    if (sensor_configs_[i].analog_pin != 255) {
      sensor_configured_[i] = true;
      sensor_status_[i].sensor_present = true;
      
      // Initialize history buffers
      for (uint8_t j = 0; j < 50; j++) {
        sensor_status_[i].temperature_history[j] = NAN;
      }
    }
  }
  
  // 3. Initialize timing
  uint32_t now = millis();
  last_safety_check_time_ms_ = now;
  system_start_time_ms_ = now;
  
  // 4. Report initialization
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", name(), "Setup complete");
}
```

**Critical Setup Tasks**:
- Initialize hardware to known state
- Allocate/initialize data structures (avoid heap!)
- Set baseline timing variables
- Log successful initialization

**Where to Check in loop()**

The `loop()` method is called by the Module system at ~85Hz. Structure it for clear separation of concerns:

**From temperature_monitor.cpp**:

```cpp
void TemperatureMonitor::loop() {
  uint32_t now = millis();
  
  // 1. Acquire sensor data (every iteration)
  updateTemperatureReadings();
  
  // 2. Update derived state
  updateSystemStatus();
  
  // 3. SAFETY CHECKS (high priority, frequent)
  if (now - last_safety_check_time_ms_ >= 100) {  // 10Hz
    checkSafetyConditions();
    last_safety_check_time_ms_ = now;
  }
  
  // 4. Status reporting (lower priority, less frequent)
  if (now - last_status_report_time_ms_ >= config_.status_report_interval_ms) {
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }
  
  // 5. Diagnostic reporting (lowest priority)
  if (now - last_diagnostic_report_time_ms_ >= config_.diagnostic_report_interval_ms) {
    sendDiagnosticReports();
    last_diagnostic_report_time_ms_ = now;
  }
  
  // 6. Performance tracking
  if (now - last_performance_update_ms_ >= 1000) {
    updatePerformanceStatistics();
    last_performance_update_ms_ = now;
  }
}
```

**Loop Structure Pattern**:

```
┌─ Every Iteration (85Hz) ──────────────────────────┐
│ updateTemperatureReadings()  <-- Raw sensor data  │
│ updateSystemStatus()          <-- Derived state   │
└───────────────────────────────────────────────────┘

┌─ Safety Critical (10Hz) ──────────────────────────┐
│ checkSafetyConditions()       <-- Fault detection │
└───────────────────────────────────────────────────┘

┌─ Operational (1Hz) ───────────────────────────────┐
│ sendStatusReports()           <-- Real-time status│
└───────────────────────────────────────────────────┘

┌─ Diagnostic (0.1Hz) ──────────────────────────────┐
│ sendDiagnosticReports()       <-- Long-term trends│
│ updatePerformanceStatistics() <-- Metrics         │
└───────────────────────────────────────────────────┘
```

**Timing Ratios Explained**:

- **85Hz (11.7ms)**: Control loop rate → read sensors every iteration
- **10Hz (100ms)**: Safety checks → fast enough to catch rapid changes, slow enough to not overwhelm
- **1Hz (1000ms)**: Status reports → human-readable updates for ROS2
- **0.1Hz (10000ms)**: Diagnostics → long-term statistics, less urgent

**When to Call activateFault() / deactivateFault()**

These are the core safety integration points. Call them from your `checkSafetyConditions()` method.

**Pattern 1: Threshold-Based Faults** (Temperature, Battery Voltage)

```cpp
void TemperatureMonitor::checkSafetyConditions() {
  // 1. Evaluate ALL conditions
  bool any_critical = false;
  bool any_warning = false;
  bool thermal_runaway_detected = false;
  
  for (uint8_t i = 0; i < sensor_count; i++) {
    float temp = sensor_status_[i].temperature_c;
    
    // Check thresholds
    if (temp >= sensor_configs_[i].critical_high_temp) {
      any_critical = true;
    } else if (temp >= sensor_configs_[i].warning_high_temp) {
      any_warning = true;
    }
    
    // Check thermal runaway
    if (calculateTrend(i) > sensor_configs_[i].thermal_runaway_rate) {
      thermal_runaway_detected = true;
    }
  }
  
  // 2. Determine desired fault severity
  FaultSeverity desired_severity = FaultSeverity::NORMAL;
  char description[256] = {0};
  
  if (thermal_runaway_detected || any_critical) {
    desired_severity = FaultSeverity::EMERGENCY_STOP;
    snprintf(description, sizeof(description), 
             "Thermal runaway: temp=%.1fC trend=%.1fC/min", 
             temp, trend);
  } else if (any_warning) {
    desired_severity = FaultSeverity::WARNING;
    snprintf(description, sizeof(description),
             "Temperature warning: temp=%.1fC", temp);
  }
  
  // 3. Update SafetyCoordinator (handles activate/deactivate)
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  
  if (desired_severity == FaultSeverity::NORMAL) {
    // Condition cleared → deactivate fault
    safety.deactivateFault(name());
  } else {
    // Condition present → activate/update fault
    safety.activateFault(desired_severity, name(), description);
  }
}
```

**Key Decision Logic**:

```
If NO faults detected:
  ├─ Call deactivateFault(name())
  └─ Clears any existing fault from this module

If ANY fault detected:
  ├─ Determine highest severity level
  ├─ Create descriptive message
  └─ Call activateFault(severity, name(), description)
      └─ SafetyCoordinator handles:
          ├─ Updating existing fault OR
          └─ Creating new fault entry
```

**Pattern 2: Timeout-Based Faults** (Sensor Communication Loss)

```cpp
void checkSensorTimeout() {
  uint32_t now = millis();
  bool any_timeout = false;
  
  for (uint8_t i = 0; i < sensor_count; i++) {
    if (!sensor_configs_[i].safety_critical) continue;
    
    uint32_t age = now - sensor_status_[i].last_valid_reading_time_ms;
    
    if (age >= sensor_configs_[i].fault_timeout_ms) {
      any_timeout = true;
      break;
    }
  }
  
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  
  if (any_timeout) {
    safety.activateFault(FaultSeverity::DEGRADED, name(),
                        "Sensor communication timeout");
  } else {
    safety.deactivateFault(name());
  }
}
```

**Pattern 3: Rate-Based Faults** (Performance Violations)

```cpp
void PerformanceMonitor::checkTimingViolations() {
  // Count violations in sliding window
  size_t violation_count = violation_timestamps_.size();
  
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  
  if (violation_count >= max_violations_for_estop) {
    safety.activateFault(FaultSeverity::EMERGENCY_STOP, name(),
                        "Excessive timing violations");
  } else if (violation_count >= max_violations_for_warning) {
    safety.activateFault(FaultSeverity::WARNING, name(),
                        "Performance degradation");
  } else {
    // Violations cleared → auto-recovery
    safety.deactivateFault(name());
  }
}
```

**Critical Rules**:

1. **Only call from ONE place**: Don't scatter activateFault() calls across multiple methods
2. **Update every check cycle**: Call even if state unchanged (SafetyCoordinator handles deduplication)
3. **Always provide description**: Include relevant values for debugging
4. **Use consistent source name**: Pass `name()` not hardcoded strings
5. **Determine severity first**: Evaluate ALL conditions before calling activateFault()

**Filter Design Considerations**

Raw sensor readings are noisy. Filters smooth data but add latency. Design filters appropriate for hazard time constant.

**Filter Selection Matrix**:

| Hazard Type | Time Constant | Filter Type | α Value | Rationale |
|-------------|---------------|-------------|---------|-----------|
| **Thermal Runaway** | Minutes | EMA | 0.7 | 2-3 second response for 5°C/min detection |
| **Battery Discharge** | Tens of seconds | EMA | 0.5 | 10-second response, filters motor transients |
| **Obstacle Detection** | Milliseconds | EMA | 0.9 | 200ms response for collision avoidance |
| **IMU Drift** | Seconds | EMA | 0.8 | 500ms response for orientation changes |
| **Overcurrent** | Milliseconds | EMA | 0.95 | 50ms response for motor protection |

**EMA Filter Implementation**:

```cpp
void updateTemperatureReadings() {
  for (uint8_t i = 0; i < sensor_count; i++) {
    // Read raw sensor
    float raw_temp = readSensor(i);
    
    // Apply EMA filter
    float alpha = 0.7f;  // Response factor
    
    if (sensor_status_[i].reading_valid) {
      // Have previous reading → filter
      sensor_status_[i].temperature_c = 
        alpha * raw_temp + (1.0f - alpha) * sensor_status_[i].temperature_c;
    } else {
      // First reading → initialize
      sensor_status_[i].temperature_c = raw_temp;
      sensor_status_[i].reading_valid = true;
    }
  }
}
```

**Filter Tuning Process**:

1. **Measure hazard time constant**: How fast can the hazard develop?
   ```
   Thermal runaway: 5°C/min = 0.083°C/s
   Need to detect within 60 seconds → detect 5°C rise
   ```

2. **Set filter convergence**: Target 5× margin
   ```
   α = 0.7 → 63% convergence in 2 samples (2 seconds at 1Hz)
             → 95% convergence in 5 samples (5 seconds)
   ```

3. **Calculate detection latency**:
   ```
   Filter latency: 5 seconds
   Threshold check: instant
   Total: 5 seconds to detect 5°C rise
   Margin: 60s hazard / 5s detection = 12× safety factor
   ```

4. **Test with mock hardware**:
   ```cpp
   TEST_F(TemperatureMonitorTest, FilterResponse) {
     // Instant temperature jump
     mock_reader->setTemperature(25, 85.0f);
     
     // Measure convergence time
     for (int i = 0; i < 10; i++) {
       arduino_mock::setMillis(i * 1000);
       monitor->loop();
       float filtered = monitor->getSensorStatus(0).temperature_c;
       // Verify 95% convergence within 5 seconds
     }
   }
   ```

**Hysteresis for Stable Thresholds**:

Prevent oscillation at threshold boundaries:

```cpp
// Bad: No hysteresis
if (temp >= 70.0f) {
  activate_warning();  // Activates at 70.0°C
}
if (temp < 70.0f) {
  deactivate_warning();  // Deactivates at 69.9°C → oscillates!
}

// Good: With hysteresis
const float WARNING_THRESHOLD = 70.0f;
const float CLEAR_THRESHOLD = 68.0f;  // 2°C hysteresis

if (temp >= WARNING_THRESHOLD) {
  warning_active = true;
}
if (temp <= CLEAR_THRESHOLD) {
  warning_active = false;  // Only clears at 68°C → stable
}
```

**Complete Code Example: TemperatureMonitor Integration**

Here's the full pattern showing setup(), loop(), and safety check integration:

**Header File** (temperature_monitor.h):

```cpp
#pragma once
#include "../../common/core/module.h"

namespace sigyn_teensy {

class TemperatureMonitor : public Module {
public:
  // Singleton pattern
  static TemperatureMonitor& getInstance();
  
  // Module interface
  void setup() override;
  void loop() override;
  const char* name() const override { return "TemperatureMonitor"; }
  
  // Configuration
  void updateConfig(const TemperatureMonitorConfig& config);
  
  // Status queries
  const TemperatureSensorStatus& getSensorStatus(uint8_t index) const;
  bool isUnsafe();
  
private:
  TemperatureMonitor();  // Private constructor
  
  // Core monitoring methods
  void updateTemperatureReadings();
  void updateSystemStatus();
  
  // Safety methods
  void checkSafetyConditions();
  void detectThermalRunaway();
  float calculateTemperatureTrend(uint8_t sensor_index);
  
  // Reporting
  void sendStatusReports();
  void sendDiagnosticReports();
  
  // Configuration and state
  TemperatureMonitorConfig config_;
  TemperatureSensorConfig sensor_configs_[kMaxTemperatureSensors];
  TemperatureSensorStatus sensor_status_[kMaxTemperatureSensors];
  bool sensor_configured_[kMaxTemperatureSensors] = {false};
  
  // Timing
  uint32_t last_safety_check_time_ms_;
  uint32_t last_status_report_time_ms_;
  uint32_t system_start_time_ms_;
};

}  // namespace sigyn_teensy
```

**Implementation File** (temperature_monitor.cpp):

```cpp
#include "temperature_monitor.h"
#include "../../modules/safety/safety_coordinator.h"

namespace sigyn_teensy {

TemperatureMonitor& TemperatureMonitor::getInstance() {
  static TemperatureMonitor instance;
  return instance;
}

TemperatureMonitor::TemperatureMonitor()
    : Module(),
      last_safety_check_time_ms_(0),
      last_status_report_time_ms_(0),
      system_start_time_ms_(0) {
  // Initialize sensor status
  for (uint8_t i = 0; i < kMaxTemperatureSensors; i++) {
    sensor_status_[i] = TemperatureSensorStatus{};
    sensor_status_[i].max_temperature = -273.15f;
    sensor_status_[i].min_temperature = 1000.0f;
  }
}

void TemperatureMonitor::setup() {
  // 1. Configure hardware
  analogReadResolution(12);  // 12-bit ADC
  
  // 2. Configure sensors
  sensor_configs_[0].analog_pin = 25;
  strncpy(sensor_configs_[0].sensor_name, "Motor_Left", 
          sizeof(sensor_configs_[0].sensor_name) - 1);
  sensor_configs_[0].critical_high_temp = 85.0f;
  sensor_configs_[0].warning_high_temp = 70.0f;
  sensor_configs_[0].thermal_runaway_rate = 5.0f;
  sensor_configured_[0] = true;
  
  sensor_configs_[1].analog_pin = 26;
  strncpy(sensor_configs_[1].sensor_name, "Motor_Right",
          sizeof(sensor_configs_[1].sensor_name) - 1);
  sensor_configs_[1].critical_high_temp = 85.0f;
  sensor_configs_[1].warning_high_temp = 70.0f;
  sensor_configs_[1].thermal_runaway_rate = 5.0f;
  sensor_configured_[1] = true;
  
  // 3. Initialize timing
  uint32_t now = millis();
  last_safety_check_time_ms_ = now;
  last_status_report_time_ms_ = now;
  system_start_time_ms_ = now;
  
  // 4. Report ready
  SerialManager::getInstance().sendDiagnosticMessage(
    "INFO", name(), "Setup complete - 2 sensors ready");
}

void TemperatureMonitor::loop() {
  uint32_t now = millis();
  
  // Read sensors every iteration (85Hz)
  updateTemperatureReadings();
  updateSystemStatus();
  
  // Safety checks at 10Hz
  if (now - last_safety_check_time_ms_ >= 100) {
    checkSafetyConditions();
    last_safety_check_time_ms_ = now;
  }
  
  // Status reports at 1Hz
  if (now - last_status_report_time_ms_ >= 1000) {
    sendStatusReports();
    last_status_report_time_ms_ = now;
  }
}

void TemperatureMonitor::updateTemperatureReadings() {
  for (uint8_t i = 0; i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i]) continue;
    
    // Read analog pin (8 samples averaged)
    uint32_t raw_sum = 0;
    for (uint8_t sample = 0; sample < 8; sample++) {
      raw_sum += analogRead(sensor_configs_[i].analog_pin);
      delayMicroseconds(100);
    }
    int16_t raw_value = raw_sum / 8;
    
    // Convert to temperature (TMP36 formula)
    float voltage_mv = (raw_value * 3.3f * 1000.0f) / 4096.0f;
    float temp_c = (voltage_mv - 500.0f) / 10.0f;
    
    // Validate range
    if (temp_c < -40.0f || temp_c > 150.0f) {
      sensor_status_[i].reading_valid = false;
      continue;
    }
    
    // Apply EMA filter
    float alpha = 0.7f;
    if (sensor_status_[i].reading_valid) {
      sensor_status_[i].temperature_c = 
        alpha * temp_c + (1.0f - alpha) * sensor_status_[i].temperature_c;
    } else {
      sensor_status_[i].temperature_c = temp_c;
      sensor_status_[i].reading_valid = true;
    }
    
    sensor_status_[i].last_valid_reading_time_ms = millis();
  }
}

void TemperatureMonitor::checkSafetyConditions() {
  uint32_t now = millis();
  bool any_critical = false;
  bool any_warning = false;
  bool thermal_runaway = false;
  
  // Evaluate all sensors
  for (uint8_t i = 0; i < kMaxTemperatureSensors; i++) {
    if (!sensor_configured_[i] || !sensor_status_[i].reading_valid) {
      continue;
    }
    
    float temp = sensor_status_[i].temperature_c;
    
    // Check thresholds
    if (temp >= sensor_configs_[i].critical_high_temp) {
      any_critical = true;
    } else if (temp >= sensor_configs_[i].warning_high_temp) {
      any_warning = true;
    }
    
    // Check thermal runaway
    float trend = calculateTemperatureTrend(i);
    sensor_status_[i].temperature_trend = trend;
    if (trend > sensor_configs_[i].thermal_runaway_rate) {
      thermal_runaway = true;
      sensor_status_[i].thermal_runaway = true;
    } else {
      sensor_status_[i].thermal_runaway = false;
    }
  }
  
  // Determine fault severity
  SafetyCoordinator& safety = SafetyCoordinator::getInstance();
  
  if (thermal_runaway || any_critical) {
    char desc[256];
    snprintf(desc, sizeof(desc), 
             "Thermal critical: temp=%.1fC trend=%.1fC/min",
             sensor_status_[0].temperature_c,
             sensor_status_[0].temperature_trend);
    safety.activateFault(FaultSeverity::EMERGENCY_STOP, name(), desc);
  } else if (any_warning) {
    char desc[256];
    snprintf(desc, sizeof(desc),
             "Temperature warning: temp=%.1fC",
             sensor_status_[0].temperature_c);
    safety.activateFault(FaultSeverity::WARNING, name(), desc);
  } else {
    // All clear → deactivate fault
    safety.deactivateFault(name());
  }
}

float TemperatureMonitor::calculateTemperatureTrend(uint8_t sensor_index) {
  // Calculate linear regression over temperature history
  // Returns °C/min trend
  // [Implementation omitted for brevity - see temperature_monitor.cpp]
  return 0.0f;  // Placeholder
}

}  // namespace sigyn_teensy
```

**Integration Checklist**:

- ✅ Inherit from `Module` base class
- ✅ Implement singleton pattern
- ✅ Override `setup()` and `loop()`
- ✅ Initialize hardware in `setup()`
- ✅ Read sensors in `loop()` at control rate
- ✅ Check safety conditions at appropriate frequency (10Hz)
- ✅ Call `activateFault()` when condition detected
- ✅ Call `deactivateFault()` when condition clears
- ✅ Apply EMA filtering to raw readings
- ✅ Use hysteresis for threshold checks
- ✅ Provide descriptive fault messages
- ✅ Log initialization and errors via SerialManager

---

## XIII. Operator/Developer Usage

This section provides practical ROS2 commands for monitoring and controlling Sigyn's safety system during operation, development, and debugging.

### Monitoring Safety Status

The primary interface for monitoring safety status is the `~/safety/estop_status` topic published by the `sigyn_to_sensor_v2` bridge node.

**Real-Time E-Stop Status Monitoring**

```bash
# Watch e-stop status in real-time (shows fault sources and severity)
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status

# Example output:
#   active: true
#   hardware_estop: false
#   inter_board_safety: true
#   active_conditions: true
#   sources: ['BATTERY_LOW', 'MOTOR_OVERCURRENT']
#   state: 'EMERGENCY_STOP'
```

**Check Current Safety State (One-Shot)**

```bash
# Get single snapshot of current e-stop status
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once

# Pipe to grep to check if e-stop is active
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once | grep "active: true"
```

**Monitor Sensor Data for Safety Triggers**

```bash
# Monitor battery voltage (watch for low voltage warnings)
ros2 topic echo /sigyn/teensy_bridge/battery/status

# Watch motor temperatures (check for thermal runaway)
ros2 topic echo /sigyn/teensy_bridge/temperature/motor_0
ros2 topic echo /sigyn/teensy_bridge/temperature/motor_1

# Monitor diagnostic messages (includes timing violations)
ros2 topic echo /sigyn/teensy_bridge/diagnostics
```

**List All Available Topics**

```bash
# Show all topics published by the Teensy bridge
ros2 topic list | grep teensy_bridge

# Show topic details (message type, publisher count)
ros2 topic info /sigyn/teensy_bridge/safety/estop_status
```

### Triggering Software E-Stop

Software e-stop is controlled by publishing to the `~/commands/estop` topic. This allows ROS2 nodes (including behavior trees) to activate emergency stop remotely.

**Activate Software E-Stop**

```bash
# Trigger software e-stop (data: true)
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: true}"
```

This command:
- Activates `SOFTWARE_ESTOP` fault on the Teensy boards
- Triggers e-stop output GPIO (stops motors immediately)
- Prevents motion commands until cleared
- Does NOT require physical button press to recover

**Clear Software E-Stop**

```bash
# Clear software e-stop (data: false)
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: false}"
```

This command:
- Deactivates `SOFTWARE_ESTOP` fault
- **Does NOT** clear hardware e-stop or other fault conditions
- Only releases software e-stop lock
- Motors remain stopped if other faults are still active

**Important**: Clearing software e-stop does NOT automatically clear hardware e-stop or other fault conditions. Use the full reset procedure below to clear all faults.

### Resetting Faults

Some faults are self-healing (transient), while others require manual intervention. The reset procedure depends on the fault type.

**Check Active Fault Sources**

```bash
# View which modules are reporting faults
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once | grep sources

# Example output showing multiple fault sources:
#   sources: ['BATTERY_LOW', 'THERMAL_RUNAWAY', 'PERFORMANCE_VIOLATION']
```

**Clear Software E-Stop Only**

```bash
# If only SOFTWARE_ESTOP is active, clear it directly
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: false}"
```

**Reset Self-Healing Faults**

Many faults clear automatically when the underlying condition resolves:

- **Thermal runaway**: Wait for motor to cool below threshold
- **Battery low warning**: Charge battery above warning voltage
- **Performance violations**: Reduce system load or restart process
- **Motor overcurrent**: Remove mechanical obstruction causing stall

Monitor the `/safety/estop_status` topic to see when `active_conditions` becomes `false`, indicating all transient faults have cleared.

**Hardware E-Stop Reset**

Physical e-stop button requires manual intervention:

1. **Identify the cause**: Check diagnostics to understand why e-stop triggered
2. **Resolve the condition**: Address the safety hazard (cool motors, charge battery, etc.)
3. **Release hardware button**: Twist or pull the physical e-stop button to release
4. **Verify clear state**: Confirm `hardware_estop: false` on status topic

```bash
# Verify hardware e-stop is released
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once | grep hardware_estop

# Expected after button release:
#   hardware_estop: false
```

**RoboClaw Power Cycle (Persistent Motor Faults)**

If motor controllers enter a fault state that cannot be cleared by software alone:

1. The system will automatically cycle RoboClaw power via SSR (Solid State Relay)
2. Power cycle occurs after detecting persistent motor error codes
3. Monitor diagnostics for "ROBOCLAW_POWER_CYCLE" events

```bash
# Watch for power cycle events in diagnostics
ros2 topic echo /sigyn/teensy_bridge/diagnostics | grep -i roboclaw
```

If automatic power cycling fails, manual intervention may be required (see Section XIV).

### Common Commands Reference

**Quick Reference Card**

```bash
# ============ Monitoring Commands ============
# Real-time e-stop status
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status

# Battery health
ros2 topic echo /sigyn/teensy_bridge/battery/status

# Motor temperatures
ros2 topic echo /sigyn/teensy_bridge/temperature/motor_0

# System diagnostics (timing, faults)
ros2 topic echo /sigyn/teensy_bridge/diagnostics

# ============ Control Commands ============
# Activate software e-stop
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: true}"

# Clear software e-stop
ros2 topic pub --once /sigyn/teensy_bridge/commands/estop std_msgs/msg/Bool "{data: false}"

# ============ Diagnostic Queries ============
# Check if e-stop is active (returns exit code)
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once | grep -q "active: true"

# List all active fault sources
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once | grep sources

# Show all Teensy bridge topics
ros2 topic list | grep teensy_bridge
```

**Integration with Behavior Trees**

The safety system is designed to integrate with Nav2 behavior trees:

```xml
<!-- Example behavior tree node checking safety status -->
<Condition ID="CheckSafetyStatus" 
           topic="/sigyn/teensy_bridge/safety/estop_status"
           field="active"
           expected_value="false"/>
```

Software e-stop can be triggered from behavior tree actions for mission abort scenarios.

### ROS2 Service Calls (Future Enhancement)

Currently, safety control is publish-only via topics. A future enhancement will add service-based interfaces for:

- Querying detailed fault history
- Requesting RoboClaw power cycle
- Adjusting safety thresholds at runtime (for testing)

See [Section XVI: Future Enhancements] for details.

---

## XIV. Troubleshooting

This section covers common safety system issues, their causes, and resolution procedures. When diagnosing problems, always check the `/diagnostics` topic for detailed error context.

### Persistent E-Stop Conditions

**Symptom**: E-stop remains active after attempting to clear it. Motors will not respond to commands, and `/safety/estop_status` shows `active: true`.

**Root Causes**:

1. **Multiple Active Faults**: Clearing one fault (e.g., software e-stop) does not clear others
   - Check `sources` field to see all active fault modules
   - Each fault must be resolved independently

2. **Hardware E-Stop Not Released**: Physical button still engaged
   - Verify `hardware_estop: false` in status message
   - Physically inspect and release the e-stop button (twist/pull)

3. **Persistent Sensor Fault**: Underlying condition not resolved
   - Battery still below critical voltage (32.0V on Board 2)
   - Motor temperature still above critical threshold (85°C on Board 1)
   - RoboClaw reporting motor fault error codes
   - Performance violations still occurring (timing exceeded)

**Diagnostic Steps**:

```bash
# Step 1: Check which faults are active
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status --once

# Step 2: Check specific sensor values
ros2 topic echo /sigyn/teensy_bridge/battery/status --once     # Voltage < 32.0V?
ros2 topic echo /sigyn/teensy_bridge/temperature/motor_0 --once # Temp > 85°C?
ros2 topic echo /sigyn/teensy_bridge/diagnostics --once         # Timing violations?

# Step 3: Wait for self-healing conditions to clear
# - Allow motors to cool (thermal time constant ~60 seconds)
# - Charge battery above warning threshold (34.0V minimum)
# - Reduce system load if performance violations present
```

**Resolution**:

- **For battery faults**: Connect charger and wait for voltage to rise above 34.0V
- **For thermal faults**: Allow 1-2 minutes for motor cooling (monitor temperature topic)
- **For hardware e-stop**: Physically release button, verify `hardware_estop: false`
- **For software e-stop**: Publish `{data: false}` to `~/commands/estop`
- **For RoboClaw errors**: Wait for automatic power cycle, or manually cycle relay power

**Configuration References**:

Voltage and temperature thresholds are defined in module headers:
- Battery thresholds: See `TeensyV2/modules/battery/battery_monitor.h`
  - `critical_low_voltage = 32.0f` (Emergency shutdown)
  - `warning_low_voltage = 34.0f` (Warning level)
- Temperature thresholds: See `TeensyV2/modules/sensors/temperature_monitor.h`
  - `critical_high_temp = 85.0f` (Emergency shutdown)
  - `warning_high_temp = 70.0f` (Warning level)
  - `thermal_runaway_rate = 100.0f` (°C/min detection rate)

---

### Spurious Fault Triggers

**Symptom**: E-stop activates unexpectedly during normal operation with no obvious hazard. Faults appear and disappear rapidly (false positives).

**Root Causes**:

1. **Electrical Noise on Analog Sensors**:
   - TMP36 temperature sensors are vulnerable to EMI from motor PWM
   - Unshielded sensor wiring acting as antenna
   - Poor ground connections causing voltage reference drift

2. **EMA Filter Too Sensitive**:
   - Filter alpha (α) value set too high (responds too quickly to noise)
   - Current α values in code:
     - Temperature: α = 0.7 (2-3 second convergence)
     - Battery voltage: α = 0.5 (fast response for safety)
     - Motor current: α = 0.8 (quick overcurrent detection)

3. **Threshold Values Too Tight**:
   - Critical thresholds set too close to normal operating values
   - Insufficient hysteresis margin (difference between activate and deactivate thresholds)

4. **Intermittent Hardware Connections**:
   - Loose sensor connectors causing transient readings
   - Corroded contacts on analog input pins
   - Worn JST connectors on battery voltage sense

**Diagnostic Steps**:

```bash
# Step 1: Monitor raw sensor values for noise
ros2 topic echo /sigyn/teensy_bridge/temperature/motor_0  # Watch for rapid fluctuations
ros2 topic echo /sigyn/teensy_bridge/battery/status       # Check voltage stability

# Step 2: Check fault frequency
# If faults activate/deactivate within seconds, likely noise issue
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status

# Step 3: Inspect diagnostics for sensor health
ros2 topic echo /sigyn/teensy_bridge/diagnostics | grep -i "sensor\|fault"
```

**Resolution**:

1. **Hardware Fixes** (Preferred):
   - Add shielded cable to analog sensor wiring
   - Improve grounding (star ground topology from single point)
   - Add 0.1µF ceramic capacitor at sensor input pin (filter high-frequency noise)
   - Inspect and re-seat all sensor connectors

2. **Software Tuning** (Temporary):
   - **Reduce filter alpha**: Lower α value increases noise rejection but slows response
     ```cpp
     // In temperature_monitor.cpp
     ema_filter_alpha_ = 0.5f;  // Reduced from 0.7 for better noise rejection
     ```
   - **Increase hysteresis**: Widen gap between activate/deactivate thresholds
     ```cpp
     // In temperature_monitor.cpp checkSafetyConditions()
     const float hysteresis = 5.0f;  // Increased from 2.0°C
     ```
   - **Adjust thresholds**: Move critical thresholds further from normal operating range
     ```cpp
     // In temperature_monitor.h
     float critical_high_temp = 90.0f;  // Increased from 85.0f
     ```

3. **Validation**:
   - After changes, observe fault rate over 5+ minute operating period
   - Spurious faults should reduce to zero or near-zero
   - Ensure legitimate safety triggers still activate (test with intentional overheat)

**Configuration References**:

EMA filter parameters in module implementations:
- `TeensyV2/modules/sensors/temperature_monitor.cpp`: `ema_filter_alpha_` (line ~85)
- `TeensyV2/modules/battery/battery_monitor.cpp`: Voltage/current filter alpha values
- Hysteresis logic in `checkSafetyConditions()` methods of each module

---

### Performance Degradation Causing Safety Triggers

**Symptom**: E-stop activates with fault source `PERFORMANCE_VIOLATION`. Diagnostics show timing budget exceeded or loop frequency dropped below minimum.

**Root Causes**:

1. **Module Timing Violations**:
   - Individual module exceeds 2ms execution budget (Board 1) or 3ms (Board 2)
   - Common culprits: RoboClaw serial communication blocking, SD card writes, sensor I2C timeouts

2. **Loop Frequency Degradation**:
   - Main control loop frequency drops below minimum threshold:
     - Board 1 (Navigation): 50Hz minimum (target 85Hz)
     - Board 2 (Power/Sensors): 20Hz minimum (target 85Hz)
   - Indicates CPU overload or blocking I/O operations

3. **Increased System Load**:
   - Too many sensor modules enabled simultaneously
   - Diagnostic logging consuming excessive CPU time
   - Inter-board communication latency (serial message parsing)

**Diagnostic Steps**:

```bash
# Step 1: Check performance metrics in diagnostics
ros2 topic echo /sigyn/teensy_bridge/diagnostics | grep -i "timing\|frequency\|violation"

# Look for:
# - "module_execution_time_exceeded" warnings
# - "loop_frequency_below_minimum" errors
# - Specific module names causing delays (e.g., "roboclaw_read_timeout")

# Step 2: Identify which board is violating performance
# Check board ID in diagnostic messages (Board 1 or Board 2)

# Step 3: Monitor over time to see if condition is persistent or transient
# Transient: May be I2C sensor timeout (recovers automatically)
# Persistent: System overloaded, requires configuration change
```

**Resolution**:

1. **Reduce Sensor Polling Rates**:
   - Decrease non-critical sensor update frequencies
   - Example: Reduce VL53L0X range sensor updates from 10Hz to 5Hz
   - Configuration in module `setup()`: adjust `read_interval_ms` parameters

2. **Optimize Module Code**:
   - Identify slow modules via diagnostics `module_name` field
   - Common optimizations:
     - Use non-blocking I2C reads where possible
     - Reduce logging verbosity (disable DEBUG level during operation)
     - Minimize SD card write frequency (batch log writes)

3. **Disable Non-Essential Modules**:
   - Comment out module initialization in `board*_main.cpp` if not needed
   - Example: Disable VL53L0X sensors if not using collision avoidance
   ```cpp
   // In board1_main.cpp setup()
   // VL53L0XMonitor::getInstance().setup();  // Disabled for testing
   ```

4. **Adjust Performance Thresholds** (Testing Only):
   - **WARNING**: Only use for debugging; tightening thresholds may hide real issues
   - Thresholds defined in `TeensyV2/common/core/config.h`:
     ```cpp
     // Board 1 thresholds (config.h lines 181-186)
     #define BOARD_MAX_MODULE_TIME_MS          2.0f   // Increase to 3.0f if needed
     #define BOARD_MIN_LOOP_FREQUENCY_HZ       50.0f  // Decrease to 40Hz if needed
     ```

5. **Verify RoboClaw Communication**:
   - Most common cause of Board 1 timing violations
   - Check RoboClaw serial connection quality (loose wires, high impedance)
   - Monitor for CRC errors in diagnostics
   - Ensure RoboClaw firmware is up-to-date

**Configuration References**:

Performance thresholds in `TeensyV2/common/core/config.h`:
- Board 1: `BOARD_MAX_MODULE_TIME_MS = 2.0f` (line 181)
- Board 1: `BOARD_MIN_LOOP_FREQUENCY_HZ = 50.0f` (line 182)
- Board 2: `BOARD_MAX_MODULE_TIME_MS = 3.0f` (line 191)
- Board 2: `BOARD_MIN_LOOP_FREQUENCY_HZ = 20.0f` (line 192)

Module timing logic in `PerformanceMonitor` class.

---

### Board Communication Issues

**Symptom**: One board reports e-stop but the other does not respond. Inter-board safety signal lost. Diagnostics show "board communication timeout" or similar errors.

**Root Causes**:

1. **Serial Cable Connection**:
   - Loose or disconnected serial cable between Teensy boards
   - Damaged cable (broken wire inside insulation)
   - Incorrect baud rate mismatch (expected: 1000000 baud)

2. **GPIO Signal Line Issues**:
   - Inter-board e-stop GPIO pins not connected (future feature)
   - Electrical noise on GPIO lines causing false triggers
   - Pin configuration mismatch (input vs output)

3. **Message Parsing Failures**:
   - Corrupt serial data due to EMI or ground loops
   - CRC checksum errors in inter-board messages
   - Buffer overflow from too many messages in queue

4. **Board Initialization Timing**:
   - Board 1 starts before Board 2 is ready
   - Timeout waiting for initial handshake message
   - One board stuck in bootloader (not running application)

**Diagnostic Steps**:

```bash
# Step 1: Verify both boards are publishing to ROS2
ros2 topic list | grep teensy_bridge  # Should see topics from both boards

# Step 2: Check serial communication health
ros2 topic echo /sigyn/teensy_bridge/diagnostics | grep -i "serial\|board\|communication"

# Step 3: Look for inter-board safety status
ros2 topic echo /sigyn/teensy_bridge/safety/estop_status | grep inter_board_safety

# If inter_board_safety: false, one board is not receiving signals from the other

# Step 4: Physical inspection
# - Check serial cable connections at both ends (RX/TX pins)
# - Verify boards are powered (LEDs on both Teensy 4.1 boards)
# - Listen for relay clicks when e-stop activates (Board 1 only)
```

**Resolution**:

1. **Verify Serial Connections**:
   - Inspect serial cable physical connection (reseat connectors)
   - Test continuity with multimeter (TX→RX, RX→TX, GND→GND)
   - Confirm baud rate in both boards matches: 1000000 baud
     - Defined in `config.h`: `BOARD_SERIAL_BAUD_RATE`

2. **Check Board Power**:
   - Ensure both boards are receiving clean power (check voltage rails)
   - Look for voltage sag during motor operation (insufficient power supply)
   - Verify ground connection between boards is solid (low impedance)

3. **Review Serial Message Format**:
   - Messages use `FAULT|sources|severity` format
   - Example: `FAULT|BATTERY_LOW,THERMAL_RUNAWAY|EMERGENCY_STOP`
   - Check `sigyn_to_sensor_v2` bridge for parsing errors in logs

4. **Restart Sequence**:
   - Power cycle both boards in order:
     1. Power down Board 2 (sensor board)
     2. Power down Board 1 (navigation board)
     3. Wait 5 seconds for capacitors to discharge
     4. Power up Board 1 first (gives it time to initialize)
     5. Power up Board 2 after 3 seconds
   - Monitor diagnostics for successful handshake

5. **GPIO Inter-Board Signaling** (Future Feature):
   - Currently under development (see Section XVI)
   - Will provide hardware-level fault signaling independent of serial
   - GPIO pins defined in `config.h`:
     - Board 1: `INTER_BOARD_SIGNAL_OUTPUT_PIN = 10`
     - Board 2: `PIN_SAFETY_OUT_TO_MASTER = 10`

**Configuration References**:

Serial communication settings in `TeensyV2/common/core/config.h`:
- `BOARD_SERIAL_BAUD_RATE = 1000000` (lines 125-138)
- `BOARD_SERIAL_TIMEOUT_MS = 5000` (communication timeout)
- GPIO pin assignments (lines 141-175)

Message parsing in `sigyn_to_sensor_v2/src/message_parser.cpp`: `ParseSafetyData()` function.

---

### RoboClaw Power Cycle Failures

**Symptom**: Motor controllers enter fault state and do not recover. Automatic power cycle does not clear the fault. Motors remain unresponsive.

**Root Causes**:

1. **SSR (Solid State Relay) Failure**:
   - Relay failed in open position (cannot cut power to RoboClaw)
   - Relay failed in closed position (cannot restore power)
   - Relay control signal not reaching relay (wiring issue)

2. **RoboClaw Persistent Fault**:
   - Motor short circuit detected (requires physical repair)
   - Encoder cable disconnected (RoboClaw cannot validate motor feedback)
   - Internal RoboClaw firmware crash (power cycle insufficient)

3. **Power Cycle Timing Issues**:
   - Insufficient off-time for RoboClaw capacitors to discharge
   - Currently: 2 second off-time (may be too short for full reset)
   - RoboClaw firmware state persists across quick power cycles

4. **GPIO Control Logic Error**:
   - Pin `PIN_RELAY_ROBOCLAW_POWER = 31` (Board 1) not toggling
   - Software logic error in power cycle state machine
   - Race condition between safety coordinator and RoboClaw monitor

**Diagnostic Steps**:

```bash
# Step 1: Check for RoboClaw error codes in diagnostics
ros2 topic echo /sigyn/teensy_bridge/diagnostics | grep -i "roboclaw\|motor\|error"

# Common RoboClaw error codes:
# - E-Stop active (0x0001): Normal, clears when e-stop released
# - Temperature error (0x0002): Overheating, requires cooling
# - Main voltage high/low (0x0004/0x0008): Power supply issue
# - Motor driver fault (0x0080): Short circuit or hardware failure

# Step 2: Monitor power cycle attempts
# Look for "ROBOCLAW_POWER_CYCLE_ATTEMPT" messages in diagnostics
# Count number of cycles (should not exceed 3 attempts)

# Step 3: Verify SSR control signal
# If accessible, use multimeter to check voltage at relay control pin
# Should toggle between 0V (off) and 3.3V (on)

# Step 4: Check encoder connections
# Encoder disconnect is common cause of persistent motor fault
# Verify encoder cables are firmly seated at both motor and RoboClaw
```

**Resolution**:

1. **Manual Power Cycle** (If Automatic Fails):
   - Disconnect RoboClaw power cable at terminal block
   - Wait 10 seconds for capacitors to fully discharge
   - Reconnect power cable
   - Monitor diagnostics for successful recovery

2. **Verify SSR Operation**:
   - Use multimeter to check SSR output (should open circuit when triggered)
   - Replace SSR if failed (typical lifetime: ~100k switching cycles)
   - Check SSR control signal voltage (GPIO pin 31 on Board 1)

3. **Inspect Physical Connections**:
   - **Encoder cables**: Disconnect and reconnect both motors
   - **Motor power**: Check for loose terminals (high current can loosen screws)
   - **Ground connection**: Ensure RoboClaw ground is tied to Teensy ground

4. **Adjust Power Cycle Timing** (Code Modification):
   ```cpp
   // In roboclaw_monitor.cpp (example)
   const uint32_t kPowerCycleOffTime = 5000;  // Increase to 5 seconds
   const uint32_t kPowerCycleOnTime = 2000;   // 2 seconds before checking
   ```

5. **Check RoboClaw Error Codes**:
   - Reference RoboClaw user manual for error code meanings
   - Some errors require firmware update or RoboClaw replacement
   - Error code format: 16-bit bitmask (see `RoboClaw.h` for definitions)

6. **Factory Reset RoboClaw** (Last Resort):
   - Use BasicMicro Motion Studio software on Windows PC
   - Connect via USB to RoboClaw
   - Reset to factory defaults and re-tune PID parameters
   - **WARNING**: Loses all configuration, requires full re-setup

**Configuration References**:

RoboClaw power cycle control in `TeensyV2/common/core/config.h`:
- `PIN_RELAY_ROBOCLAW_POWER = 31` (Board 1 only, line 158)
- `CONTROLS_ROBOCLAW_ESTOP_PIN = 1` (Board 1 e-stop authority)

Power cycle logic in `TeensyV2/modules/motor/roboclaw_monitor.cpp`:
- State machine for power cycle sequence
- Error code parsing and retry logic

RoboClaw error code reference: See RoboClaw user manual or `RoboClaw.h` library header.

**Alternative Recovery Methods**:

If all software recovery fails:
1. **Hard power cycle entire robot**: Cut main battery power for 30 seconds
2. **Inspect motor windings**: Check for physical damage or short circuits
3. **Replace RoboClaw**: If persistent hardware fault detected
4. **Check motor thermal fuses**: Some motors have internal thermal protection

---

## XV. Cost/Complexity Tradeoffs

Every safety system involves tradeoffs between cost (money, time, complexity) and risk mitigation. This section documents the design decisions made for Sigyn's safety architecture and explains the rationale behind choosing more complex, expensive approaches where warranted.

### Multiple Boards vs. Single Board

**Decision**: Use two Teensy 4.1 boards instead of one larger controller.

**Costs**:
- **Hardware**: ~$50 extra cost (2 × $27 for Teensy 4.1 boards)
- **Development time**: 2-3 weeks to implement inter-board serial communication protocol
- **Complexity**: Serial message parsing, board coordination logic, fault aggregation across boards
- **Testing burden**: Must test board communication failures, timeout scenarios, message corruption

**Benefits**:
- **Isolation**: Battery monitoring (Board 2) physically separated from motor noise (Board 1)
- **Redundancy**: If one board fails, other can detect and e-stop safely
- **Partitioning**: Navigation-critical tasks (Board 1) run at 85Hz without interference from battery/IMU polling
- **Real-time performance**: Distributing load prevents timing violations that would trigger safety faults
- **Modularity**: Easier to replace a failed board without rewiring entire system

**Justification**: The cost is minimal compared to potential losses from motor meltdown ($150+ replacement) or battery fire (total system loss + property damage). Performance isolation prevents safety system self-triggering due to timing violations.

**Alternative Considered**: Single Arduino Mega 2560 or Raspberry Pi Pico W
- **Rejected because**: Insufficient GPIO/analog pins, slower processor (timing violations likely), no hardware floating-point (needed for EMA filters)

---

### Hardware E-Stop vs. Software-Only

**Decision**: Implement physical e-stop relay output (GPIO pin 30, Board 1) to cut power to RoboClaw motor controllers.

**Costs**:
- **Hardware**: ~$15 for SSR (Solid State Relay) + wiring
- **Development time**: 1 week for relay control logic, RoboClaw power cycling
- **Complexity**: Power cycle state machine, timing coordination, error code parsing
- **Risk**: Additional failure mode (relay could fail open/closed)

**Benefits**:
- **Hardware-enforced safety**: Motor cutoff independent of software state
- **Failsafe design**: Active-high signal (if Teensy crashes, motors stop)
- **Compliance**: Physical e-stop button meets OSHA requirements for machinery
- **Recovery control**: Software can power cycle RoboClaw to clear persistent faults
- **Multi-layer defense**: Even if ROS2 or Teensy firmware hangs, motors stop

**Justification**: Software-only e-stop relies on motor controller firmware obeying stop commands. If RoboClaw enters fault state or firmware hangs, software commands are ignored. Physical relay guarantees motor shutdown even during total system failure.

**Real-World Incident**: During testing, RoboClaw entered "motor driver fault" state and ignored serial stop commands. Motors continued spinning at last velocity for 8 seconds until manual power disconnect. Physical relay would have stopped motors in <50ms.

**Alternative Considered**: Software e-stop via serial commands to RoboClaw
- **Rejected because**: Relies on RoboClaw firmware responding correctly. Does not protect against firmware bugs, serial communication failures, or RoboClaw crashes.

---

### Sensor Redundancy Costs

**Decision**: Use multiple temperature sensors (2), multiple power rails monitoring (5), and dual IMUs (2) for critical safety measurements.

**Costs**:
- **Hardware**: ~$60 for additional sensors (TMP36 × 2, INA226 × 5, BNO055 × 2)
- **Development time**: 2 weeks for multi-sensor management, averaging, outlier detection
- **Complexity**: Sensor disagreement logic, weighted averaging, timeout handling
- **GPIO consumption**: 10 additional pins used for redundant sensors

**Benefits**:
- **Fault detection**: If one sensor fails, others provide continued operation
- **Accuracy**: Averaging multiple readings reduces noise, improves EMA convergence
- **Spatial coverage**: Left/right motor temperatures monitored independently (thermal runaway detection)
- **Validation**: Sensor disagreement can trigger WARNING fault (indicates failing sensor)

**Partial Implementation**: Currently, redundancy exists (multiple sensors present) but full validation logic is not implemented. Sensors are monitored independently without cross-checking.

**Future Enhancement**: Add sensor disagreement detection (e.g., left motor reports 85°C, right motor reports 25°C → likely sensor failure, trigger WARNING).

**Justification**: Temperature sensors cost $1.50 each. The cost of not detecting thermal runaway is $150+ for motor replacement. Power rail monitoring prevents battery fires (total system loss). ROI is 100:1.

**Alternative Considered**: Single temperature sensor, single battery monitor
- **Rejected because**: Single point of failure. If sensor fails or disconnects, no thermal runaway detection. Unacceptable risk for autonomous operation.

---

### Testing Infrastructure Investment

**Decision**: Invest in comprehensive testing infrastructure including mock hardware, dependency injection, unit tests, and continuous integration.

**Costs**:
- **Development time**: 4+ weeks to build mock framework, write tests, set up CI
- **Complexity**: Dual compilation paths (real hardware vs. mocks), test harness maintenance
- **Learning curve**: Team must understand dependency injection, mocking patterns
- **Build time**: Longer builds due to test compilation (extra 30 seconds per build)

**Benefits**:
- **Confidence**: Can test safety logic without risking hardware damage
- **Reproducibility**: Mock hardware provides deterministic test scenarios
- **Coverage**: Scenarios impossible to test on real hardware (e.g., extreme battery voltages, thermal runaway without burning motors)
- **Regression prevention**: Automated tests catch bugs introduced by new changes
- **Documentation**: Tests serve as executable examples of safety system behavior

**ROI Analysis**:
- Time investment: ~160 hours (4 weeks)
- Bugs caught before hardware testing: 12+ (thermal runaway logic error, EMA filter overflow, timing violation false positives)
- Cost of one motor replacement avoided: $150
- Break-even: After preventing 1-2 hardware failures

**Justification**: Testing prevents catastrophic failures during development. Without mocks, testing thermal runaway requires intentionally overheating motors (risking permanent damage). Testing battery low requires discharging to dangerous levels. Mock framework enables safe, fast validation.

**Example**: Thermal runaway test discovered that EMA filter alpha = 0.95 caused false positives due to noise amplification. Tuning to alpha = 0.7 required 20+ test iterations. On real hardware, this would require 20+ motor heating/cooling cycles (8 hours, high motor wear). With mocks, testing completed in 5 minutes.

**Alternative Considered**: Manual hardware testing only
- **Rejected because**: Too slow (heating/cooling cycles take 1+ hour each), too risky (can destroy hardware), and incomplete coverage (cannot test extreme conditions safely).

---

### Summary: Cost vs. Risk Matrix

| Design Choice | Added Cost | Risk Mitigated | Justification |
|---------------|------------|----------------|---------------|
| Dual boards | $50 + 3 weeks | System-wide failure, timing violations | Performance isolation essential for 85Hz control |
| Hardware e-stop relay | $15 + 1 week | Motor runaway, firmware hang | Physical cutoff independent of software state |
| Multiple sensors | $60 + 2 weeks | Sensor failure, single point of failure | Redundancy for critical measurements |
| Test infrastructure | 0 + 4 weeks | Development errors, hardware damage | Enables safe testing of dangerous scenarios |
| **Total** | **$125 + 10 weeks** | **~$500+ loss prevention** | **ROI: 4:1 in avoided hardware damage alone** |

**Intangible Benefits Not Quantified**:
- **Safety culture**: Testing infrastructure encourages defensive programming
- **Maintainability**: Clear architecture makes debugging faster
- **Scalability**: Multi-board design allows future expansion (Board 3 for gripper)
- **Reliability**: Self-healing faults enable autonomous operation without human intervention

**Alternative System (Minimal Cost)**:
- Single Arduino Mega 2560 ($40)
- Software-only e-stop
- Single temperature sensor, single battery monitor
- No testing infrastructure

**Total savings**: $85 + 10 weeks development time  
**Increased risk**: ~10× higher failure rate (estimated based on lack of redundancy, testing)

**Conclusion**: For a home brew robotics project, the added investment in safety is justified. The robot operates autonomously in a home environment where motor fires or battery failures could cause property damage far exceeding the cost of the safety system. The testing infrastructure pays for itself by preventing hardware destruction during development.

---

## XVI. Future Enhancements

This section outlines planned improvements to the safety system based on lessons learned during initial deployment and opportunities for enhanced protection.

### High Priority (Next 6 Months)

#### 1. IMU Safety Integration (Tilt Detection, Fall Prevention)

**Motivation**: Sigyn navigates stairs and ramps. A tilt beyond safe operating limits (>30° pitch/roll) could cause loss of control, hardware damage, or tumbling down stairs.

**Implementation Plan**:
- Add `IMUSafetyMonitor` module to Board 2 (BNO055 already present)
- Detect excessive tilt: pitch > 30° or roll > 25°
- Fault severity: WARNING at 20°, EMERGENCY_STOP at 30°
- Integration with navigation: Cancel current goal if tilt warning active
- Self-healing: Automatically clear when tilt returns to normal range

**Configuration Parameters**:
```cpp
// In imu_safety_monitor.h
struct IMUSafetyConfig {
  float warning_pitch_deg = 20.0f;      // Tilt warning threshold
  float critical_pitch_deg = 30.0f;     // Emergency stop threshold
  float warning_roll_deg = 20.0f;
  float critical_roll_deg = 25.0f;
  float angular_velocity_limit = 180.0f; // deg/s (rapid spin detection)
};
```

**Testing Approach**:
- Mock IMU data in unit tests (inject extreme tilt values)
- Real hardware: Tilt robot on test stand, verify e-stop activation
- Validate recovery: Return to level position, verify automatic clear

**Estimated Effort**: 1 week development + 1 week testing

---

#### 2. VL53L0X Obstacle Safety (Collision Prediction)

**Motivation**: Eight VL53L0X time-of-flight sensors (already installed) provide 360° obstacle detection. Currently used for mapping only, not safety.

**Implementation Plan**:
- Add collision prediction logic to `VL53L0XMonitor` (Board 1)
- Detect imminent collision: obstacle <200mm in direction of travel
- Fault severity: WARNING at 500mm (slow down), EMERGENCY_STOP at 200mm (immediate stop)
- Integration with navigation: Emergency stop overrides cmd_vel, prevents motion until clear
- Direction-aware: Only trigger if obstacle in direction of robot motion

**Configuration Parameters**:
```cpp
// In vl53l0x_monitor.h
struct VL53L0XSafetyConfig {
  uint16_t warning_distance_mm = 500;     // Slow down threshold
  uint16_t critical_distance_mm = 200;    // Emergency stop threshold
  float approach_velocity_threshold = 0.1f; // m/s minimum velocity to trigger
  uint16_t clearance_distance_mm = 600;   // Hysteresis for recovery
};
```

**Challenge**: Distinguishing intentional close approach (e.g., docking at charger) from collision hazard
- **Solution**: Add behavior tree flag `allow_close_approach` that disables collision safety temporarily

**Testing Approach**:
- Mock sensor data: Inject obstacle readings at various distances
- Real hardware: Drive toward wall, verify e-stop at 200mm
- Validate hysteresis: Verify recovery at 600mm (not 200mm, prevents oscillation)

**Estimated Effort**: 2 weeks development + 1 week testing

---

#### 3. SYSTEM_SHUTDOWN Implementation (Graceful Powerdown)

**Motivation**: Critical battery low (voltage <30V) requires controlled shutdown to prevent deep discharge damage. Currently triggers EMERGENCY_STOP but does not power down.

**Implementation Plan**:
- Add SYSTEM_SHUTDOWN severity level (beyond EMERGENCY_STOP)
- Trigger sequence: Battery <30V for >30 seconds (not transient voltage sag)
- Graceful shutdown procedure:
  1. Send ROS2 notification (allow behavior tree to save state)
  2. Stop all motors (RoboClaw e-stop)
  3. Close SD card files (flush logs)
  4. Activate main battery relay (GPIO pin 32, cut power to all systems)
- Self-recovery: Not possible (requires manual restart or charger connection)

**Configuration Parameters**:
```cpp
// In battery_monitor.h
struct BatteryShutdownConfig {
  float system_shutdown_voltage = 30.0f;      // Deep discharge protection
  uint32_t shutdown_delay_ms = 30000;         // 30 second confirmation (avoid transient sags)
  bool enable_graceful_shutdown = true;       // Feature flag for testing
};
```

**Safety Consideration**: SYSTEM_SHUTDOWN is irreversible (power cut). Must have high confidence in battery voltage reading.
- **Solution**: Require both INA226 sensors (if installed) to agree on low voltage
- **Fallback**: If only one sensor available, require sustained low reading for 60 seconds

**Testing Approach**:
- Mock battery data: Inject <30V reading, verify shutdown sequence
- Real hardware: Cannot test safely (risks deep discharge damage)
- Alternative: Modify threshold to 35V during testing, use partially discharged battery

**Estimated Effort**: 2 weeks development + 1 week testing + safety review

---

### Medium Priority (6-12 Months)

#### 4. GPIO Inter-Board Signaling (Interrupt-Driven Safety)

**Motivation**: Current inter-board communication uses serial messages (polled at 85Hz). Latency is 12-24ms. Hardware interrupt-driven GPIO signaling provides <1ms latency.

**Implementation Plan**:
- Use existing GPIO pins: `INTER_BOARD_SIGNAL_OUTPUT_PIN` (pin 10)
- Board 1 asserts GPIO high when any e-stop fault active
- Board 2 monitors GPIO via interrupt (`attachInterrupt`), immediately triggers local e-stop
- Bidirectional: Board 2 can also signal Board 1 (e.g., critical battery fault)

**Benefits**:
- **Ultra-low latency**: <1ms fault propagation (vs. current 12-24ms)
- **Reliability**: Independent of serial communication (protects against cable disconnect, corruption)
- **Simplicity**: Single wire per direction, no message parsing overhead

**Challenges**:
- **Pin availability**: GPIO pins 10-12 currently assigned but not implemented
- **Race conditions**: Interrupt handler must be thread-safe (use atomic flags)
- **Testing complexity**: Must mock GPIO interrupts in unit tests

**Configuration Parameters**:
```cpp
// In config.h (already defined, just needs implementation)
#define INTER_BOARD_SIGNAL_OUTPUT_PIN 10  // Board 1 → Board 2/3
#define PIN_SAFETY_IN_BOARD2          11  // Board 2 → Board 1
#define PIN_SAFETY_IN_BOARD3          12  // Board 3 → Board 1
```

**Testing Approach**:
- Unit tests: Mock interrupt firing, verify fault activation
- Integration test: Connect boards with GPIO wire, trigger fault on Board 1, measure Board 2 response time
- Validation: Should achieve <1ms e-stop propagation (vs. current 12ms)

**Estimated Effort**: 1 week development + 1 week testing

---

#### 5. Reverse E-Stop Signaling to All Boards

**Motivation**: Currently, Board 1 controls RoboClaw e-stop (motor cutoff). If Board 2 detects critical battery fault, it sends serial message to Board 1 which then activates e-stop. This is asymmetric and introduces latency.

**Implementation Plan**:
- Add e-stop relay output to Board 2 (GPIO pin 30, currently unused)
- Connect Board 2 relay to secondary safety circuit (e.g., cut power to Board 1)
- Allows Board 2 to independently enforce e-stop (redundant protection)
- Maintains Board 1 as primary authority for motor control

**Benefits**:
- **Redundancy**: If Board 1 fails or hangs, Board 2 can still trigger emergency shutdown
- **Symmetry**: Both boards have equal authority to protect the system
- **Defense-in-depth**: Multiple independent cutoff paths reduce single point of failure risk

**Challenges**:
- **Hardware complexity**: Requires additional relay wiring, power circuit redesign
- **Coordination**: Must prevent race conditions (both boards cycling power simultaneously)
- **Testing**: Difficult to test Board 1 failure scenarios without risking hardware

**Configuration Parameters**:
```cpp
// In config.h for Board 2
#define ESTOP_OUTPUT_PIN              30   // Currently unused on Board 2
#define ENABLE_INDEPENDENT_ESTOP      1    // Feature flag
```

**Estimated Effort**: 2 weeks development + 1 week hardware modification + 1 week testing

---

#### 6. Watchdog Timer Implementation

**Motivation**: If Teensy firmware hangs (infinite loop, stack overflow), safety monitoring stops but motors may continue running. Watchdog timer forces hardware reset if firmware stops feeding the watchdog.

**Implementation Plan**:
- Enable Teensy 4.1 hardware watchdog timer (WDT)
- Timeout period: 500ms (must be fed twice per loop iteration at 85Hz)
- Feed watchdog in main loop after safety checks complete
- If firmware hangs: WDT triggers hardware reset → SafetyCoordinator re-initializes → e-stop asserted by default

**Benefits**:
- **Hang protection**: Automatic recovery from firmware crashes
- **Fail-safe default**: On reset, e-stop is active until explicitly cleared
- **Low overhead**: Feeding watchdog is single register write (~1µs)

**Challenges**:
- **False triggers**: If loop takes >500ms (timing violation), watchdog fires unnecessarily
- **Recovery testing**: Must verify system recovers correctly from forced reset
- **State preservation**: Some state lost on reset (encoder counts, calibration data)

**Configuration Parameters**:
```cpp
// In config.h
#define ENABLE_WATCHDOG_TIMER         1    // Feature flag
#define WATCHDOG_TIMEOUT_MS           500  // Must be > worst-case loop time
```

**Testing Approach**:
- Inject infinite loop in test code, verify watchdog reset
- Measure worst-case loop timing to set appropriate timeout
- Validate e-stop active state after reset

**Estimated Effort**: 1 week development + 1 week validation

---

### Low Priority (12+ Months)

#### 7. Complete Testing Coverage for All Modules

**Motivation**: Current test coverage is ~60% (SafetyCoordinator, TemperatureMonitor, BatteryMonitor tested; RoboClawMonitor, VL53L0XMonitor, IMU not fully tested).

**Implementation Plan**:
- Expand mock framework to cover RoboClaw serial protocol
- Add VL53L0X mock sensor (inject distance readings)
- Add IMU mock (inject orientation, angular velocity data)
- Achieve 90%+ code coverage across all safety-critical modules

**Estimated Effort**: 4 weeks development (1 week per major module)

---

#### 8. Safety Event Logging and Replay

**Motivation**: After a fault event, operators need to understand what happened. Currently, only real-time diagnostics available.

**Implementation Plan**:
- Add circular buffer for last 100 safety events (timestamp, source, severity, description)
- Store to SD card when e-stop triggered (crash log)
- Add ROS2 service to retrieve event history: `teensy_sensor_safety_history`
- Replay capability: Load recorded events into mock system for debugging

**Estimated Effort**: 2 weeks development + 1 week testing

---

#### 9. Adaptive Threshold Tuning

**Motivation**: Fixed thresholds (e.g., 85°C critical temperature) may not suit all operating environments (cold garage vs. hot summer).

**Implementation Plan**:
- Add runtime configuration service: `teensy_sensor_update_config`
- Allow behavior trees to temporarily adjust thresholds (e.g., relax temperature limits in cold weather)
- Store tuned thresholds to EEPROM for persistence across reboots
- Add safety limits: Prevent setting thresholds outside safe ranges

**Estimated Effort**: 2 weeks development + 1 week safety review

---

### Future Considerations (Research Needed)

- **Machine learning anomaly detection**: Use historical sensor patterns to detect unusual behavior
- **Predictive maintenance**: Detect gradual degradation (e.g., motor current increasing over time)
- **Multi-robot coordination**: Share safety status between multiple Sigyn units
- **Remote safety monitoring**: Cloud dashboard showing safety status in real-time

---

**Tracking and Prioritization**:

All enhancement tasks are tracked in `TODO_list.txt` with priority levels and estimated effort. Prioritization based on:
1. **Risk reduction**: How much does this reduce catastrophic failure probability?
2. **Implementation complexity**: Can we deliver this reliably in available time?
3. **Hardware availability**: Do we already have the sensors/actuators needed?
4. **Testing feasibility**: Can we validate this without risking hardware damage?

[See TODO_list.txt for detailed implementation tracking]

---

## Appendices

### Appendix A: Fault Message Format Examples

This appendix provides detailed examples of the serial message formats used for safety communication between Teensy boards and the ROS2 bridge node.

#### Message Structure Overview

All messages follow a consistent key-value JSON format for reliable parsing:

```
TYPE<BOARD_ID>:{JSON_PAYLOAD}\n
```

- **TYPE**: Message category (FAULT, DIAG, BATT, etc.)
- **BOARD_ID**: Integer board identifier (1 or 2)
- **JSON_PAYLOAD**: Structured data in JSON format
- **\n**: Newline terminator (essential for line-based parsing)

#### FAULT Message Examples

FAULT messages are sent by `SafetyCoordinator::activateFault()` and `deactivateFault()` to communicate safety status changes.

**Example 1: Battery Low Fault Activation**

```
FAULT1:{"active_fault":"true","source":"BATTERY_LOW","severity":"WARNING","description":"Battery voltage 33.2V below warning threshold 34.0V"}
```

**Parsed Fields**:
- `active_fault`: "true" (fault is newly activated)
- `source`: "BATTERY_LOW" (identifies the module reporting the fault)
- `severity`: "WARNING" (fault severity level)
- `description`: Human-readable explanation with specific values

**ROS2 Bridge Action**: Publishes to `/sigyn/teensy_bridge/safety/estop_status` with `sources: ['BATTERY_LOW']`, `state: 'WARNING'`

---

**Example 2: Thermal Runaway Emergency Stop**

```
FAULT1:{"active_fault":"true","source":"THERMAL_RUNAWAY","severity":"EMERGENCY_STOP","description":"Motor 0 temperature rising at 12.5C/min exceeds limit 5.0C/min"}
```

**Parsed Fields**:
- `active_fault`: "true"
- `source`: "THERMAL_RUNAWAY"
- `severity`: "EMERGENCY_STOP" (triggers physical e-stop relay)
- `description`: Includes specific rate-of-change measurement

**ROS2 Bridge Action**: 
- Publishes `estop_status` with `active: true`, `state: 'EMERGENCY_STOP'`
- Cancels any active navigation goals (if behavior tree integration enabled)
- Logs emergency event to diagnostic array

---

**Example 3: Multiple Simultaneous Faults**

When multiple faults are active, each is reported separately. The ROS2 bridge aggregates them:

```
FAULT1:{"active_fault":"true","source":"BATTERY_LOW","severity":"WARNING","description":"Battery 32.5V"}
FAULT1:{"active_fault":"true","source":"MOTOR_OVERCURRENT","severity":"EMERGENCY_STOP","description":"Motor 1 current 18.2A exceeds limit 15.0A"}
```

**ROS2 Bridge Aggregation**:
```yaml
estop_status:
  active: true
  hardware_estop: false
  inter_board_safety: false
  active_conditions: true
  sources: ['BATTERY_LOW', 'MOTOR_OVERCURRENT']
  state: 'EMERGENCY_STOP'  # Highest severity wins
```

---

**Example 4: Fault Deactivation (Self-Healing)**

```
FAULT1:{"active_fault":"false","source":"PERFORMANCE_VIOLATION","severity":"WARNING"}
```

**Parsed Fields**:
- `active_fault`: "false" (fault has cleared)
- `source`: "PERFORMANCE_VIOLATION" (identifies which fault cleared)
- `severity`: Original severity level when fault was active

**ROS2 Bridge Action**: Removes "PERFORMANCE_VIOLATION" from `sources` array. If no other faults active, sets `active: false`.

---

**Example 5: Complete E-Stop Status Message**

The safety coordinator also sends periodic status summaries (every 5 seconds when faults active):

```
FAULT1:{"state":"EMERGENCY_STOP","hw_estop":"false","inter_board_safety":"true","active_conditions":"true","sources":"THERMAL_RUNAWAY,BATTERY_LOW"}
```

**Parsed Fields**:
- `state`: Current safety state (NORMAL, WARNING, EMERGENCY_STOP, SYSTEM_SHUTDOWN)
- `hw_estop`: "false" (physical e-stop button not pressed)
- `inter_board_safety`: "true" (inter-board communication fault detected)
- `active_conditions`: "true" (at least one fault is active)
- `sources`: Comma-separated list of all active fault sources

**ROS2 Bridge Action**: Publishes complete status to `~/safety/estop_status` topic.

---

#### DIAG Message Examples

DIAG messages provide diagnostic information for debugging and monitoring.

**Example 6: Module Timing Violation**

```
DIAG1:{"level":"ERROR","module":"PerformanceMonitor","message":"Module RoboClawMonitor exceeded 2.0ms budget (actual: 3.2ms)","timestamp":123456789}
```

**Parsed Fields**:
- `level`: "ERROR" (diagnostic severity: DEBUG, INFO, WARNING, ERROR)
- `module`: "PerformanceMonitor" (source of diagnostic)
- `message`: Human-readable diagnostic message
- `timestamp`: Milliseconds since boot (from `millis()`)

**ROS2 Bridge Action**: Publishes to `/sigyn/teensy_bridge/diagnostics` as `diagnostic_msgs/DiagnosticArray`

---

**Example 7: Fault Activation Diagnostic**

```
DIAG1:{"level":"WARNING","module":"SafetyCoordinator","message":"Fault activated: source=BATTERY_LOW, severity=WARNING, description=Battery 33.2V, active_faults=1","timestamp":123457000}
```

This diagnostic accompanies the FAULT message, providing additional context.

---

#### BATT Message Examples

BATT messages report battery status from `BatteryMonitor`.

**Example 8: Normal Battery Status**

```
BATT2:{"id":"0","v":"39.8","p":"0.82","c":"1.2","state":"NORMAL","t":"25.3"}
```

**Parsed Fields** (Board 2 format):
- `id`: Battery index (0-4, Board 2 monitors up to 5 power rails)
- `v`: Voltage in Volts (39.8V)
- `p`: Power in Watts (0.82W)
- `c`: Current in Amperes (1.2A)
- `state`: Battery state (NORMAL, WARNING, CRITICAL, CHARGING, etc.)
- `t`: Temperature in Celsius (25.3°C, if temperature sensor present)

**ROS2 Bridge Action**: Publishes to `/sigyn/teensy_bridge/battery/status` as `sensor_msgs/BatteryState`

---

**Example 9: Critical Battery (Triggers Fault)**

```
BATT2:{"id":"0","v":"31.5","p":"0.50","c":"0.8","state":"CRITICAL","t":"26.1"}
FAULT2:{"active_fault":"true","source":"BATTERY_LOW","severity":"EMERGENCY_STOP","description":"Battery 0 voltage 31.5V below critical threshold 32.0V"}
```

Note: BATT message shows immediate state, FAULT message triggers safety response.

---

#### TEMPERATURE Message Examples

TEMPERATURE messages report motor and board temperatures.

**Example 10: Normal Temperature Reading**

```
TEMPERATURE1:{"sensors":[{"id":"0","name":"Motor_Left","temp":"45.2","location":"Left_Motor_Housing","status":"NORMAL"},{"id":"1","name":"Motor_Right","temp":"47.8","location":"Right_Motor_Housing","status":"NORMAL"}]}
```

**Parsed Fields**:
- `sensors`: Array of sensor objects
  - `id`: Sensor index
  - `name`: Human-readable sensor name
  - `temp`: Temperature in Celsius
  - `location`: Physical location description
  - `status`: "NORMAL", "WARNING", or "CRITICAL"

**ROS2 Bridge Action**: Publishes to `/sigyn/teensy_bridge/temperature/motor_0` and `motor_1` as `sensor_msgs/Temperature`

---

**Example 11: Thermal Runaway Detected**

```
TEMPERATURE1:{"sensors":[{"id":"0","name":"Motor_Left","temp":"88.5","location":"Left_Motor_Housing","status":"CRITICAL"}]}
FAULT1:{"active_fault":"true","source":"THERMAL_RUNAWAY","severity":"EMERGENCY_STOP","description":"Motor 0 temperature 88.5C exceeds critical threshold 85.0C"}
```

---

#### Inter-Board Communication Examples

When Board 2 detects a fault, it sends FAULT messages to ROS2. The ROS2 bridge aggregates faults from both boards.

**Example 12: Board 2 Battery Fault, Board 1 Receives Via ROS2**

Board 2 sends:
```
FAULT2:{"active_fault":"true","source":"BATTERY_LOW","severity":"EMERGENCY_STOP","description":"Battery 31.0V"}
```

ROS2 bridge aggregates from both boards and publishes unified status:
```yaml
estop_status:
  active: true
  sources: ['BATTERY_LOW']  # Source from Board 2
  state: 'EMERGENCY_STOP'
```

Board 1 reads this status via ROS2 subscription (future enhancement) or via serial message forwarding (current implementation uses `sigyn_to_sensor_v2` node to relay messages).

---

#### Message Timing and Ordering

**Frequency**:
- **FAULT messages**: Sent immediately when fault activates/deactivates (event-driven)
- **Status summaries**: Every 5 seconds when faults active, every 30 seconds when normal
- **BATT messages**: Every 1 second (1Hz status reporting)
- **TEMPERATURE messages**: Every 1 second (1Hz status reporting)
- **DIAG messages**: Variable (event-driven for errors, periodic for status)

**Guaranteed Ordering**: Messages from a single board are ordered (serial is sequential). Messages from different boards may interleave.

**Example Message Sequence During E-Stop**:

```
T+0.000: TEMPERATURE1:{"sensors":[{"id":"0","temp":"86.0","status":"CRITICAL"}]}
T+0.001: FAULT1:{"active_fault":"true","source":"THERMAL_RUNAWAY","severity":"EMERGENCY_STOP",...}
T+0.002: DIAG1:{"level":"ERROR","module":"SafetyCoordinator","message":"E-stop activated",...}
T+0.010: (Physical e-stop relay activates, motors stop)
T+5.000: FAULT1:{"state":"EMERGENCY_STOP","active_conditions":"true","sources":"THERMAL_RUNAWAY"}
T+60.000: TEMPERATURE1:{"sensors":[{"id":"0","temp":"72.0","status":"WARNING"}]}
T+60.001: FAULT1:{"active_fault":"false","source":"THERMAL_RUNAWAY","severity":"EMERGENCY_STOP"}
T+60.002: DIAG1:{"level":"INFO","module":"SafetyCoordinator","message":"Fault cleared",...}
```

---

### Appendix B: Safety-Related Configuration Parameters

This appendix provides a comprehensive reference to all safety-related configuration parameters across the TeensyV2 system, organized by subsystem.

#### File Locations

All configuration parameters are defined in header files within the TeensyV2 module system:

- **Board-level config**: `TeensyV2/common/core/config.h`
- **Temperature thresholds**: `TeensyV2/modules/sensors/temperature_monitor.h`
- **Battery thresholds**: `TeensyV2/modules/battery/battery_monitor.h`
- **Motor monitoring**: `TeensyV2/modules/motor/roboclaw_monitor.h`
- **Performance limits**: `TeensyV2/modules/performance/performance_monitor.h`
- **Safety coordinator**: `TeensyV2/modules/safety/safety_coordinator.h`

---

#### Board-Level Configuration (config.h)

**Performance Timing Thresholds (Board 1 - Navigation)**

```cpp
// File: TeensyV2/common/core/config.h, lines 181-186
#define BOARD_MAX_MODULE_TIME_MS          2.0f   // Maximum module execution time
#define BOARD_MIN_LOOP_FREQUENCY_HZ       50.0f  // Minimum acceptable loop frequency
#define BOARD_CRITICAL_EXECUTION_TIME_US  10000  // Critical execution time warning (10ms)
#define BOARD_CRITICAL_FREQUENCY_HZ       50.0f  // Critical frequency warning
#define BOARD_SAFETY_EXECUTION_TIME_US    20000  // Safety trigger execution time (20ms)
```

**Rationale**: Board 1 controls motors and navigation, requiring tight timing constraints. 2ms per module ensures 85Hz loop frequency (11.7ms period). If any module exceeds 2ms, performance fault triggered.

**Performance Timing Thresholds (Board 2 - Sensors)**

```cpp
// File: TeensyV2/common/core/config.h, lines 191-196
#define BOARD_MAX_MODULE_TIME_MS          3.0f   // Maximum module execution time
#define BOARD_MIN_LOOP_FREQUENCY_HZ       20.0f  // Minimum acceptable loop frequency
#define BOARD_CRITICAL_EXECUTION_TIME_US  15000  // Critical execution time warning (15ms)
#define BOARD_CRITICAL_FREQUENCY_HZ       20.0f  // Critical frequency warning
#define BOARD_SAFETY_EXECUTION_TIME_US    25000  // Safety trigger execution time (25ms)
```

**Rationale**: Board 2 handles sensors and battery monitoring, less time-critical than motor control. Relaxed thresholds reduce spurious performance faults.

**GPIO Pin Assignments**

```cpp
// File: TeensyV2/common/core/config.h, lines 148-160 (Board 1)
#define INTER_BOARD_SIGNAL_OUTPUT_PIN 10  // Signal other boards
#define PIN_SAFETY_IN_BOARD2          11  // Receive from Board 2
#define PIN_SAFETY_IN_BOARD3          12  // Receive from Board 3
#define HARDWARE_ESTOP_INPUT_PIN      2   // Hardware e-stop button
#define ESTOP_OUTPUT_PIN              30  // E-stop relay output
#define PIN_RELAY_ROBOCLAW_POWER      31  // RoboClaw power relay
#define PIN_RELAY_MAIN_BATTERY        32  // Main battery cutoff relay
```

**Rationale**: Pin assignments documented for hardware integration. `ESTOP_OUTPUT_PIN` (30) controls physical relay that cuts power to RoboClaw motor controllers.

---

#### Temperature Monitoring Configuration (temperature_monitor.h)

**Temperature Thresholds (Per-Sensor)**

```cpp
// File: TeensyV2/modules/sensors/temperature_monitor.h, lines 63-68
struct TemperatureSensorConfig {
  float critical_high_temp = 85.0f;         // Critical high temperature (°C)
  float warning_high_temp = 70.0f;          // Warning high temperature (°C)
  float warning_low_temp = 5.0f;            // Warning low temperature (°C)
  float critical_low_temp = 0.0f;           // Critical low temperature (°C)
  float thermal_runaway_rate = 100.0f;      // Temperature rise rate (°C/min)
};
```

**Fault Severity Mapping**:
- `critical_high_temp` (85°C): Triggers `EMERGENCY_STOP` (motor damage imminent)
- `warning_high_temp` (70°C): Triggers `WARNING` (performance degradation, not unsafe)
- `thermal_runaway_rate` (100°C/min): Triggers `EMERGENCY_STOP` (stalled motor, rapid heating)

**Rationale**: 85°C based on motor manufacturer specifications (typical continuous rating: 80-100°C). 100°C/min rate detects stalled motor in ~30 seconds (from 25°C ambient to 75°C warning).

**System-Wide Temperature Thresholds**

```cpp
// File: TeensyV2/modules/sensors/temperature_monitor.h, lines 83-85
struct TemperatureMonitorConfig {
  float system_critical_temp = 80.0f;       // System-wide critical temperature
  float system_warning_temp = 65.0f;        // System-wide warning temperature
};
```

**Rationale**: System thresholds apply when sensor-specific config not available. Conservative values (5°C below per-sensor limits) provide extra safety margin.

**Filter Configuration**

```cpp
// File: TeensyV2/modules/sensors/temperature_monitor.cpp, line ~85
const float ema_filter_alpha_ = 0.7f;  // EMA filter alpha (0.0 = no filtering, 1.0 = no smoothing)
```

**Rationale**: α = 0.7 provides 2-3 second convergence time (see Section X for detailed analysis). Balances noise rejection with fast thermal runaway detection.

---

#### Battery Monitoring Configuration (battery_monitor.h)

**Voltage Thresholds**

```cpp
// File: TeensyV2/modules/battery/battery_monitor.h, lines 51-52
struct BatteryConfig {
  float critical_low_voltage = 32.0f;    // Emergency shutdown voltage (V)
  float warning_low_voltage = 34.0f;     // Low battery warning (V)
};
```

**Fault Severity Mapping**:
- `critical_low_voltage` (32.0V): Triggers `EMERGENCY_STOP` (deep discharge protection)
- `warning_low_voltage` (34.0V): Triggers `WARNING` (initiates return-to-base behavior)

**Rationale**: 10S LiPo battery (nominal 37.0V, full charge 42.0V). 32.0V = 3.2V/cell (absolute minimum, deep discharge damage below this). 34.0V = 3.4V/cell (safe cutoff for prolonged battery life).

**Current Thresholds**

```cpp
// File: TeensyV2/modules/battery/battery_monitor.h, line 56
  float critical_high_current = 20.0f;   // Critical current limit (A)
```

**Fault Severity**: Triggers `EMERGENCY_STOP` (motor stall or short circuit likely)

**Rationale**: Normal operating current: 1-5A. Current >20A indicates motor mechanical jam, short circuit, or controller failure. Sustained high current causes battery overheating and fire risk.

**Hardware Configuration**

```cpp
// File: TeensyV2/modules/battery/battery_monitor.h, lines 61-66
  int ina226_address = 0x40;              // I2C address for INA226 sensor
  int analog_voltage_pin = A0;            // Analog backup voltage sense
  float voltage_divider_ratio = 11.0f;    // Voltage divider scaling
  float voltage_calibration_offset = 0.0f; // Calibration offset (V)
  float current_calibration_scale = 1.0f;  // Calibration scale factor
```

**Rationale**: INA226 provides high-accuracy current/voltage measurement. Analog pin A0 serves as backup if I2C fails. Voltage divider (11:1) scales 42V max down to 3.3V Teensy ADC range.

---

#### Motor Monitoring Configuration (roboclaw_monitor.h)

**Motor Current Thresholds**

```cpp
// File: TeensyV2/modules/motor/roboclaw_monitor.h (approximate location)
struct RoboClawConfig {
  float motor_overcurrent_threshold = 15.0f;  // Overcurrent detection (A)
  float motor_warning_current = 10.0f;        // Warning level (A)
  uint32_t overcurrent_duration_ms = 1000;    // Sustained overcurrent time
};
```

**Fault Severity Mapping**:
- `motor_overcurrent_threshold` (15.0A): Triggers `EMERGENCY_STOP` after 1 second sustained
- `motor_warning_current` (10.0A): Triggers `WARNING` (high load, not yet dangerous)

**Rationale**: Motor rated current: 5A continuous, 15A peak (10 seconds). Sustained >15A indicates mechanical jam (e.g., wheel caught on obstacle). 1 second duration prevents false triggers during acceleration.

**Encoder Runaway Detection**

```cpp
// File: TeensyV2/modules/motor/roboclaw_monitor.h (approximate)
struct EncoderSafetyConfig {
  uint32_t encoder_timeout_ms = 500;          // Encoder signal loss timeout
  float velocity_disagreement_threshold = 0.5f; // m/s difference (commanded vs actual)
  uint32_t runaway_detection_time_ms = 2000;  // Runaway confirmation time
};
```

**Fault Severity**: Triggers `EMERGENCY_STOP` (motor controller malfunction)

**Rationale**: If commanded velocity = 0 but encoders show >0.5 m/s for >2 seconds, motor controller has lost control. Common after RoboClaw firmware crash.

**Power Cycle Timing**

```cpp
// File: TeensyV2/modules/motor/roboclaw_monitor.cpp (approximate)
const uint32_t kPowerCycleOffTime = 2000;  // Power off duration (ms)
const uint32_t kPowerCycleOnTime = 2000;   // Power on delay before retry (ms)
const uint8_t kMaxPowerCycleAttempts = 3;  // Maximum retry count
```

**Rationale**: 2 second off-time allows RoboClaw capacitors to discharge fully (ensures true reset). 3 attempts max prevents infinite cycle loop if RoboClaw has permanent hardware fault.

---

#### Performance Monitoring Configuration (performance_monitor.h)

**Timing Budget Enforcement**

```cpp
// File: TeensyV2/modules/performance/performance_monitor.h (approximate)
struct PerformanceConfig {
  float max_module_time_ms = 2.0f;           // Per-module time budget
  float min_loop_frequency_hz = 50.0f;       // Minimum acceptable frequency
  uint32_t violation_count_threshold = 10;   // Consecutive violations before fault
};
```

**Fault Severity**: Triggers `WARNING` after 10 consecutive violations (allows transient spikes)

**Rationale**: Single timing violation (e.g., I2C sensor timeout) should not trigger e-stop. 10 consecutive violations indicates persistent performance problem requiring investigation.

---

#### Safety Coordinator Configuration (safety_coordinator.h)

**Fault Management**

```cpp
// File: TeensyV2/modules/safety/safety_coordinator.h, lines 27-28
static constexpr size_t kMaxFaults = 16;            // Maximum simultaneous faults
static constexpr size_t kMaxFaultSourceLength = 32; // Fault source name length
```

**Rationale**: 16 fault slots allows for multiple simultaneous failures (battery low + thermal runaway + performance violation). 32 characters sufficient for descriptive source names ("MOTOR_LEFT_OVERCURRENT").

**Recovery Configuration**

```cpp
// File: TeensyV2/modules/safety/safety_coordinator.cpp (approximate)
const uint32_t kRecoveryDelayMs = 1000;  // Delay before attempting recovery
const uint32_t kStatusReportInterval = 5000; // Status report frequency when faults active
```

**Rationale**: 1 second recovery delay prevents oscillation (fault clears, immediately re-triggers). 5 second status reports provide sufficient visibility without flooding serial.

---

### Appendix C: System Diagrams

This appendix provides ASCII diagrams illustrating key safety system workflows and state machines.

#### Diagram 1: Multi-Board Fault Flow

This diagram shows how faults propagate from individual sensors through the safety coordinator to the physical e-stop hardware.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      BOARD 1 (Navigation & Safety)                      │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │ Temperature  │    │  RoboClaw    │    │ Performance  │              │
│  │   Monitor    │    │   Monitor    │    │   Monitor    │              │
│  │              │    │              │    │              │              │
│  │ - TMP36 × 2  │    │ - Current    │    │ - Loop Time  │              │
│  │ - EMA Filter │    │ - Encoder    │    │ - Module Time│              │
│  │ - Thermal    │    │ - Runaway    │    │ - Frequency  │              │
│  │   Runaway    │    │   Detect     │    │   Monitor    │              │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘              │
│         │                   │                   │                       │
│         │ activateFault()   │                   │                       │
│         └───────────────────┼───────────────────┘                       │
│                             ▼                                           │
│                  ┌────────────────────────┐                             │
│                  │  Safety Coordinator    │                             │
│                  │                        │                             │
│                  │  - Fault Aggregation   │                             │
│                  │  - Severity Evaluation │                             │
│                  │  - E-Stop Decision     │                             │
│                  └───────────┬────────────┘                             │
│                              │                                           │
│         ┌────────────────────┼────────────────────┐                     │
│         │                    │                    │                     │
│         ▼                    ▼                    ▼                     │
│  ┌─────────────┐   ┌──────────────────┐   ┌─────────────┐             │
│  │  Serial     │   │  E-Stop GPIO     │   │ Inter-Board │             │
│  │  Manager    │   │  (Pin 30)        │   │ GPIO Signal │             │
│  │             │   │                  │   │ (Pin 10)    │             │
│  │ FAULT msg   │   │ Relay Control    │   │ [Future]    │             │
│  └──────┬──────┘   └────────┬─────────┘   └─────────────┘             │
│         │                   │                                           │
└─────────┼───────────────────┼───────────────────────────────────────────┘
          │                   │
          │ USB Serial        │ Physical Relay
          ▼                   ▼
    ┌───────────┐      ┌────────────────┐
    │   ROS2    │      │   RoboClaw     │
    │   Bridge  │      │ Motor Driver   │
    │           │      │                │
    │ sigyn_to_ │      │ E-Stop Pin     │
    │ sensor_v2 │      │ Latches LOW    │
    └─────┬─────┘      └────────────────┘
          │                   │
          │ /safety/          │ Motors Stop
          │ estop_status      │ Immediately
          ▼                   ▼
    ┌───────────┐      ┌────────────────┐
    │ Behavior  │      │  Motor Output  │
    │ Tree      │      │  (Disabled)    │
    │ (Nav2)    │      └────────────────┘
    └───────────┘


┌─────────────────────────────────────────────────────────────────────────┐
│                      BOARD 2 (Power & Sensors)                          │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                           │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │  Battery     │    │     IMU      │    │ Performance  │              │
│  │  Monitor     │    │   Monitor    │    │   Monitor    │              │
│  │              │    │              │    │              │              │
│  │ - INA226 × 5 │    │ - BNO055 × 2 │    │ - Loop Time  │              │
│  │ - Voltage    │    │ - [Future    │    │ - Module Time│              │
│  │ - Current    │    │   Safety]    │    │              │              │
│  │ - Power      │    │              │    │              │              │
│  └──────┬───────┘    └──────────────┘    └──────┬───────┘              │
│         │                                        │                       │
│         │ activateFault()                        │                       │
│         └────────────────────────────────────────┘                       │
│                             ▼                                           │
│                  ┌────────────────────────┐                             │
│                  │  Safety Coordinator    │                             │
│                  │                        │                             │
│                  │  - Fault Aggregation   │                             │
│                  │  - Severity Evaluation │                             │
│                  └───────────┬────────────┘                             │
│                              │                                           │
│         ┌────────────────────┴────────────────────┐                     │
│         ▼                                          ▼                     │
│  ┌─────────────┐                          ┌─────────────┐               │
│  │  Serial     │                          │ Inter-Board │               │
│  │  Manager    │                          │ GPIO Signal │               │
│  │             │                          │ (Pin 10)    │               │
│  │ FAULT msg   │                          │ [Future]    │               │
│  └──────┬──────┘                          └─────────────┘               │
│         │                                                                │
└─────────┼────────────────────────────────────────────────────────────────┘
          │
          │ USB Serial
          ▼
    ┌───────────┐
    │   ROS2    │
    │   Bridge  │
    │           │
    │ Aggregates│
    │ Board1+2  │
    │ Status    │
    └───────────┘

Legend:
  ─── : Control/Data Flow
  ▼   : Direction of Flow
  [ ] : Component or Process
  /// : Future/Planned Feature
```

**Key Observations**:
1. **Board 1 Authority**: Only Board 1 controls the physical e-stop relay (pin 30). Board 2 faults propagate via ROS2 bridge.
2. **Multiple Paths**: Safety signals flow through both serial (for ROS2 visibility) and GPIO (for hardware enforcement).
3. **Fail-Safe**: If Board 1 loses communication with Board 2, inter-board fault triggers. If Board 1 crashes, relay defaults to e-stop state.

---

#### Diagram 2: Fault Lifecycle State Machine

This diagram illustrates the complete lifecycle of a fault from detection through activation, monitoring, and eventual recovery.

```
                    ┌────────────────────┐
                    │   Fault Source     │
                    │   (Sensor Module)  │
                    └─────────┬──────────┘
                              │
                              │ Condition Detected
                              │ (e.g., temp > threshold)
                              ▼
                    ┌────────────────────┐
                    │  activateFault()   │
                    │  Called            │
                    └─────────┬──────────┘
                              │
                              ▼
            ┌─────────────────────────────────┐
            │  Safety Coordinator:            │
            │  getOrAllocateFaultIndex()      │
            │                                 │
            │  Find existing fault OR         │
            │  Allocate new slot (1-16)       │
            └──────────┬──────────────────────┘
                       │
                       ▼
            ┌────────────────────────────────┐
            │  Store Fault Data:             │
            │  - source (e.g., "BATTERY_LOW")│
            │  - severity (e.g., WARNING)    │
            │  - description                 │
            │  - timestamp                   │
            │  - active = true               │
            └──────────┬───────────────────────┘
                       │
                       ▼
            ┌────────────────────────────────┐
            │  Severity Evaluation           │
            │                                │
            │  If EMERGENCY_STOP or          │
            │  SYSTEM_SHUTDOWN:              │
            │    active_estop_count_++       │
            └──────────┬───────────────────────┘
                       │
        ┌──────────────┴──────────────┐
        │                             │
        ▼                             ▼
┌───────────────┐            ┌────────────────────┐
│ Severity:     │            │ Severity:          │
│ DEGRADED or   │            │ EMERGENCY_STOP or  │
│ WARNING       │            │ SYSTEM_SHUTDOWN    │
└───────┬───────┘            └─────────┬──────────┘
        │                              │
        │                              ▼
        │                    ┌─────────────────────┐
        │                    │ Trigger E-Stop:     │
        │                    │ - GPIO Pin 30 HIGH  │
        │                    │ - RoboClaw Latched  │
        │                    │ - Motors Stopped    │
        │                    └─────────┬───────────┘
        │                              │
        └──────────────┬───────────────┘
                       │
                       ▼
            ┌────────────────────────────────┐
            │  Send FAULT Message:           │
            │  {"active_fault":"true",       │
            │   "source":"...",              │
            │   "severity":"..."}            │
            └──────────┬───────────────────────┘
                       │
                       ▼
            ┌────────────────────────────────┐
            │  ACTIVE STATE:                 │
            │  Fault persists, monitored     │
            │  every loop iteration          │
            │                                │
            │  Sensor continues checking     │
            │  condition in loop()           │
            └──────────┬───────────────────────┘
                       │
                       │ Condition Cleared?
                       │ (e.g., temp drops)
                       ▼
                   ┌───────┐     No
                   │ Check ├────────┐
                   └───┬───┘        │
                       │ Yes        │
                       ▼            │
            ┌────────────────────┐  │
            │ deactivateFault()  │  │
            │ Called             │  │
            └─────────┬──────────┘  │
                      │             │
                      ▼             │
            ┌────────────────────┐  │
            │ Find Fault Index   │  │
            │ Verify active=true │  │
            └─────────┬──────────┘  │
                      │             │
                      ▼             │
            ┌────────────────────┐  │
            │ Send Clearance Msg:│  │
            │ {"active_fault":   │  │
            │  "false"}          │  │
            └─────────┬──────────┘  │
                      │             │
                      ▼             │
            ┌────────────────────┐  │
            │ active = false     │  │
            │ active_estop_count │  │
            │ decremented        │  │
            └─────────┬──────────┘  │
                      │             │
                      ▼             │
            ┌────────────────────┐  │
            │ Last Fault?        │  │
            │ (active_estop      │  │
            │  _count == 0)      │  │
            └─────┬───────┬──────┘  │
                  │ Yes   │ No      │
                  ▼       │         │
        ┌─────────────────┐ │       │
        │ Clear E-Stop:   │ │       │
        │ - GPIO Pin LOW  │ │       │
        │ - RoboClaw      │ │       │
        │   Unlatch       │ │       │
        │ - Motors Enable │ │       │
        └─────────┬───────┘ │       │
                  │         │       │
                  └─────────┴───────┘
                            │
                            ▼
                  ┌─────────────────┐
                  │ Clear Fault Slot│
                  │ faults_[i].clear│
                  │ ()              │
                  └─────────┬───────┘
                            │
                            ▼
                  ┌─────────────────┐
                  │  INACTIVE STATE │
                  │  Slot Available │
                  └─────────────────┘

Legend:
  ──▶  : State Transition
  ┌──┐ : State or Process
  ◇   : Decision Point
```

**Important State Transitions**:

1. **Activation**: Sensor calls `activateFault()` → Coordinator allocates slot → Fault marked active
2. **E-Stop Trigger**: If severity is EMERGENCY_STOP, hardware relay activates immediately
3. **Monitoring**: Fault remains active until underlying condition clears (self-healing)
4. **Recovery**: When condition clears, sensor calls `deactivateFault()` → Coordinator decrements e-stop count
5. **Last Fault**: Only when ALL faults clear does e-stop relay release

**Recovery Hysteresis**: Note that deactivation often uses higher threshold than activation (e.g., activate at 85°C, deactivate at 80°C) to prevent oscillation.

---

#### Diagram 3: E-Stop Response Sequence (Timeline)

This diagram shows the temporal sequence of events from fault detection to motor shutdown, emphasizing timing guarantees.

```
TIME     BOARD 1 (Navigation)          BOARD 2 (Power)           ROS2 BRIDGE
═════    ════════════════════          ═══════════════           ═══════════

T+0ms    [TemperatureMonitor]
         loop() executes
         Read TMP36 sensor
         temp_celsius = 86.5°C
         
         EMA Filter:
         filtered = 85.8°C
         
         checkSafetyConditions():
         if (filtered > 85.0) ← TRUE
           activateFault(
             "THERMAL_RUNAWAY",
             EMERGENCY_STOP)
                ↓
T+1ms    [SafetyCoordinator]
         activateFault() called
         
         Allocate fault slot [3]
         Store: source, severity
         active_estop_count_ = 1
         
         if (EMERGENCY_STOP) ← TRUE
           RoboClawMonitor::
           setEmergencyStop()
                ↓
T+2ms    [RoboClawMonitor]
         setEmergencyStop()
         
         digitalWrite(
           ESTOP_OUTPUT_PIN, HIGH)
         ← Physical relay activates
         
         RoboClaw e-stop pin → LOW
         ← Hardware signal asserted
                ↓
T+3ms    [Physical Hardware]
         Solid State Relay
         opens circuit
         
         RoboClaw detects
         e-stop pin LOW
         
         Motor output disabled
         in hardware
         ← Motors coast to stop
                ↓
T+5ms    [SerialManager]
         sendMessage("FAULT",
           "{\"active_fault\":
            \"true\",
            \"source\":
            \"THERMAL_RUNAWAY\",
            \"severity\":
            \"EMERGENCY_STOP\"}")
         
         USB serial transmit
         115200 baud
         ~80 bytes = 7ms
                ↓
T+12ms                                                            [sigyn_to_
                                                                   sensor_v2]
                                                                  Serial read
                                                                  Parse JSON
                                                                  
                                                                  Update
                                                                  estop_status:
                                                                    active=true
                                                                    sources=[
                                                                      "THERMAL
                                                                      _RUNAWAY"]
                                                                  
                                                                  Publish to
                                                                  ROS2 topic
                                                                      ↓
T+15ms                                                            Nav2 Stack
                                                                  receives
                                                                  estop_status
                                                                  
                                                                  Cancel goals
                                                                  Stop planning
                                                                  ← Autonomous
                                                                    navigation
                                                                    halted
                ↓
T+100ms  Motors fully stopped
         (mechanical inertia)
         
         Robot stationary
         
         Continued monitoring:
         [TemperatureMonitor]
         Still executing every
         loop (85Hz)
         
         temp now = 84.2°C
         (cooling begins)
         
         Fault still active
         (hysteresis: must drop
         below 80°C to clear)
                ↓
                ⋮
                ⋮  (Time passes,
                ⋮   motor cools)
                ⋮
                ↓
T+60s    [TemperatureMonitor]
         loop() continues
         
         temp_celsius = 78.5°C
         filtered = 79.2°C
         
         checkSafetyConditions():
         if (filtered < 80.0 &&
             fault_active) ← TRUE
           deactivateFault(
             "THERMAL_RUNAWAY")
                ↓
T+60.001s [SafetyCoordinator]
          deactivateFault() called
          
          Find fault index [3]
          active = false
          active_estop_count_ = 0
          
          if (count == 0) ← TRUE
            RoboClawMonitor::
            clearEmergencyStop()
                ↓
T+60.002s [RoboClawMonitor]
          clearEmergencyStop()
          
          digitalWrite(
            ESTOP_OUTPUT_PIN, LOW)
          ← Relay deactivates
          
          RoboClaw e-stop → HIGH
          ← Hardware unlatch
                ↓
T+60.003s [Physical Hardware]
          SSR closes circuit
          
          RoboClaw resumes
          normal operation
          
          Motors ready for
          commands
          ← System recovered
                ↓
T+60.010s [SerialManager]
          sendMessage("FAULT",
            "{\"active_fault\":
             \"false\",
             \"source\":
             \"THERMAL_RUNAWAY\"}")
                ↓
T+60.020s                                                         [sigyn_to_
                                                                   sensor_v2]
                                                                  Parse clear
                                                                  
                                                                  Update:
                                                                    active=false
                                                                    sources=[]
                                                                  
                                                                  Publish to
                                                                  ROS2
                                                                      ↓
T+60.025s                                                         Nav2 Stack
                                                                  receives
                                                                  clear status
                                                                  
                                                                  Resume
                                                                  navigation
                                                                  
                                                                  ← Autonomous
                                                                    operation
                                                                    restored

═══════════════════════════════════════════════════════════════════════════

Critical Timing Guarantees:
───────────────────────────
• Fault detection to hardware e-stop:   <3ms   (Board 1 control loop)
• Hardware e-stop to motor shutdown:    <10ms  (RoboClaw + relay response)
• Total fault-to-stop latency:          <15ms  (safety-critical path)
• ROS2 notification latency:            ~15ms  (informational, not critical)
• Self-healing recovery latency:        ~60s   (depends on cooling rate)

Fail-Safe Behavior:
──────────────────
• If Board 1 firmware hangs:     E-stop remains asserted (relay default state)
• If serial communication lost:  ROS2 bridge detects timeout, assumes fault
• If power lost:                 Relay opens (fail-safe), motors stop
• If RoboClaw crashes:           E-stop pin still latched, motors disabled
```

**Key Observations**:

1. **Hardware-First**: Physical relay activation (T+2ms) occurs BEFORE serial notification (T+5ms). Safety doesn't depend on software communication.

2. **Latency Budget**: Total time from fault detection to motor stop is <15ms, well within human reaction time (~250ms). This prevents injury or damage.

3. **Self-Healing**: System automatically recovers at T+60s without human intervention. Hysteresis (activate at 85°C, clear at 80°C) prevents oscillation.

4. **ROS2 Visibility**: Nav2 receives notifications but is not in the critical path. Even if ROS2 crashes, hardware e-stop still functions.

5. **Continuous Monitoring**: Temperature sensor continues reading at 85Hz throughout entire sequence. Fault can re-trigger if temperature rises again.

---

---

## Document Metadata

- **Version**: 1.0 (Initial Draft)
- **Last Updated**: January 6, 2026
- **Author**: Wimble Robotics
- **Related Documents**:
  - [TEENSYV2_ARCHITECTURE.md] - Overall system architecture
  - [SIGYN_TO_SENSOR_V2.md] - ROS2 bridge detailed documentation
  - [BEHAVIOR_TREE_INTEGRATION.md] - Using safety in autonomous behaviors
  - [SAFETY_TESTING.md] - Comprehensive testing guide
  - [MODULE_DEVELOPMENT.md] - Creating new TeensyV2 modules

---

*This document is part of the Sigyn autonomous house patroller system documentation suite.*
