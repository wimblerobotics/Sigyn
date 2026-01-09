# TeensyV2 Documentation Guide

This page provides a quick reference for navigating the complete TeensyV2 documentation suite.

## 📋 Complete Documentation Set

### 1. **ARCHITECTURE.md** — Start Here
**Best for**: First-time understanding of the system

Covers:
- System design philosophy (modular, real-time, safety-first)
- Hardware overview (dual Teensy 4.1, sensor arrays)
- Module system framework (how modules work, lifecycle)
- Core components (SerialManager, PerformanceMonitor, SafetyCoordinator)
- Each module's role and interaction
- Performance characteristics with actual metrics from logs
- Integration points with ROS2

**Read this if you**:
- Are new to TeensyV2
- Want to understand the big picture
- Need to add a new module
- Are debugging system-wide issues

---

### 2. **Message_Formats.md** — Protocol Reference
**Best for**: Understanding serial communication and data flow

Covers:
- JSON message format convention
- All message types with examples (VL53L0X1, ROBOCLAW1, TEMPERATURE1, etc.)
- Command messages (TWIST, ESTOP)
- Diagnostic messages (DIAG1, PERF1, FAULT1)
- Real-world examples from actual system logs
- Bandwidth & performance analysis

**Read this if you**:
- Are integrating ROS2 with TeensyV2
- Want to parse/send serial messages
- Are debugging communication issues
- Need to understand data types and ranges

**Quick Reference**: All message types in one place with field-by-field breakdown

---

### 3. **Safety_System.md** — Critical System Design
**Best for**: Understanding fault handling and emergency stops

Covers:
- Safety philosophy (defense in depth, fail-safe defaults, transparent logging)
- Architecture (SafetyCoordinator, fault state machine)
- 7 fault sources with thresholds & recovery mechanisms
  - Motor overcurrent
  - RoboClaw timeout
  - Sensor degradation
  - Temperature
  - Battery voltage
  - Performance violations
  - Manual E-stop
- Fault bitmask (simultaneous fault handling)
- Motor enable/disable logic
- ROS2 integration (fault status topic)
- Recovery flowchart with timelines

**Read this if you**:
- Need to understand why motors stopped
- Are implementing safety logic
- Want to tune fault thresholds
- Are debugging fault events

**Critical for**: Safety-critical applications, regulations compliance

---

### 4. **Module_Reference.md** — Detailed Component Reference
**Best for**: Diving deep into individual modules

Per-module details:
- **RoboClawMonitor**: Motor RPM encoding, current thresholds, timeout handling
- **VL53L0XMonitor**: Sensor layout, degradation detection, obstacle thresholds
- **TemperatureMonitor**: Thermal characteristics, hysteresis, cooling times
- **BatteryMonitor**: Voltage thresholds, battery chemistry
- **PerformanceMonitor**: Loop frequency, timing budgets, violation detection
- **SDLogger**: File format, write rates, statistics
- **SafetyCoordinator**: Fault bits, motor enable logic

Each module includes:
- Location in codebase
- Compilation flags
- Key parameters (thresholds, rates)
- Fault codes with recovery info
- Actual log examples

**Read this if you**:
- Need to adjust a module's threshold
- Are troubleshooting a specific module
- Want to understand fault codes
- Are analyzing performance data

---

### 5. **Testing.md** — Debugging & QA Guide
**Best for**: Troubleshooting, testing, and validation

Covers:
- Unit testing framework (dependency injection, mocks)
- Running tests on PC and on-device
- Integration testing procedures
- Live system debugging via serial monitor
- Performance monitoring (interpreting PERF1 messages)
- Log retrieval and analysis with scripts
- 10 common issues with solutions
  - RoboClaw not responding
  - VL53L0X degradation
  - Motor overcurrent
  - Temperature shutdown
  - Battery warnings
  - Performance violations
  - SD card failures
  - Serial communication issues
- Troubleshooting checklist

**Read this if you**:
- Are experiencing system issues
- Want to run tests
- Need to debug a module
- Are analyzing SD card logs

**Practical**: Includes bash commands and Python scripts for analysis

---

## 🎯 Quick Navigation by Task

### **"Why did the robot stop?"**
→ [Safety_System.md](Safety_System.md) — Check fault sources and recovery times

### **"How do I talk to Teensy?"**
→ [Message_Formats.md](Message_Formats.md) — See message examples and protocol

### **"System is slow, what's happening?"**
→ [Module_Reference.md](Module_Reference.md) — Check PerformanceMonitor section
→ [Testing.md](Testing.md) — Performance monitoring subsection

### **"Motor current seems wrong"**
→ [Module_Reference.md](Module_Reference.md) — RoboClawMonitor section for thresholds
→ [Testing.md](Testing.md) — Overcurrent issue troubleshooting

### **"Temperature keeps shutting down motors"**
→ [Module_Reference.md](Module_Reference.md) — TemperatureMonitor parameters
→ [Safety_System.md](Safety_System.md) — Temperature fault & recovery times

### **"I need to add a new sensor"**
→ [ARCHITECTURE.md](ARCHITECTURE.md) — Module system framework
→ Look at existing module (e.g., VL53L0XMonitor) as template

### **"How do I test my changes?"**
→ [Testing.md](Testing.md) — Unit testing & integration testing sections

### **"What's using 50% of the loop time?"**
→ [ARCHITECTURE.md](ARCHITECTURE.md) — Performance metrics section
→ [Message_Formats.md](Message_Formats.md) — PERF1 message breakdown

### **"Sensor readings look wrong"**
→ [Module_Reference.md](Module_Reference.md) — Sensor-specific section (VL53L0X, Temperature, etc.)
→ [Testing.md](Testing.md) — Hardware testing procedures

---

## 📚 Documentation Relationship Map

```
ARCHITECTURE.md (foundation)
  ├─→ Message_Formats.md (what data flows through system)
  ├─→ Safety_System.md (how system protects itself)
  ├─→ Module_Reference.md (specific module details)
  └─→ Testing.md (validation & debugging)

Message_Formats.md
  ├─ Shows all message types
  ├─ Examples reference real system logs
  └─ Links to Module_Reference for interpretation

Safety_System.md
  ├─ Lists 7 fault sources
  └─ Each maps to module in Module_Reference.md

Module_Reference.md
  ├─ 7 modules documented
  ├─ Each references fault codes from Safety_System.md
  └─ Actual log examples from Message_Formats.md

Testing.md
  ├─ References all modules for debugging
  ├─ Uses message formats from Message_Formats.md
  └─ Provides troubleshooting for Safety_System.md faults
```

---

## 🔧 For Different Roles

### **Software Developer (Adding Features)**
1. Start: [ARCHITECTURE.md](ARCHITECTURE.md)
2. Understand module system
3. Look at similar module for template
4. Read [Testing.md](Testing.md) for testing approach
5. Reference [Module_Reference.md](Module_Reference.md) for specific details

### **Integration Engineer (ROS2 Bridge)**
1. Start: [Message_Formats.md](Message_Formats.md) — see all possible messages
2. Look at [ARCHITECTURE.md](ARCHITECTURE.md) — understand ROS2 integration section
3. Reference [Message_Formats.md](Message_Formats.md) examples for parsing

### **Troubleshooter (Debugging Issues)**
1. Check symptoms in [Testing.md](Testing.md) — common issues section
2. If fault-related: [Safety_System.md](Safety_System.md)
3. If module-specific: [Module_Reference.md](Module_Reference.md)
4. Use [Testing.md](Testing.md) debugging tools (scripts, serial inspection)

### **Safety Engineer (Compliance & Certification)**
1. Core: [Safety_System.md](Safety_System.md)
2. Understanding: [ARCHITECTURE.md](ARCHITECTURE.md) — system design philosophy
3. Validation: [Testing.md](Testing.md) — test procedures
4. Reference: [Module_Reference.md](Module_Reference.md) — fault thresholds & timeouts

### **Hardware Engineer (PCB/Firmware Integration)**
1. Overview: [ARCHITECTURE.md](ARCHITECTURE.md) — hardware section
2. Data flow: [Message_Formats.md](Message_Formats.md)
3. Modules: [Module_Reference.md](Module_Reference.md) — sensor specs & parameters
4. Testing: [Testing.md](Testing.md) — hardware test procedures

---

## 🚀 Getting Started Workflow

### First Time Using TeensyV2?

1. **Understand the system** (15 min)
   - Read [ARCHITECTURE.md](ARCHITECTURE.md) Overview & Big Picture Architecture sections

2. **Understand data flow** (10 min)
   - Skim [Message_Formats.md](Message_Formats.md) to see what messages exist
   - Look at "Examples from Live System" section

3. **Understand safety** (10 min)
   - Skim [Safety_System.md](Safety_System.md) for fault overview
   - Know the 7 fault sources and how motors stop

4. **Run a test** (5 min)
   - Follow [Testing.md](Testing.md) → "Serial Monitor Inspection" section
   - Connect serial monitor and observe live messages

5. **Familiarize with debugging** (5 min)
   - Bookmark [Testing.md](Testing.md) troubleshooting section
   - Know how to interpret error messages

**Total time**: ~45 minutes for complete system understanding

---

## 📖 Notation & Conventions

Across all documents:

- **File paths**: `/module/file.cpp` (absolute, from TeensyV2 root)
- **Code blocks**: Language specified (cpp, json, bash, python)
- **Message examples**: Actual data from system logs, not synthetic
- **Timings**: milliseconds (ms) unless otherwise stated
- **Fault codes**: Hexadecimal with bit definitions (0x0001 = bit 0, etc.)
- **Hyperlinks**: Cross-references between documents use `[Text](File.md)` format

---

## ❓ FAQ

**Q: Which document should I read first?**
A: [ARCHITECTURE.md](ARCHITECTURE.md) — gives you the foundation for everything else.

**Q: I see an error message, where do I look?**
A: First, [Safety_System.md](Safety_System.md) if it's a fault. Then, [Module_Reference.md](Module_Reference.md) for the specific module. If still stuck, [Testing.md](Testing.md) troubleshooting section.

**Q: Can I modify fault thresholds?**
A: Yes, see [Module_Reference.md](Module_Reference.md) for the specific module and threshold location.

**Q: Where do I find actual performance numbers?**
A: [ARCHITECTURE.md](ARCHITECTURE.md) Performance section and [Message_Formats.md](Message_Formats.md) PERF1 section.

**Q: How do I add a new module?**
A: See [ARCHITECTURE.md](ARCHITECTURE.md) Module System section, then follow the pattern from an existing module.

**Q: What's the expected behavior of E-stop?**
A: [Safety_System.md](Safety_System.md) Safety Actions section and [Message_Formats.md](Message_Formats.md) for ESTOP message format.

---

## 📝 Document Maintenance

These documents describe the **current state** of TeensyV2 as of the latest update. However:

- **Implementation changes** may make specific parameter values stale
- **Always verify against source code** before making critical changes
- **Performance metrics** are from actual system logs but may vary based on compile flags
- **If you find inaccuracies**, please update both docs and code to keep them in sync

---

## 🔗 Related Resources

**In Sigyn Workspace**:
- `sigyn_to_sensor_v2/` — ROS2 bridge connecting TeensyV2 to host
- `base/launch/sigyn.launch.py` — Full system launch configuration
- `TeensyV2/src/` — Source code for all modules

**Manufacturer Documentation**:
- [RoboClaw 2x15a User Manual](https://www.basicmicro.com/)
- [Teensy 4.1 Documentation](https://www.pjrc.com/teensy/)
- [VL53L0X Datasheet](https://www.st.com/en/imaging-and-photonics/vl53l0x.html)
- [TCA9548A I²C Multiplexer](https://www.ti.com/product/TCA9548A)

---

**Last Updated**: January 8, 2026  
**Version**: 2.0 (Documentation Suite)
