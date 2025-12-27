# Sigyn Safety System Overview

## Introduction

This document provides a technical overview of the safety system architecture for the Sigyn robot. It is intended for developers who want to understand the design intent, the implementation details, and how the various software components interact to ensure safe operation.

The safety system is designed as a **distributed, fail-safe, multi-layered** architecture spanning the embedded microcontrollers (Teensy 4.1) and the high-level ROS 2 PC.

## Architecture

The system is divided into two main domains:
1.  **Embedded Domain (TeensyV2)**: Handles real-time safety, hardware monitoring, and immediate E-stop execution.
2.  **ROS 2 Domain (sigyn_to_sensor_v2)**: Handles high-level safety coordination, logging, and integration with the navigation stack.

### 1. Embedded Domain (TeensyV2)

The embedded software runs on three custom PCBs, each powered by a Teensy 4.1. The code is structured around a modular framework.

#### Key Components

*   **`Module` Class (`common/core/module.h`)**:
    *   The base class for all subsystems.
    *   **Intent**: Enforces a standard interface for safety.
    *   **Mechanism**: Every module must implement `isUnsafe()`. This method is polled in every loop cycle. If it returns `true`, the system triggers an emergency stop.

*   **`SafetyCoordinator` (`modules/safety/safety_coordinator.cpp`)**:
    *   **Intent**: Central brain for safety on each board.
    *   **Mechanism**:
        *   Polls `Module::isAnyModuleUnsafe()`.
        *   Monitors the physical E-stop button (Board 1).
        *   Monitors the inter-board safety signal (Boards 1 & 2).
        *   If any condition is met, it asserts the hardware E-stop line (cutting power to motors) and sends a CRITICAL message to ROS.

*   **`SerialManager` (`common/core/serial_manager.cpp`)**:
    *   **Intent**: Robust communication channel.
    *   **Mechanism**: Handles sending JSON-formatted diagnostic and safety messages to the PC. It prioritizes safety messages.

#### Board Roles

*   **Board 1 (Navigation)**: Runs `RoboClawMonitor` (motor safety), `VL53L0XMonitor` (obstacle safety), and `SafetyCoordinator`.
*   **Board 2 (Power)**: Runs `BatteryMonitor` (voltage/current monitoring) and `SafetyCoordinator`.
*   **Board 3 (Gripper)**: Runs `StepperMotor` control. *(Note: Currently lacks SafetyCoordinator integration)*.

### 2. ROS 2 Domain (sigyn_to_sensor_v2)

This package acts as the bridge between the hardware and the ROS 2 ecosystem.

#### Key Components

*   **`TeensyBridge` (`src/teensy_bridge.cpp`)**:
    *   **Intent**: Translate serial data to ROS topics and vice versa.
    *   **Mechanism**:
        *   Reads serial streams from all connected boards.
        *   Parses messages using `MessageParser`.
        *   Publishes `std_msgs/Bool` to `/estop_status` when a safety violation occurs.
        *   Listens to `/estop_command` to trigger software E-stops.

*   **`MessageParser` (`src/message_parser.cpp`)**:
    *   **Intent**: robustly decode incoming data.
    *   **Mechanism**: Handles the `TYPE:JSON` format, extracting board IDs and payload data.

## Data Flow Example: Motor Overcurrent

1.  **Detection**: `RoboClawMonitor` on Board 1 detects current > 20A.
2.  **Flagging**: `RoboClawMonitor::isUnsafe()` returns `true`.
3.  **Coordination**: `SafetyCoordinator` loop calls `isAnyModuleUnsafe()`, sees `true`.
4.  **Action**:
    *   `SafetyCoordinator` sets state to `EMERGENCY_STOP`.
    *   Asserts hardware E-stop pin (HIGH).
    *   Calls `SerialManager::sendDiagnosticMessage("CRITICAL", ...)` with reason.
5.  **Transmission**: `SerialManager` sends `DIAG1:{"level":"CRITICAL", "message":"...Motor 1 overcurrent..."}` over USB.
6.  **Bridging**: `TeensyBridge` receives message, parses it.
7.  **Notification**: `TeensyBridge` publishes `true` to `/estop_status`.
8.  **Reaction**: ROS 2 Navigation stack (Nav2) sees `/estop_status`, cancels goals, and halts planning.

## File Map

| File Path | Role | Safety Contribution |
| :--- | :--- | :--- |
| `TeensyV2/common/core/module.h` | Interface | Defines `isUnsafe()` contract. |
| `TeensyV2/modules/safety/safety_coordinator.cpp` | Manager | Aggregates safety signals, triggers hardware stop. |
| `TeensyV2/modules/roboclaw/roboclaw_monitor.cpp` | Monitor | Implements motor safety (runaway, overcurrent). |
| `TeensyV2/modules/battery/battery_monitor.cpp` | Monitor | Monitors power (implementation pending). |
| `TeensyV2/common/core/serial_manager.cpp` | Comms | Transmits safety alerts to PC. |
| `sigyn_to_sensor_v2/src/teensy_bridge.cpp` | Bridge | Publishes ROS safety topics, handles E-stop cmds. |

## Design Philosophy

The system is designed to be **fail-safe**. If the software crashes, the watchdog (if enabled) or the hardware E-stop circuit should default to a safe state. If the serial connection is lost, the `RoboClawMonitor` detects the timeout and stops the motors. The distributed nature ensures that a failure on the PC side does not prevent the embedded system from stopping the robot to protect hardware.
