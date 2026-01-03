// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file config.h
 * @brief Board-specific configuration definitions for TeensyV2 system
 *
 * This file centralizes all board-specific feature configurations, making it
 * easy to manage capabilities across multiple boards without scattered
 * conditional compilation directives throughout the codebase.
 *
 * The configuration uses BOARD_ID (defined in platformio.ini) to determine
 * which features are enabled for each specific board, allowing for clean
 * separation of concerns and easy addition of new boards.
 *
 * @author Wimble Robotics
 * @date 2025
 * @version 2.0
 */

#pragma once

 // Ensure BOARD_ID is defined (should come from platformio.ini)
#ifndef BOARD_ID
#error "BOARD_ID must be defined in platformio.ini build flags"
#endif

// Override certain features during unit testing
#ifdef UNIT_TEST
// Disable RoboClaw dependencies during unit testing
#define UNIT_TEST_DISABLE_ROBOCLAW 1
#endif

// =============================================================================
// BOARD 1: Navigation and Safety Board
// =============================================================================
#if BOARD_ID == 1
#define BOARD_HAS_SD_LOGGING     1
#define BOARD_HAS_MOTOR_CONTROL  1
#define BOARD_HAS_VL53L0X        1
#define BOARD_HAS_TEMPERATURE    1
#define BOARD_HAS_PERFORMANCE    1
#define BOARD_HAS_SAFETY         1
#define BOARD_HAS_BATTERY        0
#define BOARD_HAS_IMU            0

// =============================================================================
// BOARD 2: Power and Sensor Board  
// =============================================================================
#elif BOARD_ID == 2
#define BOARD_HAS_SD_LOGGING     1
#define BOARD_HAS_MOTOR_CONTROL  0
#define BOARD_HAS_VL53L0X        0
#define BOARD_HAS_TEMPERATURE    0
#define BOARD_HAS_PERFORMANCE    1
#define BOARD_HAS_SAFETY         1
#define BOARD_HAS_BATTERY        1
#define BOARD_HAS_IMU            1

// =============================================================================
// BOARD 3: Future Expansion Board
// =============================================================================
#elif BOARD_ID == 3
#define BOARD_HAS_SD_LOGGING     1
#define BOARD_HAS_MOTOR_CONTROL  1
#define BOARD_HAS_VL53L0X        0
#define BOARD_HAS_TEMPERATURE    0
#define BOARD_HAS_PERFORMANCE    1
#define BOARD_HAS_SAFETY         0
#define BOARD_HAS_BATTERY        0
#define BOARD_HAS_IMU            0

// =============================================================================
// BOARD 99: RoboClaw Timing Test (standalone)
// =============================================================================
#elif BOARD_ID == 99
#define BOARD_HAS_SD_LOGGING     0
#define BOARD_HAS_MOTOR_CONTROL  1
#define BOARD_HAS_VL53L0X        0
#define BOARD_HAS_TEMPERATURE    0
#define BOARD_HAS_PERFORMANCE    0
#define BOARD_HAS_SAFETY         0
#define BOARD_HAS_BATTERY        0
#define BOARD_HAS_IMU            0

// =============================================================================
// Unknown Board - Error Condition
// =============================================================================
#else
#error "Unknown BOARD_ID. Please define configuration for this board."
#endif

// =============================================================================
// Feature Validation
// =============================================================================
// Ensure at least one board has each critical capability
#if !defined(BOARD_HAS_SD_LOGGING) || !defined(BOARD_HAS_PERFORMANCE) || !defined(BOARD_HAS_SAFETY)
#error "All boards must define SD_LOGGING, PERFORMANCE, and SAFETY capabilities"
#endif

// =============================================================================
// Convenience Macros
// =============================================================================
// These provide readable conditional compilation throughout the codebase
#define ENABLE_SD_LOGGING    (BOARD_HAS_SD_LOGGING == 1)
#define ENABLE_VL53L0X       (BOARD_HAS_VL53L0X == 1)
#define ENABLE_TEMPERATURE   (BOARD_HAS_TEMPERATURE == 1)
#define ENABLE_PERFORMANCE   (BOARD_HAS_PERFORMANCE == 1)
#define ENABLE_SAFETY        (BOARD_HAS_SAFETY == 1)
#define ENABLE_BATTERY       (BOARD_HAS_BATTERY == 1)
#define ENABLE_IMU           (BOARD_HAS_IMU == 1)

// =============================================================================
// Board-Specific Configuration Parameters
// =============================================================================

// --- Communication Settings ---
#if BOARD_ID == 1
#define BOARD_SERIAL_BAUD_RATE        1000000
#define BOARD_SERIAL_TIMEOUT_MS       5000
#define BOARD_SERIAL_WAIT_MS          3000
#elif BOARD_ID == 2
#define BOARD_SERIAL_BAUD_RATE        1000000
#define BOARD_SERIAL_TIMEOUT_MS       5000
#define BOARD_SERIAL_WAIT_MS          5000
#elif BOARD_ID == 3
#define BOARD_SERIAL_BAUD_RATE        1000000
#define BOARD_SERIAL_TIMEOUT_MS       5000
#define BOARD_SERIAL_WAIT_MS          3000
#elif BOARD_ID == 99
#define BOARD_SERIAL_BAUD_RATE        1000000
#define BOARD_SERIAL_TIMEOUT_MS       5000
#define BOARD_SERIAL_WAIT_MS          3000
#endif

// --- Inter-Board E-Stop GPIO Pins ---
#if BOARD_ID == 1
#ifdef UNIT_TEST_DISABLE_ROBOCLAW
#define CONTROLS_ROBOCLAW_ESTOP_PIN   0   ///< Disabled for unit testing
#else
#define CONTROLS_ROBOCLAW_ESTOP_PIN   1   ///< Board 1 controls RoboClaw E-stop
#endif
#define INTER_BOARD_SIGNAL_OUTPUT_PIN 10  ///< Pin to signal other boards
#define PIN_SAFETY_IN_BOARD2          11  ///< Pin to receive signals from Board 2
#define PIN_SAFETY_IN_BOARD3          12  ///< Pin to receive signals from Board 3
#define HARDWARE_ESTOP_INPUT_PIN      2   ///< Hardware E-stop button input
#define ESTOP_OUTPUT_PIN              30  ///< E-stop relay output
#define PIN_RELAY_ROBOCLAW_POWER      31  ///< Relay to cut power to RoboClaw
#define PIN_RELAY_MAIN_BATTERY        32  ///< Relay to cut main battery power
#elif BOARD_ID == 2
#define CONTROLS_ROBOCLAW_ESTOP_PIN   0   ///< Board 1 controls RoboClaw E-stop
#define PIN_SAFETY_OUT_TO_MASTER      10  ///< Pin to signal Board 1
#define HARDWARE_ESTOP_INPUT_PIN      2   ///< Hardware E-stop button input (if any)
#define ESTOP_OUTPUT_PIN              30   ///< E-stop relay output (not used on Board 2)
#elif BOARD_ID == 3
#define CONTROLS_ROBOCLAW_ESTOP_PIN   0   ///< Board 1 controls RoboClaw E-stop
#define PIN_SAFETY_OUT_TO_MASTER      10  ///< Pin to signal Board 1
#define HARDWARE_ESTOP_INPUT_PIN      2   ///< Hardware E-stop button input (if any)
#define ESTOP_OUTPUT_PIN              30   ///< E-stop relay output (not used on Board 3)
#elif BOARD_ID == 99
#define CONTROLS_ROBOCLAW_ESTOP_PIN   1   ///< Timing test board controls RoboClaw E-stop
#define INTER_BOARD_SIGNAL_OUTPUT_PIN 10  ///< Unused in test, keep defined
#define PIN_SAFETY_IN_BOARD2          11  ///< Unused in test, keep defined
#define PIN_SAFETY_IN_BOARD3          12  ///< Unused in test, keep defined
#define HARDWARE_ESTOP_INPUT_PIN      2   ///< Hardware E-stop button input
#define ESTOP_OUTPUT_PIN              30  ///< E-stop relay output
#define PIN_RELAY_ROBOCLAW_POWER      31  ///< Relay to cut power to RoboClaw
#define PIN_RELAY_MAIN_BATTERY        32  ///< Relay to cut main battery power
#endif

// --- Performance Monitoring Thresholds ---
#if BOARD_ID == 1
  // Board 1: Navigation and Safety - Higher performance requirements
#define BOARD_MAX_MODULE_TIME_MS          2.0f   ///< Maximum module execution time
#define BOARD_MIN_LOOP_FREQUENCY_HZ       50.0f  ///< Minimum acceptable loop frequency
#define BOARD_CRITICAL_EXECUTION_TIME_US  10000  ///< Critical execution time warning
#define BOARD_CRITICAL_FREQUENCY_HZ       50.0f  ///< Critical frequency warning
#define BOARD_SAFETY_EXECUTION_TIME_US    20000  ///< Safety trigger execution time
#elif BOARD_ID == 2
  // Board 2: Power and Sensors - More relaxed performance requirements
#define BOARD_MAX_MODULE_TIME_MS          3.0f   ///< Maximum module execution time
#define BOARD_MIN_LOOP_FREQUENCY_HZ       20.0f  ///< Minimum acceptable loop frequency
#define BOARD_CRITICAL_EXECUTION_TIME_US  15000  ///< Critical execution time warning
#define BOARD_CRITICAL_FREQUENCY_HZ       20.0f  ///< Critical frequency warning
#define BOARD_SAFETY_EXECUTION_TIME_US    25000  ///< Safety trigger execution time
#elif BOARD_ID == 3
  // Board 3: Future Expansion - Basic performance requirements
#define BOARD_MAX_MODULE_TIME_MS          5.0f   ///< Maximum module execution time
#define BOARD_MIN_LOOP_FREQUENCY_HZ       10.0f  ///< Minimum acceptable loop frequency
#define BOARD_CRITICAL_EXECUTION_TIME_US  20000  ///< Critical execution time warning
#define BOARD_CRITICAL_FREQUENCY_HZ       10.0f  ///< Critical frequency warning
#define BOARD_SAFETY_EXECUTION_TIME_US    30000  ///< Safety trigger execution time
#elif BOARD_ID == 99
  // Board 99: RoboClaw timing test - focus on motor driver comms
#define BOARD_MAX_MODULE_TIME_MS          2.0f
#define BOARD_MIN_LOOP_FREQUENCY_HZ       50.0f
#define BOARD_CRITICAL_EXECUTION_TIME_US  10000
#define BOARD_CRITICAL_FREQUENCY_HZ       50.0f
#define BOARD_SAFETY_EXECUTION_TIME_US    20000
#endif

// --- Safety Monitoring Intervals ---
#if BOARD_ID == 1
#define SAFETY_CHECK_INTERVAL_US      100000  ///< Safety check every 100ms (10Hz)
#define VL53L0X_CHECK_INTERVAL_US     100000  ///< VL53L0X check every 100ms (10Hz)
#elif BOARD_ID == 2
#define SAFETY_CHECK_INTERVAL_US      200000  ///< Safety check every 200ms (5Hz)
#define VL53L0X_CHECK_INTERVAL_US     200000  ///< VL53L0X check every 200ms (5Hz)
#elif BOARD_ID == 3
#define SAFETY_CHECK_INTERVAL_US      500000  ///< Safety check every 500ms (2Hz)
#define VL53L0X_CHECK_INTERVAL_US     500000  ///< VL53L0X check every 500ms (2Hz)
#elif BOARD_ID == 99
#define SAFETY_CHECK_INTERVAL_US      100000  ///< Safety check every 100ms (10Hz)
#define VL53L0X_CHECK_INTERVAL_US     100000  ///< Unused in test, keep defined
#endif
