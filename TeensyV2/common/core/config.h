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
  #define BOARD_HAS_ROBOCLAW       1
  #define BOARD_HAS_BATTERY        0
  #define BOARD_HAS_IMU            0

// =============================================================================
// BOARD 2: Power and Sensor Board  
// =============================================================================
#elif BOARD_ID == 2
  #define BOARD_HAS_SD_LOGGING     1
  #define BOARD_HAS_MOTOR_CONTROL  0
  #define BOARD_HAS_VL53L0X        0
  #define BOARD_HAS_TEMPERATURE    1
  #define BOARD_HAS_PERFORMANCE    1
  #define BOARD_HAS_SAFETY         1
  #define BOARD_HAS_ROBOCLAW       0
  #define BOARD_HAS_BATTERY        1
  #define BOARD_HAS_IMU            1

// =============================================================================
// BOARD 3: Future Expansion Board
// =============================================================================
#elif BOARD_ID == 3
  #define BOARD_HAS_SD_LOGGING     1
  #define BOARD_HAS_MOTOR_CONTROL  0
  #define BOARD_HAS_VL53L0X        0
  #define BOARD_HAS_TEMPERATURE    0
  #define BOARD_HAS_PERFORMANCE    1
  #define BOARD_HAS_SAFETY         1
  #define BOARD_HAS_ROBOCLAW       0
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
#define ENABLE_MOTOR_CONTROL (BOARD_HAS_MOTOR_CONTROL == 1)
#define ENABLE_VL53L0X       (BOARD_HAS_VL53L0X == 1)
#define ENABLE_TEMPERATURE   (BOARD_HAS_TEMPERATURE == 1)
#define ENABLE_PERFORMANCE   (BOARD_HAS_PERFORMANCE == 1)
#define ENABLE_SAFETY        (BOARD_HAS_SAFETY == 1)
#define ENABLE_ROBOCLAW      (BOARD_HAS_ROBOCLAW == 1)
#define ENABLE_BATTERY       (BOARD_HAS_BATTERY == 1)
#define ENABLE_IMU           (BOARD_HAS_IMU == 1)
