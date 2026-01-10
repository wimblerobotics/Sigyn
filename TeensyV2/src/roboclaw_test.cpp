// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file roboclaw_test.cpp
 * @brief Minimal RoboClaw timing-test firmware entrypoint.
 */

#include <Arduino.h>

#include "common/core/config.h"
#include "common/core/module.h"
#include "common/core/serial_manager.h"
#include "modules/roboclaw/roboclaw_monitor.h"

using namespace sigyn_teensy;

static SerialManager* serial_manager = nullptr;
static RoboClawMonitor* roboclaw_monitor = nullptr;

void setup() {
  uint32_t start_time = millis();
  Serial.begin(BOARD_SERIAL_BAUD_RATE);
  while (!Serial && (millis() - start_time) < BOARD_SERIAL_WAIT_MS) {
    // Wait briefly for USB serial
  }

  // RoboClaw serial port
  RoboClawConfig config;
  Serial7.begin(config.baud_rate);

  serial_manager = &SerialManager::getInstance();
  roboclaw_monitor = &RoboClawMonitor::getInstance();

  Module::setupAll();

  serial_manager->sendDiagnosticMessage("INFO", "roboclaw_test", "Initialized");
}

void loop() {
  Module::loopAll();
}
