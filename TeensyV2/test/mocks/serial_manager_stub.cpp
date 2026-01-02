// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 Wimblerobotics
// https://github.com/wimblerobotics/Sigyn

/**
 * @file serial_manager_stub.cpp
 * @brief Stub implementation of SerialManager for unit testing
 *
 * This stub allows code that uses SerialManager to compile and link,
 * but discards all output. For Phase 1, we just need the functions
 * to exist so temperature_monitor.cpp will link.
 */

#include "../../common/core/serial_manager.h"

namespace sigyn_teensy {

// Stub singleton
SerialManager& SerialManager::getInstance() {
  static SerialManager instance;
  return instance;
}

SerialManager::SerialManager() {}

void SerialManager::sendMessage(const char*, const char*) {
  // Discard output in tests
}

void SerialManager::sendDiagnosticMessage(const char*, const char*, const char*) {
  // Discard output in tests
}

} // namespace sigyn_teensy
