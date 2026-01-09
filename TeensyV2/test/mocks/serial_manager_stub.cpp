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

#include <cstdio>

namespace sigyn_teensy {

namespace {
char g_latest_twist[SerialManager::kMaxMessageLength] = {0};
bool g_has_new_twist = false;

char g_latest_sddir[SerialManager::kMaxMessageLength] = {0};
bool g_has_new_sddir = false;

char g_latest_sdfile[SerialManager::kMaxMessageLength] = {0};
bool g_has_new_sdfile = false;
}  // namespace

// Stub singleton
SerialManager& SerialManager::getInstance() {
  static SerialManager instance;
  return instance;
}

SerialManager::SerialManager() {}

void SerialManager::initialize(uint32_t) {
  // No-op for unit tests
}

void SerialManager::sendMessage(const char*, const char*) {
  // Discard output in tests
}

void SerialManager::sendDiagnosticMessage(const char*, const char*, const char*) {
  // Discard output in tests
}

void SerialManager::processIncomingMessages() {
  // No-op for unit tests
}

void SerialManager::setLatestTwistCommand(const char* twist_data) {
  snprintf(g_latest_twist, sizeof(g_latest_twist), "%s", twist_data ? twist_data : "");
  g_has_new_twist = true;
}

const char* SerialManager::getLatestTwistCommand() const { return g_latest_twist; }

bool SerialManager::hasNewTwistCommand() {
  if (g_has_new_twist) {
    g_has_new_twist = false;
    return true;
  }
  return false;
}

void SerialManager::setLatestSDDirCommand(const char* sddir_data) {
  snprintf(g_latest_sddir, sizeof(g_latest_sddir), "%s", sddir_data ? sddir_data : "");
  g_has_new_sddir = true;
}

const char* SerialManager::getLatestSDDirCommand() const { return g_latest_sddir; }

bool SerialManager::hasNewSDDirCommand() {
  if (g_has_new_sddir) {
    g_has_new_sddir = false;
    return true;
  }
  return false;
}

void SerialManager::setLatestSDFileCommand(const char* sdfile_data) {
  snprintf(g_latest_sdfile, sizeof(g_latest_sdfile), "%s", sdfile_data ? sdfile_data : "");
  g_has_new_sdfile = true;
}

const char* SerialManager::getLatestSDFileCommand() const { return g_latest_sdfile; }

bool SerialManager::hasNewSDFileCommand() {
  if (g_has_new_sdfile) {
    g_has_new_sdfile = false;
    return true;
  }
  return false;
}

} // namespace sigyn_teensy
