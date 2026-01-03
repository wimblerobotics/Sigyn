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

namespace {
String g_latest_twist;
bool g_has_new_twist = false;

String g_latest_sddir;
bool g_has_new_sddir = false;

String g_latest_sdfile;
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

void SerialManager::setLatestTwistCommand(const String& twist_data) {
  g_latest_twist = twist_data;
  g_has_new_twist = true;
}

String SerialManager::getLatestTwistCommand() {
  return g_latest_twist;
}

bool SerialManager::hasNewTwistCommand() {
  if (g_has_new_twist) {
    g_has_new_twist = false;
    return true;
  }
  return false;
}

void SerialManager::setLatestSDDirCommand(const String& sddir_data) {
  g_latest_sddir = sddir_data;
  g_has_new_sddir = true;
}

String SerialManager::getLatestSDDirCommand() {
  return g_latest_sddir;
}

bool SerialManager::hasNewSDDirCommand() {
  if (g_has_new_sddir) {
    g_has_new_sddir = false;
    return true;
  }
  return false;
}

void SerialManager::setLatestSDFileCommand(const String& sdfile_data) {
  g_latest_sdfile = sdfile_data;
  g_has_new_sdfile = true;
}

String SerialManager::getLatestSDFileCommand() {
  return g_latest_sdfile;
}

bool SerialManager::hasNewSDFileCommand() {
  if (g_has_new_sdfile) {
    g_has_new_sdfile = false;
    return true;
  }
  return false;
}

} // namespace sigyn_teensy
