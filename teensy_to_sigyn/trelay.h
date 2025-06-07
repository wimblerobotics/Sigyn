/**
 * MIT License
 * Copyright 2021 by Michael Wimble
 *
 * Control an 8-channel relay board.
 */

#pragma once

#include <stdint.h>

#include "tmodule.h"

class TRelay : TModule {
 public:
  // The purpose of each relay (in order on the board)
  typedef enum {
    kUnused0,
    kUnused1,
    kUnused2,
    kUnused3,
    kUnused4,
    kMotorEStop,
    kNumberDevices  // Number of relays on the board
  } TRelayDevice;

  // Is in the power-on state?
  bool IsPoweredOn(TRelayDevice device);

  // Power the relay on.
  void PowerOff(TRelayDevice device);

  // Power the relay off.
  void PowerOn(TRelayDevice device);

  // Singleton constructor.
  static TRelay& singleton();

 protected:
  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "RLAY"; }

  // From TModule.
  void setup();

 private:
  // GPIO pins to control the relays.
  typedef enum {
    kUnused0Pin = 0,
    kUnused1Pin = 1,
    kUnused2Pin = 2,
    kUnused3Pin = 3,
    kUnused4Pin = 4,
    kMotorEStopPin = 5
  } TPinNumbers;

  // How long (in milliseconds) to trip the relay to simulate a reset push.
  static const uint32_t kResetDurationMs = 1000;

  // Private constructor.
  TRelay();

  // A non-zero value indicates the relay is powered-on.
  // Relays which are momentary switches will use the value here to
  // determine when to reset the relay.
  static uint32_t g_device_set_time_ms_[kNumberDevices];

  // Singleton instance.
  static TRelay* g_singleton_;
};