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
    INTEL_POWER,
    INTEL_RESET,
    MOTOR_POWER,
    NVIDIA_POWER,
    NVIDIA_RESET,
    MOTOR_ESTOP,
    NUMBER_DEVICES  // Number of relays on the board
  } TRelayDevice;

  // Is in the power-on state?
  bool isPoweredOn(TRelayDevice device);

  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "TRelay"; }

  // Power the relay on.
  void powerOff(TRelayDevice device);

  // Power the relay off.
  void powerOn(TRelayDevice device);

  // From TModule.
  void setup();

  // Singleton constructor.
  static TRelay& singleton();

 private:
  // GPIO pins to control the relays.
  typedef enum {
    INTEL_ON_OFF_PIN = 0,
    INTEL_RESET_PIN = 1,
    MOTOR_ON_OFF_PIN = 5, //###
    NVIDIA_ON_OFF_PIN = 3,
    NVIDIA_RESET_PIN = 4,
    MOTOR_ESTOP_PIN = 2 //#####
  } TPinNumbers;

  // How long (in milliseconds) to trip the relay to simulate a reset push.
  static const uint32_t RESET_DURATION_MS = 1000;

  // Private constructor.
  TRelay();

  // A non-zero value indicates the relay is powered-on.
  // Relays which are momentary switches will use the value here to
  // determine when to reset the relay.
  static uint32_t g_deviceSetTimeMs[NUMBER_DEVICES];

  // Singleton instance.
  static TRelay* g_singleton;
};