#pragma once

#include <NativeEthernet.h>

#include "tmodule.h"

class TRosClient : TModule {
 public:
  // From TModule.
  void loop();

  // From TModule.
  const char *name() { return "TRosClient"; }

  // From TModule.
  void setup();

  // Singleton constructor.
  static TRosClient &singleton();

 private:
  // States for loop execution.
  typedef enum {
    NONE,
    AWAIT_CLIENT,
    AWAIT_RESPONSE,
    GATHER_RESPONSE,
  } TState;

  // Private constructor.
  TRosClient();

  // Request data from ROS.
  void makeHttpRequest();

  // The singleton instance of the HTTP client.
  static EthernetClient g_client;

  // Singleton instance.
  static TRosClient *g_singleton;

  // Current state in state machine.
  static TState g_state;

  // THe MAC address of the device.ß
  static const uint8_t MAC_ADDRESS[6];
};