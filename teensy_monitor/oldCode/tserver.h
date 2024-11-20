#pragma once

#include <NativeEthernet.h>
#include <stdint.h>

#include <string>

#include "tmodule.h"

class TServer : TModule {
 public:
  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "TServer"; }

  // From TModule.
  void setup();

  // Singleton constructor.
  static TServer& singleton();

 private:
  // States for the state machine.
  typedef enum TState { NO_DEVICE, NO_LINK, AWAIT_CLIENT, READ_REQUEST } TState;

  // Private constructor.
  TServer();

  // Get the sensor string literal for the response message.
  static std::string sensorString();

  // The singleton instance of the client request handler device.
  static EthernetClient g_client;

  // The singleton instance of the server device.
  static EthernetServer g_server;

  // Current state of the state machine.
  static TState g_state;

  // Singleton instance.
  static TServer* g_singleton;

  // The device MAC address.
  static const uint8_t MAC_ADDRESS[6];

  // The boiler plate template for a response to the request.
  static const char* g_header;
};