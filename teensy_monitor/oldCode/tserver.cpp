#include "tserver.h"

#include <NativeEthernet.h>
#include <stdio.h>

#include "tmotor_current.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

void TServer::loop() {
  static char str[2048]; // Big enough to hold the HTTP request.
  static uint16_t strIndex = 0; // Index to stuff next character into 'str'.

  switch (g_state) {
    case AWAIT_CLIENT:
      g_client = g_server.available();
      if (g_client) {
        g_state = READ_REQUEST;
      }

      break;

    case READ_REQUEST:
      if (!g_client.connected() || !g_client.available()) {
        // Connection dropped.
        g_client.stop();
        g_state = AWAIT_CLIENT;
      } else {
        char c = g_client.read();
        if (strIndex < (sizeof(str) - 1)) {
          str[strIndex++] = c;
        }

        str[strIndex] = 0;
        if (c == '\n') {
          // End of the request has been received.
          std::string result = sensorString();
          char content[512]; // To hold the response.
          sprintf(content, g_header, strlen(result.c_str()), result.c_str());
          g_client.println(content);
          delay(1);
          g_client.stop();
          g_state = AWAIT_CLIENT;
        }
      }

      break;

    default:
      break;
  }
}

std::string TServer::sensorString() {
  static uint32_t sequenceNumber = 0;
  int motorCurrentValues[TMotorCurrent::NUMBER_MOTORS];
  int sonarValues[TSonar::NUMBER_SONARS];
  int temperatureValues[TTemperature::NUMBER_TEMPERATURES];
  int timeOfFlightValues[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT];

  for (uint8_t i = 0; i < TMotorCurrent::NUMBER_MOTORS; i++) {
    motorCurrentValues[i] = TMotorCurrent::singleton().getValueMa(
        static_cast<TMotorCurrent::MOTOR>(i));
  }

  for (uint8_t i = 0; i < TSonar::NUMBER_SONARS; i++) {
    sonarValues[i] = TSonar::singleton().getValueMm(static_cast<TSonar::SONAR>(i));
  }

  for (uint8_t i = 0; i < TTemperature::NUMBER_TEMPERATURES; i++) {
    temperatureValues[i] = TTemperature::singleton().getValueTenthsC(static_cast<TTemperature::TEMPERATURE>(i));
  }

  for (uint8_t i = 0; i < TTimeOfFlight::NUMBER_TIME_OF_FLIGHT; i++) {
    timeOfFlightValues[i] = TTimeOfFlight::singleton().getValueMm(static_cast<TTimeOfFlight::TIMEOFFLIGHT>(i));
  }

  char result[512];
  sprintf(result,
          "{\n"
          " \"sequence_number\": %d,\n"
          "  \"motor_currents_ma\": [ %d, %d],\n"
          "  \"sonar_mm\": [%d, %d, %d, %d],\n"
          "  \"temperature_tenthsC\": [%d, %d],\n"
          "  \"time_of_flight_mm\": [%d, %d, %d, %d, %d, %d, %d, %d]\n"
          "}\n",
          sequenceNumber++,
          motorCurrentValues[0], motorCurrentValues[1], sonarValues[0],
          sonarValues[1], sonarValues[2], sonarValues[3], temperatureValues[0],
          temperatureValues[1], timeOfFlightValues[0], timeOfFlightValues[1],
          timeOfFlightValues[2], timeOfFlightValues[3], timeOfFlightValues[4],
          timeOfFlightValues[5], timeOfFlightValues[6], timeOfFlightValues[7]);

  // Serial.print("[TServer::handleRequest] result length:
  // ");Serial.println(strlen(result));
  std::string str(result);
  return str;
}

void TServer::setup() {
  Ethernet.begin((uint8_t *)&MAC_ADDRESS, IPAddress(10, 42, 0, 132));
  g_server.begin();
   Serial.print("My IP address: ");
   Serial.println(Ethernet.localIP());

  g_state = AWAIT_CLIENT;
}

TServer::TServer() : TModule() {}

TServer &TServer::singleton() {
  if (!g_singleton) {
    g_singleton = new TServer();
  }

  return *g_singleton;

}

EthernetClient TServer::g_client;
EthernetServer TServer::g_server(80);

TServer::TState TServer::g_state = NO_DEVICE;

TServer *TServer::g_singleton = nullptr;

const uint8_t TServer::MAC_ADDRESS[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

const char *TServer::g_header =
    "HTTP/1.1 200 OK\n"
    "Content-Type: application/json\n"
    "Content-Length: %d\n"
    "Connection: close\n"
    // "Refresh: 5\n"
    "\r\n"
    "%s";
