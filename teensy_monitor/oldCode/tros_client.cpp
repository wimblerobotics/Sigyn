#include "tros_client.h"

#include <NativeEthernet.h>

void TRosClient::loop() {
  static int loopAwaitResultCount = 0;
  static int maxLoopAwaitResultCount = -1000;
  static String response = "";
  // Serial.print("[TRosClient::loop] g_state: ");
  // Serial.println(g_state);
  switch (g_state) {
    case NONE:
      break;

    case AWAIT_CLIENT:
      makeHttpRequest();
      break;

    case AWAIT_RESPONSE:
      if (g_client.connected() && g_client.available()) {
        g_state = GATHER_RESPONSE;
      }

      break;

    case GATHER_RESPONSE: {
      loopAwaitResultCount++;
      if (g_client.connected() && g_client.available()) {
        while (g_client.connected() && g_client.available()) {
          char c = g_client.read();
          response += c;
        }

        if (response.indexOf("</html>") >= 0) {
          // Serial.println("Response string: >>");
          // Serial.println(response);
          // Serial.println("<<");
          if (maxLoopAwaitResultCount < loopAwaitResultCount) {
            maxLoopAwaitResultCount = loopAwaitResultCount;
            // Serial.print("maxLoopAwaitResultCount: ");
            // Serial.println(maxLoopAwaitResultCount);
          }

          response = "";
          g_client.stop();
          loopAwaitResultCount = 0;
          g_state = AWAIT_CLIENT;
        }
      }

      break;

      default:
        break;
    }
  }
}

void TRosClient::makeHttpRequest() {
  IPAddress server(67, 20, 113, 11);
  // char server[] = "wimble.org";
  int connectSuccess;
  if ((connectSuccess = g_client.connect(server, 80)) == 1) {
    g_client.println("GET /index.html HTTP/1.1");
    g_client.println("Host: www.wimble.org");
    g_client.println("User-Agent: arduino-ethernet");
    g_client.println("Connection: close");
    g_client.println();
    g_state = AWAIT_RESPONSE;
  } else {
    // Serial.print("connection failed: ");
    // Serial.println(connectSuccess);
  }
}

void TRosClient::setup() {
  Ethernet.begin((uint8_t *)&MAC_ADDRESS, IPAddress(10, 0, 0, 132));
  // Serial.print("My IP address: ");
  // Serial.println(Ethernet.localIP());
  g_state = AWAIT_CLIENT;
  delay(100);
}

TRosClient::TRosClient() : TModule() {}

TRosClient &TRosClient::singleton() {
  if (!g_singleton) {
    g_singleton = new TRosClient();
  }

  return *g_singleton;
}

EthernetClient TRosClient::g_client;

TRosClient *TRosClient::g_singleton = nullptr;

TRosClient::TState TRosClient::g_state = TRosClient::NONE;

const uint8_t TRosClient::MAC_ADDRESS[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
