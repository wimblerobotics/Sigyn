#include <Wire.h>
#include <cstdint>
#include <Arduino.h>

// Hardware configuration constants
constexpr uint8_t I2C_MULTIPLEXER_ADDRESS = 0x70;
constexpr uint8_t I2C_MULTIPLEXER_ENABLE_PIN = 8;
constexpr uint32_t I2C_CLOCK_FREQUENCY = 400000;

void setup() {
  // Do other setup here...
  Serial.begin(115200);
  while (!Serial)
  {
    ;  // Wait for serial port to connect. Needed for native USB port only
  }

  // Initialize I2C communication
  pinMode(I2C_MULTIPLEXER_ENABLE_PIN, OUTPUT);
  digitalWrite(I2C_MULTIPLEXER_ENABLE_PIN, HIGH);
  Wire.begin();
  Wire.setClock(I2C_CLOCK_FREQUENCY);

  // Test I2C multiplexer
  multiplexer_available_ = testMultiplexer();
  if (!multiplexer_available_)
  {
    Serial.println("I2C multiplexer not found");
    return;
  }
}

void loop()
{
  // Example usage of the multiplexer
  uint8_t sensor_index = 0;  // A number in [0..7] selecting the sensor channel (connector).
  selectSensorChannel(sensor_index);
  // Read data from the selected sensor channel
  // Insert your I2C read/write code here
}

void selectSensorChannel(uint8_t sensor_index)
{
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);  // I2C_MULTIPLEXER_ADDRESS, adjust as needed
  Wire.write(1 << sensor_index);                    // Select channel
  Wire.endTransmission();
  delayMicroseconds(100);
}

bool testMultiplexer()
{
  Wire.beginTransmission(I2C_MULTIPLEXER_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0)
  {
    String msg = "I2C multiplexer found at address 0x" + String(I2C_MULTIPLEXER_ADDRESS, HEX);
    Serial.println(msg.c_str());
    return true;
  }
  else
  {
    String msg =
        "I2C multiplexer not found at address 0x" + String(I2C_MULTIPLEXER_ADDRESS, HEX) + ", error: " + String(error);
    Serial.println(msg.c_str());
    return false;
  }
}
