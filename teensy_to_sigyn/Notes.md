
# Analog inputs
analogRead(9) will read from J5 pin 2
analogRead(8) will read from J5 pin 3

```code
^ teensy side
+----------+
| GND   A9 |
| A8   3.3 |
|    |-----+
+----+
v towards TOF connectors
```

```code
// Basic analog read example for Teensy 4.2
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Wait for serial port to connect (optional, for debugging)
  while (!Serial && millis() < 5000) {
    // Wait up to 5 seconds for serial connection
  }
  
  // Configure analog resolution (optional)
  // Teensy 4.2 supports 8, 10, 12, or 16-bit resolution
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  
  Serial.println("Teensy 4.2 Analog Pin 12 Reader");
  Serial.println("Reading analog values...");
}

void loop() {
  // Read analog value from pin 12
  int analogValue = analogRead(9);
  
  // Convert to voltage (assuming 3.3V reference)
  float voltage = (analogValue * 3.3) / 4095.0;  // For 12-bit resolution
  
  // Print the values
  Serial.print("Pin 12 - Raw: ");
  Serial.print(analogValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage, 3);  // 3 decimal places
  Serial.println("V");
  
  // Wait 500ms before next reading
  delay(500);
}
```

# E-stop

Pin numbers for relays and connector details:

```code
^ towards board edge
+----------------------|
| GND 7 6 5 4 3 2 1 5v |
+----------------------|
```

```code
const uint8_t kMotorEStopPin = 7; // Relay7

void setup() {
  pinMode(kMotorEStopPin, OUTPUT);
  digitalWrite(kMotorEStopPin, LOW);
}

void loop() {
  digitalWrite(kMotorEStopPin, HIGH);
  delay(100);
  digitalWrite(kMotorEStopPin, LOW);
  delay(100);
}
```

# Teensy chip temperature
Note that the chip will typicall run 40 degrees C or hotter.
Over 85 would probably be unsafe.

```code
extern float tempmonGetTemp(void);

void setup() {
  while (!Serial);
}

void loop() {
  Serial.print( tempmonGetTemp() );
  Serial.println("°C");
  delay(500);
}
```