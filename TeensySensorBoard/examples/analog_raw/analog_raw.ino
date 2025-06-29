// Pin numbers corresponding to the Analog Block connectors.
enum TeensySensorAnalogPins {
  ALG0 = 24,
  ALG1 = 25,
  ALG2 = 26,
  ALG3 = 27,
  AGL4 = 38,
  ALG5 = 39,
  ALG6 = 40,
  ALG7 = 41
};

void setup() { Serial.begin(38400); }

void loop() {
  int val = analogRead(ALG7);
  Serial.print("ALG7 value is: ");
  Serial.println(val);
  delay(250);
}
