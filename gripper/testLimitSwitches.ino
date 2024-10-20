#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>

  #include "Wire.h"
  enum {
    kPinEcho3 = 35, // Extender in limit.
    kPinTrigger3 = 34,
    kPinEcho2 = 37,
    kPinTrigger2 = 36,
    kPinEcho1 = 41,
    kPinTrigger1 = 40,
    kPinEcho0 = 15,
    kPinTrigger0 = 14
  };

void setup() {
  pinMode(kPinEcho0, INPUT);
  pinMode(kPinTrigger0, INPUT);

  pinMode(kPinEcho1, INPUT);
  pinMode(kPinTrigger1, INPUT);

  pinMode(kPinEcho2, INPUT);
  pinMode(kPinTrigger2, INPUT);

  pinMode(kPinEcho3, INPUT);
  pinMode(kPinTrigger3, INPUT);

  Serial.begin(9600);
}

void loop() {
  uint8_t t;
  uint8_t e;
  static int counter = 0;

  Serial.print(counter++);
  t = digitalRead(kPinTrigger0);
  e = digitalRead(kPinEcho0);
  Serial.print("     0t: ");
  Serial.print(t);
  Serial.print(", 0e: ");
  Serial.print(e);

  t = digitalRead(kPinTrigger1);
  e = digitalRead(kPinEcho1);
  Serial.print(",     1t: ");
  Serial.print(t);
  Serial.print(", 1e: ");
  Serial.print(e);
  
  t = digitalRead(kPinTrigger2);
  e = digitalRead(kPinEcho2);
  Serial.print(",     2t: ");
  Serial.print(t);
  Serial.print(", 2e: ");
  Serial.print(e);
  
  t = digitalRead(kPinTrigger3);
  e = digitalRead(kPinEcho3);
  Serial.print(",     3t: ");
  Serial.print(t);
  Serial.print(", 3e: ");
  Serial.println(e);
  
  delay(500);
}
