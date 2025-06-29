#include <Servo.h>

enum TeensySensorPWMPins {
  PWM0 = 4,
  PWM1 = 5,
  PWM2 = 6,
  PWM3 = 7,
};

Servo myservo;

void setup() {
  myservo.attach(PWM0);  // attaches the servo on pin 20
}

void loop() {
  for (int pos = 10; pos < 170;
       pos += 1)         // goes from 10 degrees to 170 degrees
  {                      // in steps of 1 degree
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);           // waits 15ms for the servo to reach the position
  }
  for (int pos = 180; pos >= 1; pos -= 1)  // goes from 180 degrees to 0 degrees
  {
    myservo.write(pos);  // tell servo to go to position in variable 'pos'
    delay(15);           // waits 15ms for the servo to reach the position
  }
}