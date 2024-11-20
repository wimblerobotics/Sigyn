#include <stdint.h>
#include <Adafruit_INA260.h>

#include "tmotor_current.h"

TMotorCurrent::TMotorCurrent() {
  for (int8_t i = 0; i < AVERAGE_COUNT; i++) {
    g_leftMotorCurrentMaReadings[i] = 0.0;
    g_rightMotorCurrentMaReadings[i] = 0;
  }
}


int TMotorCurrent::getValueMa(MOTOR index) {
  if (index == LEFT) return g_leftMotorCurrentMa;
  else return g_rightMotorCurrentMa;
}


void TMotorCurrent::loop() {
  if (g_haveLeftMotorSensor) {
    g_leftMotorCurrentMaReadings[g_nextMotorCurrentIndex] = g_leftINA260.readCurrent();
  } else {
    g_leftMotorCurrentMaReadings[g_nextMotorCurrentIndex] = 0;
  }

  if (g_haveRightMotorSensor) {
    g_rightMotorCurrentMaReadings[g_nextMotorCurrentIndex] = g_rightINA260.readCurrent();
  } else {
    g_rightMotorCurrentMaReadings[g_nextMotorCurrentIndex] = 0;
  }

  g_nextMotorCurrentIndex++;
  if (g_nextMotorCurrentIndex >= AVERAGE_COUNT) {
    g_nextMotorCurrentIndex = 0;
  }

  float leftAveragaMa = 0.0;
  float rightAveragaMa = 0.0;
  for (uint8_t i = 0; i < AVERAGE_COUNT; i++) {
    leftAveragaMa += g_leftMotorCurrentMaReadings[i];
    rightAveragaMa += g_rightMotorCurrentMaReadings[i];
  }

  g_leftMotorCurrentMa = leftAveragaMa / AVERAGE_COUNT;
  g_rightMotorCurrentMa = rightAveragaMa / AVERAGE_COUNT;
}


void TMotorCurrent::setup() {
  g_haveLeftMotorSensor = g_leftINA260.begin(LEFT_MOTOR__ADDRESS);
  g_haveRightMotorSensor = g_rightINA260.begin(RIGHT_MOTOR_ADDRESS);
  if (!g_haveLeftMotorSensor) {
    Serial.println("No left motor current sensor");
  }

  if (!g_haveRightMotorSensor) {
    Serial.println("No right motor current sensor");
  }
}


TMotorCurrent& TMotorCurrent::singleton() {
  if (!g_singleton) {
    g_singleton = new TMotorCurrent();
  }

  return *g_singleton;
}


TMotorCurrent* TMotorCurrent::g_singleton = nullptr;

uint8_t TMotorCurrent::g_nextMotorCurrentIndex = 0;

bool TMotorCurrent::g_haveLeftMotorSensor;
bool TMotorCurrent::g_haveRightMotorSensor;

Adafruit_INA260 TMotorCurrent::g_leftINA260;
Adafruit_INA260 TMotorCurrent::g_rightINA260;

float TMotorCurrent::g_leftMotorCurrentMa = 0;

float TMotorCurrent::g_rightMotorCurrentMa = 0;

float TMotorCurrent::g_leftMotorCurrentMaReadings[AVERAGE_COUNT];

float TMotorCurrent::g_rightMotorCurrentMaReadings[AVERAGE_COUNT];