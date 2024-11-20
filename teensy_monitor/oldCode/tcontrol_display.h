#pragma once

#include <stdint.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>


// Encapsulate the Color 320x240 TFT Touchscreen, ILI9341 Controller Chip
// for the Teensy chip. This is similar to the AdaFruit ILI9341 color
// touchscreen except for a different touchscreen controller (aparently).
//
// This class forwards all the interesting methods of the underlying hardware
// libraries, so just add any further hardware methods to this class as you
// need this. 
//
// This class unifies the color screen and the touch screen into a single class 
// and provides color-screen pixel coordinate conversion from the touch screen.
// That is, e.g., touchedXCharPosition will return the touched position in
// the color pixel range of [0 .. 319] rather than the touch screen coordinate
// range of [0 .. 3899].
//
// This is a "singleton" class -- that is, there will be only one instance of this
// class created, and classes that want to use this functionality just need to 
// call the "void singleton()" method which will lazily create the instance as needed.
//
// Cookbook:
//    #include <Wire.h>
//    #include "tcontrol_display.h"
//    TControlDisplay& tc = TControlDisplay::singleton(); 
//    tc.fillScreen(ILI9341_BLUE);
//    tc.setRotation(1);
//    tc.setTextColor(ILI9341_WHITE);
//    tc.setTextSize(3);
//    String label = "Some Text";
//    uint16_t labelX = 30;
//    uint16_t labelY = 50;
//    uint16_t yCenter = tc.measureTextHeight(label.c_str(), 0);
//    tc.setCursor(labelX, labelY + yCenter);
//    tc.print(label);
//    if (tc.touched()) {
//      Serial.print("Touched at x: "); Serial.println(tc.touchedXCharPosition());
//    }
//
class TControlDisplay {
public:
  static TControlDisplay& singleton();

  void clearAll();

  // Color display methods forwarded from the underlying hardware library.
  void drawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  void drawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
  void fillScreen(uint16_t color);
  uint16_t measureTextHeight(const char* text, int num);
  uint16_t measureTextWidth(const char* text, int num);
  void print(String s);
  void println(String s);
  void setCursor(int16_t x, int16_t y);
  void setRotation(uint8_t r);
  void setTextColor(uint16_t c);
  void setTextSize(uint8_t s);

  // Touchscreen methods forwarded from the underlying hardware library.
  TS_Point getPoint();
  bool touched();

  // Touched X-position in terms of character coordinates in [0 .. 319].
  int16_t touchedXCharPosition() const;

  // Touched Y-position in terms of character coordinates in [0 .. 239].
  int16_t touchedYCharPosition() const;
  
private:

  TControlDisplay();
   
   // Change these to match your hardware wiring.
  static const uint8_t TFT_CS = 10; //j8;
  static const uint8_t T_CS = 29;//#####28; //RX7
  static const uint8_t TFT_DC = 9; // OUT1C
  static const uint8_t TIRQ_PIN = 28;//#####29; // TX7

  // Constants to match the hardware touchscreen.
  static const uint16_t MAX_X_TOUCH_VALUE = 3900;
  static const uint16_t MIN_X_TOUCH_VALUE = 360;
  static const uint16_t MAX_Y_TOUCH_VALUE = 3900;
  static const uint16_t MIN_Y_TOUCH_VALUE = 300;

  // The underlying color display.
  static Adafruit_ILI9341 _display;

  // The singleton instance of this class.  
  static TControlDisplay* _singleton;

  // The underlying touchscreen.
  static XPT2046_Touchscreen* _touchScreen;
};
