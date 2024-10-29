#pragma once

#include <stdint.h>

#include "tcontrol_display.h"

class TOnOffButton {
 public:
  typedef enum { ON, OFF } TButtonState;

  typedef void (*callbackFunction)(TOnOffButton& button, void* parameter);

  static void clearAll();

  static void makeButton(uint8_t rotation, uint16_t x, uint16_t y,
                         uint16_t width, uint16_t height, uint16_t colorButton,
                         uint16_t colorText, String textOn, String textOff,
                         TButtonState initialState,
                         callbackFunction function = nullptr,
                         void* functionParameter = nullptr,
                         uint8_t id = 255);

  static void setState(uint8_t id, TButtonState newState);

  void setState(TButtonState state);

  TButtonState state();

  static void update();

 private:
  friend class TLabeledOnOffButton;

  typedef enum {
    AwaitTouch,       // Waiting for a touch.
    DelayTiming,      // Start a clock and test for touch after that.
    GraphicFeedback,  // Show graphic feedback until touch released.
  } TPressState;

  TOnOffButton(uint8_t rotation, uint16_t x, uint16_t y, uint16_t width,
               uint16_t height, uint16_t colorButton, uint16_t colorText,
               String textOn, String textOff, TButtonState initialState,
               callbackFunction function, void* functionParameter, uint8_t id);

  TOnOffButton();

  TOnOffButton& operator=(const TOnOffButton&);

  static void drawButton(TControlDisplay& tc, TOnOffButton& b, bool invert);

  static int8_t touchedButtonIndex(TControlDisplay& tc);

  uint16_t _colorButton;
  uint16_t _colorText;
  callbackFunction _function;
  void* _functionParameter;
  uint16_t _height;
  uint8_t _id;
  uint8_t _rotation;
  TButtonState _state;
  String _textOff;
  String _textOn;
  uint16_t _width;
  uint16_t _x;
  uint16_t _y;

  static const uint8_t MAX_BUTTONS = 24;
  static const uint32_t REQUIRED_TOUCH_MS = 500;

  static TOnOffButton _allButtonList[MAX_BUTTONS];
  static uint8_t _nextIndex;
  static int8_t _touchIndex;
  static TPressState _touchState;
  static uint32_t _touchTimerMs;
};