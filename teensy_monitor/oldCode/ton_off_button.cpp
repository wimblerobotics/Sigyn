#include "ton_off_button.h"

TOnOffButton::TOnOffButton(uint8_t rotation, uint16_t x, uint16_t y,
                           uint16_t width, uint16_t height,
                           uint16_t colorButton, uint16_t colorText,
                           String textOn, String textOff,
                           TButtonState initialState, callbackFunction function,
                           void* functionParameter, uint8_t id)
    : _colorButton(colorButton),
      _colorText(colorText),
      _function(function),
      _functionParameter(functionParameter),
      _height(height),
      _id(id),
      _rotation(rotation),
      _state(initialState),
      _textOff(textOff),
      _textOn(textOn),
      _width(width),
      _x(x),
      _y(y) {
  TControlDisplay& tc = TControlDisplay::singleton();
  drawButton(tc, *this, initialState == OFF);
}


TOnOffButton::TOnOffButton() {
  _colorButton = 0;
  _colorText = 0;
  _function = nullptr;
  _functionParameter = nullptr;
  _id = -1;
  _height = 0;
  _rotation = 0;
  _state = ON;
  _textOff = "";
  _textOn = "";
  _width = 0;
  _x = 0;
  _y = 0;
}


TOnOffButton& TOnOffButton::operator=(const TOnOffButton& b) {
  _colorButton = b._colorButton;
  _colorText = b._colorText;
  _function = b._function;
  _functionParameter = b._functionParameter;
  _height = b._height;
  _id = b._id;
  _rotation = b._rotation;
  _state = b._state;
  _textOff = b._textOff;
  _textOn = b._textOn;
  _width = b._width;
  _x = b._x;
  _y = b._y;
  return *this;
}


void TOnOffButton::setState(uint8_t id, TButtonState newState) {
  for (uint8_t i = 0; i < _nextIndex; i++) {
    TOnOffButton& button = _allButtonList[i];
    if (button._id == id) {
      button._state = newState;
      //#####Serial.print("[TOnOffButton::setState] MATCH for id: "); Serial.print(id);//#####
      //#####Serial.print(", newState: ");Serial.println(newState);
      drawButton(TControlDisplay::singleton(), button, button._state == OFF);
      return;
    } else {
      //#####Serial.print("[TOnOffButton::setState] no match for id: ");
      //#####Serial.print(id);
      //#####Serial.print(", and button _id: ");
      //#####Serial.println(button._id);
    }
  }

  //#####Serial.print("[TOnOffButton::setState] no match for id: "); Serial.println(id); //#####
}


void TOnOffButton::drawButton(TControlDisplay& tc, TOnOffButton& b,
                              bool invert) {
  uint16_t colorButton = invert ? b._colorText : b._colorButton;
  uint16_t colorText = invert ? b._colorButton : b._colorText;

  tc.setRotation(b._rotation);
  tc.fillRect(b._x, b._y, b._width, b._height, colorButton);
  tc.setTextColor(colorText);
  tc.setTextSize(2);

  int16_t tXCtr;
  int16_t tYCtr;

  if (b._state == ON) {
    tXCtr = 12 * strlen(b._textOn.c_str());
    tYCtr = 16;
    // tXCtr = tc.measureTextWidth(b._textOn.c_str(), 0);
    // tYCtr = tc.measureTextHeight(b._textOn.c_str(), 0);
  } else {
    tXCtr = 12 * strlen(b._textOff.c_str());
    tYCtr = 16;
    // tXCtr = tc.measureTextWidth(b._textOff.c_str(), 0);
    // tYCtr = tc.measureTextHeight(b._textOff.c_str(), 0);
  }

  tc.setCursor(b._x + (b._width / 2) - (tXCtr / 2),
               b._y + (b._height / 2) - (tYCtr / 2));
  if (b._state == ON) {
    tc.println(b._textOn);
  } else {
    tc.println(b._textOff);
  }
}


void TOnOffButton::makeButton(uint8_t rotation, uint16_t x, uint16_t y,
                              uint16_t width, uint16_t height,
                              uint16_t colorButton, uint16_t colorText,
                              String textOn, String textOff,
                              TButtonState initialState,
                              callbackFunction function,
                              void* functionParameter,
                              uint8_t id) {
  TOnOffButton b(rotation, x, y, width, height, colorButton, colorText, textOn,
                 textOff, initialState, function, functionParameter, id);
  _allButtonList[_nextIndex++] = b;
}


void TOnOffButton::clearAll() { _nextIndex = 0; }


void TOnOffButton::setState(TOnOffButton::TButtonState state) {
  _state = state;
  drawButton(TControlDisplay::singleton(), *this, _state != ON);
}


TOnOffButton::TButtonState TOnOffButton::state() { return _state; }


int8_t TOnOffButton::touchedButtonIndex(TControlDisplay& tc) {
  tc.setRotation(0);
  int16_t x = 320 * (((float)tc.touchedXCharPosition()) / 240.0);
  int16_t y = 240 * (((float)tc.touchedYCharPosition()) / 320.0);

  for (int i = 0; i < _nextIndex; i++) {
    TOnOffButton& b = _allButtonList[i];
    tc.setRotation(b._rotation);
    int16_t bx = b._x;
    int16_t by = b._y;
    int16_t h = b._height;
    int16_t w = b._width;
    switch (b._rotation) {
      case 0:
        bx = b._y;  // 320 - (b._y + b._height);
        by = 240 - (b._x + b._width);
        h = b._width;
        w = b._height;
        break;

      default:
        break;
    }

    // if (_touchState != AwaitTouch) {
    //   Serial.print("[touchedButtonIndex] "); Serial.print(b._textOn);
    //   Serial.print(", i: "); Serial.print(i);
    //   Serial.print(", x: "); Serial.print(x);
    //   Serial.print(", y: "); Serial.print(y);
    //   Serial.print(", bx: "); Serial.print(bx);
    //   Serial.print(", by: "); Serial.print(by);
    //   Serial.print(", bx+w: "); Serial.print(bx + w);
    //   Serial.print(", by+h: "); Serial.println(by + h);
    //   Serial.print("    box_x: "); Serial.print(b._x);
    //   Serial.print(", box_width: "); Serial.print(b._width);
    //   Serial.print(", box_y: "); Serial.print(b._y);
    //   Serial.print(", box_height: "); Serial.println(b._height);
    // }
    if ((x >= bx) && (x < (bx + w)) && (y >= by) && (y < (by + h))) {
      // Serial.print("Match "); Serial.print(b._textOn); Serial.print(",
      // i: "); Serial.println(i);
      return i;
    }
  }

  return -1;
}


void TOnOffButton::update() {
  TControlDisplay& tc = TControlDisplay::singleton();

  if (tc.touched()) {
    switch (_touchState) {
      case AwaitTouch:
        // Capture the object that was touched and the time of the
        // touch.
        _touchIndex = touchedButtonIndex(tc);
        if (_touchIndex != -1) {
          _touchTimerMs = millis();
          _touchState = DelayTiming;
        }

        break;

      case DelayTiming: {
        uint32_t now = millis();
        if (now > (_touchTimerMs + REQUIRED_TOUCH_MS)) {
          // Enough time has elapsed, see if there is still a touch
          // for the same object.
          int8_t currentTouchIndex = touchedButtonIndex(tc);
          if ((currentTouchIndex == -1) || (currentTouchIndex != _touchIndex)) {
            // Not touching same object, abort.
            _touchState = AwaitTouch;
          } else {
            // Still touching same object. Start feedback and away
            // end of touch.
            drawButton(tc, _allButtonList[_touchIndex], true);
            _touchState = GraphicFeedback;
          }
        }
      }

      break;

      case GraphicFeedback: {
        int8_t currentTouchIndex = touchedButtonIndex(tc);
        if ((currentTouchIndex == -1) || (currentTouchIndex != _touchIndex)) {
          // Not touching same object, abort.
          drawButton(tc, _allButtonList[_touchIndex], false);
          _touchState = AwaitTouch;
        }
      } break;

    }  // switch
  } else {
    if (_touchState == GraphicFeedback) {
      // Completed the whole statemachine. Do the button action.
      drawButton(tc, _allButtonList[_touchIndex], false);
      if (_allButtonList[_touchIndex]._function) {
        _allButtonList[_touchIndex]._function(
            _allButtonList[_touchIndex],
            _allButtonList[_touchIndex]._functionParameter);
      }
    }

    _touchState = AwaitTouch;
  }
}


TOnOffButton TOnOffButton::_allButtonList[/*TOnOffButton::MAX_BUTTONS*/];

uint8_t TOnOffButton::_nextIndex = 0;

int8_t TOnOffButton::_touchIndex;

TOnOffButton::TPressState TOnOffButton::_touchState = AwaitTouch;

uint32_t TOnOffButton::_touchTimerMs = 0;
