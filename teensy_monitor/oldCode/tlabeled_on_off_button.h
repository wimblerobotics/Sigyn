#pragma once

#include <stdint.h>
#include "tcontrol_display.h"
#include "ton_off_button.h"


class TLabeledOnOffButton {
public:

  static void makeButton(String label,
                      uint8_t rotation,
                      uint16_t labelX,
                      uint16_t labelY,
                      uint16_t labelHeight,
                      uint16_t labelWidth,
                      uint16_t buttonWidth,
                      uint16_t colorButton,
                      uint16_t colorText,
                      String textOn,
                      String textOff,
                      TOnOffButton::TButtonState initialState,
                      TOnOffButton::callbackFunction function = nullptr,
                      void* functionParameter = nullptr,
                      uint8_t id = 255);

private:

  TLabeledOnOffButton(String label,
                      uint8_t rotation,
                      uint16_t labelX,
                      uint16_t labelY,
                      uint16_t labelHeight,
                      uint16_t labelWidth,
                      uint16_t buttonWidth,
                      uint16_t colorButton,
                      uint16_t colorText,
                      String textOn,
                      String textOff,
                      TOnOffButton::TButtonState initialState,
                      TOnOffButton::callbackFunction function,
                      void* functionParameter,
                      uint8_t id);

  TOnOffButton _button;
  uint8_t _id;
  String  _label;
  uint16_t _labelHeight;
  uint16_t _labelWidth;
  uint16_t _x;
  uint16_t _y;

};
