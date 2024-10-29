#include "tlabeled_on_off_button.h"

TLabeledOnOffButton::TLabeledOnOffButton(
    String label, uint8_t rotation, uint16_t labelX, uint16_t labelY,
    uint16_t labelHeight, uint16_t labelWidth, uint16_t buttonWidth,
    uint16_t colorButton, uint16_t colorText, String textOn, String textOff,
    TOnOffButton::TButtonState initialState,
    TOnOffButton::callbackFunction function, void* functionParameter, uint8_t id)
    : _id(id),
      _label(label),
      _labelHeight(labelHeight),
      _labelWidth(labelWidth),
      _x(labelX),
      _y(labelY) {
    TOnOffButton::makeButton(rotation, labelX + labelWidth + 2, labelY,
                             buttonWidth, labelHeight, colorButton, colorText,
                             textOn, textOff, initialState, function,
                             functionParameter, id);
    TControlDisplay& tc = TControlDisplay::singleton();
    int16_t yCenter = 16;
    tc.setCursor(labelX, labelY + yCenter);
    tc.setTextColor(colorText);
    tc.print(label);
}

void TLabeledOnOffButton::makeButton(
    String label, uint8_t rotation, uint16_t labelX, uint16_t labelY,
    uint16_t labelHeight, uint16_t labelWidth, uint16_t buttonWidth,
    uint16_t colorButton, uint16_t colorText, String textOn, String textOff,
    TOnOffButton::TButtonState initialState,
    TOnOffButton::callbackFunction function, void* functionParameter, uint8_t id) {
    TLabeledOnOffButton(label, rotation, labelX, labelY, labelHeight,
                        labelWidth, buttonWidth, colorButton, colorText, textOn,
                        textOff, initialState, function, functionParameter, id);
}
