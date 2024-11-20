#include "tcontrol_display.h"

#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include <SPI.h>

TControlDisplay& TControlDisplay::singleton() {
    if (!_singleton) {
        _singleton = new TControlDisplay();
    }

    return *_singleton;
}

TControlDisplay::TControlDisplay() {
    _display.begin();
    pinMode(T_CS, OUTPUT);
    _touchScreen = new XPT2046_Touchscreen(T_CS, TIRQ_PIN);
    if (!_touchScreen->begin()) {
        Serial.println("### FAILED TO INIT TOUCHSCREEN");
    } else {
    }
}

void TControlDisplay::clearAll() { _display.fillScreen(ILI9341_BLACK); }

void TControlDisplay::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
    _display.drawRect(x, y, w, h, color);
}

void TControlDisplay::drawHLine(int16_t x, int16_t y, int16_t w,
                                uint16_t color) {
    _display.drawFastHLine(x, y, w, color);
}

void TControlDisplay::drawVLine(int16_t x, int16_t y, int16_t h,
                                uint16_t color) {
    _display.drawFastVLine(x, y, h, color);
}

void TControlDisplay::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                               uint16_t color) {
    _display.fillRect(x, y, w, h, color);
}

void TControlDisplay::fillScreen(uint16_t color) { _display.fillScreen(color); }

TS_Point TControlDisplay::getPoint() { return _touchScreen->getPoint(); }

uint16_t TControlDisplay::measureTextHeight(const char* text, int num) {
    return strlen(text) * 16;
    // return _display.measureTextHeight(text, num);
}

uint16_t TControlDisplay::measureTextWidth(const char* text, int num) {
    return strlen(text) * 12;
    // return _display.measureTextWidth(text, num);
}

void TControlDisplay::print(String s) { _display.print(s); }

void TControlDisplay::println(String s) { _display.println(s); }

void TControlDisplay::setCursor(int16_t x, int16_t y) {
    _display.setCursor(x, y);
}

void TControlDisplay::setRotation(uint8_t r) { _display.setRotation(r); }

void TControlDisplay::setTextColor(uint16_t c) { _display.setTextColor(c); }

void TControlDisplay::setTextSize(uint8_t s) { _display.setTextSize(s); }

bool TControlDisplay::touched() { return _touchScreen->touched(); }

int16_t TControlDisplay::touchedXCharPosition() const {
    TS_Point p = _touchScreen->getPoint();
    int16_t result =
        map(MAX_X_TOUCH_VALUE + MIN_X_TOUCH_VALUE - p.x, MIN_X_TOUCH_VALUE,
            MAX_X_TOUCH_VALUE, 0, _display.width());
    if (result < 0) result = 0;
    if (result > _display.width()) result = _display.width();
    // Serial.print("Touched at X: ");Serial.println(result);
    return result;
}

int16_t TControlDisplay::touchedYCharPosition() const {
    TS_Point p = _touchScreen->getPoint();
    int16_t result =
        map(MAX_Y_TOUCH_VALUE + MIN_Y_TOUCH_VALUE - p.y, MIN_Y_TOUCH_VALUE,
            MAX_Y_TOUCH_VALUE, 0, _display.height());
    if (result < 0) result = 0;
    if (result > _display.height()) result = _display.height();
    // Serial.print("Touched at Y: ");Serial.println(result);
    return result;
}

Adafruit_ILI9341 TControlDisplay::_display = Adafruit_ILI9341(TFT_CS, TFT_DC);

TControlDisplay* TControlDisplay::_singleton;

XPT2046_Touchscreen* TControlDisplay::_touchScreen;
