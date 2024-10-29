#include "tpower_panel.h"

#include "tlabeled_on_off_button.h"
#include "ton_off_button.h"
#include "trelay.h"


TPowerPanel::TPowerPanel() {
}


void TPowerPanel::loop() {
}


void TPowerPanel::motorOnOffCallback(TOnOffButton &button, void *parameter) {
  button.setState(button.state() == TOnOffButton::ON ? TOnOffButton::OFF
                                                     : TOnOffButton::ON);
  if (button.state() == TOnOffButton::ON) {
    TRelay::singleton().powerOn(TRelay::MOTOR_POWER);
  } else {
    TRelay::singleton().powerOff(TRelay::MOTOR_POWER);
  }
}


void TPowerPanel::intelOnOffCallback(TOnOffButton &button, void *parameter) {
  button.setState(button.state() == TOnOffButton::ON ? TOnOffButton::OFF
                                                     : TOnOffButton::ON);
  if (button.state() == TOnOffButton::ON) {
    TRelay::singleton().powerOn(TRelay::INTEL_POWER);
  } else {
    TRelay::singleton().powerOff(TRelay::INTEL_POWER);
  }
}


void TPowerPanel::intelResetCallback(TOnOffButton &button, void *parameter) {
  TRelay::singleton().powerOn(TRelay::INTEL_RESET);
}


void TPowerPanel::nvidiaOnOffCallback(TOnOffButton &button, void *parameter) {
  button.setState(button.state() == TOnOffButton::ON ? TOnOffButton::OFF
                                                     : TOnOffButton::ON);
  if (button.state() == TOnOffButton::ON) {
    TRelay::singleton().powerOn(TRelay::NVIDIA_POWER);
  } else {
    TRelay::singleton().powerOff(TRelay::NVIDIA_POWER);
  }
}


void TPowerPanel::nvidiaResetCallback(TOnOffButton &button, void *parameter) {
  TRelay::singleton().powerOn(TRelay::NVIDIA_RESET);
}


void TPowerPanel::setup() {
  g_tc.fillScreen(PANEL_BACKGROUND_COLOR);
  g_tc.setTextColor(ILI9341_WHITE);
  g_tc.setTextSize(3);

  bool motorOn = TRelay::singleton().isPoweredOn(TRelay::MOTOR_POWER);
  bool intelOn = TRelay::singleton().isPoweredOn(TRelay::INTEL_POWER);
  bool intelResetOn = TRelay::singleton().isPoweredOn(TRelay::INTEL_RESET);
  bool nvidiaOn = TRelay::singleton().isPoweredOn(TRelay::NVIDIA_POWER);
  bool nvidiaResetOn = TRelay::singleton().isPoweredOn(TRelay::NVIDIA_RESET);
  TOnOffButton::TButtonState motorPowerButtonState = motorOn ? TOnOffButton::ON : TOnOffButton::OFF;
  TOnOffButton::TButtonState intelPowerButtonState = intelOn ? TOnOffButton::ON : TOnOffButton::OFF;
  TOnOffButton::TButtonState intelResetButtonState = intelResetOn ? TOnOffButton::ON : TOnOffButton::OFF;
  TOnOffButton::TButtonState nvidiaPowerButtonState = nvidiaOn ? TOnOffButton::ON : TOnOffButton::OFF;
  TOnOffButton::TButtonState nvidiaResetButtonState = nvidiaResetOn ? TOnOffButton::ON : TOnOffButton::OFF;
  TLabeledOnOffButton::makeButton("MOTORS", 1, 6, 2, 50, 100, 50, ILI9341_RED,
                                  ILI9341_WHITE, "ON", "OFF", motorPowerButtonState,
                                  motorOnOffCallback, nullptr, 1);
  TLabeledOnOffButton::makeButton("Intel", 1, 6, 58, 50, 100, 50, ILI9341_RED,
                                  ILI9341_WHITE, "ON", "OFF", intelPowerButtonState,
                                  intelOnOffCallback, nullptr, 2);
  TOnOffButton::makeButton(1, 6 + 150 + 8, 58, 60, 50, ILI9341_ORANGE,
                           ILI9341_WHITE, "RESET", "RESET", intelResetButtonState,
                           intelResetCallback, nullptr, 3);
  TLabeledOnOffButton::makeButton("Nvidia", 1, 6, 114, 50, 100, 50, ILI9341_RED,
                                  ILI9341_WHITE, "ON3", "OFF", nvidiaPowerButtonState,
                                  nvidiaOnOffCallback, nullptr, 4);
  TOnOffButton::makeButton(1, 6 + 150 + 8, 114, 60, 50, ILI9341_ORANGE,
                           ILI9341_WHITE, "RESET", "RESET", nvidiaResetButtonState,
                           nvidiaResetCallback, nullptr, 5);
}


TPowerPanel &TPowerPanel::singleton() {
  if (!g_singleton) {
    g_singleton = new TPowerPanel();
  }

  return *g_singleton;
}


TPowerPanel *TPowerPanel::g_singleton = nullptr;

TControlDisplay &TPowerPanel::g_tc = TControlDisplay::singleton();
