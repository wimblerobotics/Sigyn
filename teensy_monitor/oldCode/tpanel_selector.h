#pragma once

#include <stdint.h>

#include "tcontrol_display.h"
#include "tmodule.h"
#include "ton_off_button.h"

class TPanelSelector : TModule {
 public:
  // The different touch screen panels.
  typedef enum { POWER_PANEL, PROXIMITY_PANEL } TPanel;

  // Which panel is currently active.
  TPanel activePanel() { return g_selectedPanel; }

  // From TModule.
  void loop();

  // From TModule.
  const char* name() { return "TPanelSelector"; }

  // From TModule.
  void setup();

  // Singleton instance.
  static TPanelSelector& singleton();

 private:
  // Private constructor.
  TPanelSelector();

  // Set the alert icon to the given color.
  void colorizeAlertIcon(uint16_t color);

  // Callback to handle screen press for selecting power panel.
  static void powerPanelCallback(TOnOffButton& button, void* parameter);

  // Callback to handle screen press for selecting power panel.
  static void proximityPanelCallback(TOnOffButton& button, void* parameter);

  // Currently selected panel.
  static TPanel g_selectedPanel;

  // Singleton instance.
  static TPanelSelector* g_singleton;

  // Touchscreen display device.
  static TControlDisplay& g_tc;

  // Duration (ms) between alert/background color changes when alert is flashing.
  static const int ALERT_ALTERNATING_FLASH_DURATION_MS = 500;
};
