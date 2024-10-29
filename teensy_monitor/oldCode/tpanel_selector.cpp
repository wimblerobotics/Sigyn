#include "tpanel_selector.h"

#include "ton_off_button.h"
#include "tpower_panel.h"
#include "tproximity_panel.h"

TPanelSelector::TPanelSelector() { g_selectedPanel = PROXIMITY_PANEL; }

void TPanelSelector::colorizeAlertIcon(uint16_t color) {
  g_tc.fillRect(270, 190, 50, 50, color);
}

void TPanelSelector::loop() {
  static uint32_t periodStart = millis();  // For timing some-alert flashing.
  static bool useAlertColor =
      true;  // some-alert highlight color to use (alert color/bacground color).
  static bool isFlashing = false;  // Is flashing occurring?

  TOnOffButton::update();
  TPowerPanel::singleton().loop();
  TProximityPanel::singleton().loop();
  if (TProximityPanel::singleton().hasAlert()) {
    // Flash the alert icon.
    uint32_t duration = millis() - periodStart;
    uint16_t backgroundColor = g_selectedPanel == PROXIMITY_PANEL
                                   ? TProximityPanel::PANEL_BACKGROUND_COLOR
                                   : TPowerPanel::PANEL_BACKGROUND_COLOR;
    if (duration > ALERT_ALTERNATING_FLASH_DURATION_MS) {
      // Time to alternate alert color.
      if (useAlertColor) {
        colorizeAlertIcon(ILI9341_RED);
      } else {
        colorizeAlertIcon(backgroundColor);
      }

      useAlertColor = !useAlertColor;
      periodStart = millis();
    }

    isFlashing = true;
  } else {
    if (isFlashing) {
      colorizeAlertIcon(ILI9341_GREEN);
      isFlashing = false;
    }
  }
}

void TPanelSelector::powerPanelCallback(TOnOffButton &button, void *parameter) {
  TControlDisplay::singleton().clearAll();
  TOnOffButton::clearAll();
  g_selectedPanel = POWER_PANEL;
  TPanelSelector::singleton().setup();
}

void TPanelSelector::proximityPanelCallback(TOnOffButton &button,
                                            void *parameter) {
  TControlDisplay::singleton().clearAll();
  TOnOffButton::clearAll();
  g_selectedPanel = PROXIMITY_PANEL;
  TPanelSelector::singleton().setup();
}

void TPanelSelector::setup() {
  if (g_selectedPanel == POWER_PANEL) {
    TPowerPanel::singleton().setup();
  } else if (g_selectedPanel == PROXIMITY_PANEL) {
    TProximityPanel::singleton().setup();
  }

  TOnOffButton::makeButton(0, 155, 270, 85, 50, ILI9341_BLACK, ILI9341_WHITE,
                           " POWER", "POWER", TOnOffButton::ON,
                           powerPanelCallback, nullptr);

  TOnOffButton::makeButton(0, 80, 270, 75, 50, ILI9341_OLIVE, ILI9341_WHITE,
                           "SENSE", "SENSE", TOnOffButton::ON,
                           proximityPanelCallback, nullptr);
  colorizeAlertIcon(ILI9341_GREEN);
}

TPanelSelector &TPanelSelector::singleton() {
  if (!g_singleton) {
    g_singleton = new TPanelSelector();
  }

  return *g_singleton;
}

TPanelSelector::TPanel TPanelSelector::g_selectedPanel =
    TPanelSelector::POWER_PANEL;

TPanelSelector *TPanelSelector::g_singleton = nullptr;

TControlDisplay &TPanelSelector::g_tc = TControlDisplay::singleton();
