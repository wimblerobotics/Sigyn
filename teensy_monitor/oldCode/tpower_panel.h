#pragma once

#include "ton_off_button.h"

#include "tcontrol_display.h"
#include "tpanel_selector.h"

class TPowerPanel {
public:

    static TPowerPanel& singleton();

    void loop();
    
    void setup();

    static const uint16_t PANEL_BACKGROUND_COLOR = ILI9341_BLUE;
    
private:

    TPowerPanel();

    static void motorOnOffCallback(TOnOffButton& button, void* parameter);

    static void intelOnOffCallback(TOnOffButton& button, void* parameter);

    static void intelResetCallback(TOnOffButton& button, void* parameter);

    static void nvidiaOnOffCallback(TOnOffButton& button, void* parameter);

    static void nvidiaResetCallback(TOnOffButton& button, void* parameter);


    static TPanelSelector& g_panelSelector;

    static TPowerPanel* g_singleton;

    static TControlDisplay& g_tc;
};