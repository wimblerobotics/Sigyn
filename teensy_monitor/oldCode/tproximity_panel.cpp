#include "tproximity_panel.h"

#include "tlabeled_on_off_button.h"
#include "tmotor_current.h"
#include "ton_off_button.h"
#include "tpanel_selector.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

TProximityPanel::TProximityPanel() { g_lastDisplayUpdateTime = millis(); }


bool TProximityPanel::hasAlert() {
  return g_tofAlert || g_sonarAlert || g_motorCurrentAlert || g_temperatureAlert;
}


void TProximityPanel::loopTimeOfFlight() {
  g_tofAlert = false;
  for (uint8_t i = 0; i < TTimeOfFlight::NUMBER_TIME_OF_FLIGHT; i++) {
    g_tc.setTextColor(ILI9341_WHITE);
    g_tc.setTextSize(2);
    int valueMm = TTimeOfFlight::singleton().getValueMm(static_cast<TTimeOfFlight::TIMEOFFLIGHT>(i));
    g_tc.fillRect(TOF_BOX_POSITIONS[i][0], TOF_BOX_POSITIONS[i][1],
                  TEXT_SIZE_3_WIDTH * 5 - 1, TEXT_SIZE_3_HEIGHT - 1,
                  PANEL_BACKGROUND_COLOR);
    g_tc.setCursor(TOF_BOX_POSITIONS[i][0], TOF_BOX_POSITIONS[i][1]);
    if (valueMm < 0) {
      g_tc.setTextSize(1);
      g_tc.println("none");
    } else {
      char str[6];
      if (valueMm < 2000) {
        if (valueMm < WARNING_RANGE_MM) {
          g_tofAlert = true;
          g_tc.setTextColor(ILI9341_RED);
        }

        str[0] = ((valueMm / 1000) % 10) + '0';
        str[1] = '.';
        str[2] = ((valueMm / 100) % 10) + '0';
        str[3] = ((valueMm / 10) % 10) + '0';
        str[4] = (valueMm % 10) + '0';
        str[5] = '\0';
        g_tc.println(str);
      } else {
        g_tc.setTextSize(1);
        g_tc.println("rng");
      }
    }
  }
}


void TProximityPanel::loopSonar() {
  g_sonarAlert = false;
  for (uint8_t i = 0; i < TSonar::NUMBER_SONARS; i++) {
    g_tc.setTextColor(ILI9341_WHITE);
    g_tc.setTextSize(2);
    int valueMm = TSonar::singleton().getValueMm(static_cast<TSonar::SONAR>(i));
    g_tc.fillRect(SONAR_BOX_POSITIONS[i][0], SONAR_BOX_POSITIONS[i][1],
                  TEXT_SIZE_3_WIDTH * 5 - 1, TEXT_SIZE_3_HEIGHT - 1,
                  PANEL_BACKGROUND_COLOR);
    g_tc.setCursor(SONAR_BOX_POSITIONS[i][0], SONAR_BOX_POSITIONS[i][1]);
    if (valueMm < 0) {
      g_tc.setTextSize(1);
      g_tc.println("none");
    } else {
      char str[6];
      if (valueMm < 2000) {
        if (valueMm < WARNING_RANGE_MM) {
          g_tc.setTextColor(ILI9341_RED);
          g_sonarAlert = true;
        }

        str[0] = ((valueMm / 1000) % 10) + '0';
        str[1] = '.';
        str[2] = ((valueMm / 100) % 10) + '0';
        str[3] = ((valueMm / 10) % 10) + '0';
        str[4] = (valueMm % 10) + '0';
        str[5] = '\0';
        g_tc.println(str);
      } else {
        g_tc.setTextSize(1);
        g_tc.println("rng");
      }
    }
  }
}


void TProximityPanel::loopMotor() {
  g_motorCurrentAlert = false;
  for (uint8_t i = 0; i < TMotorCurrent::NUMBER_MOTORS; i++) {
    g_tc.setTextColor(ILI9341_WHITE);
    g_tc.setTextSize(2);
    float valueMa = TMotorCurrent::singleton().getValueMa(static_cast<TMotorCurrent::MOTOR>(i));
    g_tc.fillRect(MOTOR_BOX_POSITIONS[i][0], MOTOR_BOX_POSITIONS[i][1],
                  TEXT_SIZE_3_WIDTH * 5 - 1, TEXT_SIZE_3_HEIGHT - 1,
                  PANEL_BACKGROUND_COLOR);
    g_tc.setCursor(MOTOR_BOX_POSITIONS[i][0], MOTOR_BOX_POSITIONS[i][1]);
    if ((valueMa > 9'000) || (valueMa < -9'000)) {
        g_tc.setTextSize(1);
        g_tc.println("rng");
    } else {
      char str[6];
      if ((valueMa > 2'000) || (valueMa < -2'000)) {
        g_tc.setTextColor(ILI9341_RED);
        g_motorCurrentAlert = true;
      }

      if (valueMa < 0) {
        str[0] = '-';
        valueMa = -valueMa;
      } else {
        str[0] = ' ';
      }

      int value =  valueMa / 10;
      str[1] = ((value / 100) % 10) + '0';
      str[2] = '.';
      str[3] = ((value / 10) % 10) + '0';
      str[4] = (value % 10) + '0';
      str[5] = '\0';
      g_tc.println(str);
    }
  }
}



void TProximityPanel::loopTemperature() {
  g_temperatureAlert = false;
  for (uint8_t i = 0; i < TTemperature::NUMBER_TEMPERATURES; i++) {
    g_tc.setTextColor(ILI9341_WHITE);
    g_tc.setTextSize(2);
    int valueTenthsC = TTemperature::singleton().getValueTenthsC(static_cast<TTemperature::TEMPERATURE>(i));
    g_tc.fillRect(TEMP_BOX_POSITIONS[i][0], TEMP_BOX_POSITIONS[i][1],
                  TEXT_SIZE_3_WIDTH * 5 - 1, TEXT_SIZE_3_HEIGHT - 1,
                  PANEL_BACKGROUND_COLOR);
    g_tc.setCursor(TEMP_BOX_POSITIONS[i][0], TEMP_BOX_POSITIONS[i][1]);
    char str[6];
    if (valueTenthsC > WARNING_TEMP_TENTHS_C) {
      g_tc.setTextColor(ILI9341_RED);
      g_temperatureAlert = true;
    }

    sprintf(str, "%3.1f", valueTenthsC / 10.0);
    g_tc.println(str);
  }
}


void TProximityPanel::loop() {
  if (TPanelSelector::singleton().activePanel() ==
      TPanelSelector::PROXIMITY_PANEL) {
    uint32_t now = millis();
    uint32_t durationMs = now - g_lastDisplayUpdateTime;
    if (durationMs > PANEL_UPDATE_DELAY_MS) {
      g_tc.setRotation(1);
      loopTimeOfFlight();
      loopSonar();
      loopMotor();
      loopTemperature();
      g_lastDisplayUpdateTime = now;
    }
  }
}


void TProximityPanel::setup() {
  TOnOffButton::clearAll();
  g_tc.fillScreen(PANEL_BACKGROUND_COLOR);

  g_tc.setTextColor(ILI9341_WHITE);
  g_tc.setRotation(1);

  // Setup time-Of-flight dispaly.
  g_tc.setTextSize(1);
  g_tc.setCursor(BORDER_PAD, TOF_FIRST_LINE_Y);
  g_tc.println("Time of flight sensors");
  uint16_t boxWidth = TEXT_SIZE_3_WIDTH * (5 + 11 + 5) + BORDER_PAD + 1;
  g_tc.drawRect(0, 0, boxWidth,
                TOF_SIXTH_LINE_Y + TEXT_SIZE_3_HEIGHT + BORDER_PAD,
                ILI9341_BLUE);
  g_tc.drawHLine(0, TOF_SECOND_LINE_Y - TEXT_SIZE_3_HEIGHT + 2 + BORDER_PAD,
                 boxWidth, ILI9341_BLUE);
  g_tc.drawHLine(0, TOF_FOURTH_LINE_Y + 6, boxWidth, ILI9341_BLUE);
  g_tc.drawVLine(boxWidth / 2,
                 TOF_SECOND_LINE_Y - TEXT_SIZE_3_HEIGHT + 2 + BORDER_PAD,
                 TEXT_SIZE_3_HEIGHT * 6 + BORDER_PAD - 5, ILI9341_BLUE);
  
  // Setup SONAR display.
  g_tc.setTextSize(1);
  g_tc.setCursor(BORDER_PAD, SONAR_TITLE_Y - 1);
  g_tc.println("SONAR sensors");
  g_tc.setTextSize(2);
  boxWidth = TEXT_SIZE_3_WIDTH * (5 + 5 + 5) + BORDER_PAD + 1;
  g_tc.drawRect(0, SONAR_TITLE_Y - 8, boxWidth,
                TEXT_SIZE_3_HEIGHT * 4 + 6 + BORDER_PAD,
                ILI9341_GREEN);
  g_tc.drawHLine(0, SONAR_SECOND_LINE_Y - TEXT_SIZE_3_HEIGHT + 9 + BORDER_PAD, boxWidth, ILI9341_GREEN);

  // Setup Motors display
  g_tc.setCursor(BORDER_PAD, MOTOR_SECOND_LINE_Y + 2);
  g_tc.setTextSize(1);
  g_tc.println("Motors");
  g_tc.setTextSize(2);
  boxWidth = 50 + (TEXT_SIZE_3_WIDTH * 11) + BORDER_PAD;
  g_tc.drawRect(0, MOTOR_SECOND_LINE_Y - 8, boxWidth,
                TEXT_SIZE_3_HEIGHT * 1 + 7 + BORDER_PAD,
                ILI9341_PINK);
  g_tc.drawVLine(MOTOR_BOX_POSITIONS[0][0] - 7, MOTOR_BOX_POSITIONS[0][1] - BORDER_PAD - 4, TEXT_SIZE_3_HEIGHT + BORDER_PAD * 2 + 2, ILI9341_PINK);
  g_tc.drawVLine(MOTOR_BOX_POSITIONS[1][0] - 7, MOTOR_BOX_POSITIONS[0][1] - BORDER_PAD - 4, TEXT_SIZE_3_HEIGHT + BORDER_PAD * 2 + 2, ILI9341_PINK);

  // Setup Temperature display.
  int tempXStart = TEXT_SIZE_3_WIDTH * (5 + 5 + 5) + BORDER_PAD + 5;
  g_tc.setTextSize(1);
  g_tc.setCursor(tempXStart + 2, TEMP_TITLE_Y - 1);
  g_tc.println("Temp sensors");
  g_tc.setTextSize(2);
  boxWidth = TEXT_SIZE_3_WIDTH * (6) + BORDER_PAD + 1;
  g_tc.drawRect(tempXStart, TEMP_TITLE_Y - 8, boxWidth,
                TEXT_SIZE_3_HEIGHT * 4 + 6 + BORDER_PAD,
                ILI9341_ORANGE);
  g_tc.drawHLine(tempXStart, TEMP_SECOND_LINE_Y - TEXT_SIZE_3_HEIGHT + 9 + BORDER_PAD, boxWidth, ILI9341_ORANGE);
}


TProximityPanel& TProximityPanel::singleton() {
  if (!g_singleton) {
    g_singleton = new TProximityPanel();
  }

  return *g_singleton;
}


bool TProximityPanel::g_tofAlert = false;

bool TProximityPanel::g_sonarAlert = false;

bool TProximityPanel::g_motorCurrentAlert = false;

bool TProximityPanel::g_temperatureAlert = false;

uint32_t TProximityPanel::g_lastDisplayUpdateTime;

TProximityPanel* TProximityPanel::g_singleton = nullptr;

TControlDisplay& TProximityPanel::g_tc = TControlDisplay::singleton();

const int TProximityPanel::TOF_BOX_POSITIONS[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT][2] =
    {
        /*0*/ {TEXT_SIZE_3_WIDTH * 5 + BORDER_PAD, TOF_SECOND_LINE_Y},
        /*1*/ {TEXT_SIZE_3_WIDTH * (5 + 6) + BORDER_PAD, TOF_SECOND_LINE_Y},
        /*2*/ {0 + BORDER_PAD, TOF_THIRD_LINE_Y},
        /*3*/ {TEXT_SIZE_3_WIDTH * (5 + 11) + BORDER_PAD, TOF_THIRD_LINE_Y},
        /*4*/ {0 + BORDER_PAD, TOF_FIFTH_LINE_Y},
        /*5*/ {TEXT_SIZE_3_WIDTH * (5 + 11) + BORDER_PAD, TOF_FIFTH_LINE_Y},
        /*6*/ {TEXT_SIZE_3_WIDTH * 5 + BORDER_PAD, TOF_SIXTH_LINE_Y},
        /*7*/ {TEXT_SIZE_3_WIDTH * (5 + 6) + BORDER_PAD, TOF_SIXTH_LINE_Y}};

const int TProximityPanel::SONAR_BOX_POSITIONS[TSonar::NUMBER_SONARS][2] =
    {
        /*0*/ {TEXT_SIZE_3_WIDTH * 5 + BORDER_PAD, SONAR_SECOND_LINE_Y},
        /*1*/ {0 + BORDER_PAD, SONAR_THIRD_LINE_Y},
        /*2*/ {TEXT_SIZE_3_WIDTH * (5 + 5) + BORDER_PAD, SONAR_THIRD_LINE_Y},
        /*3*/ {TEXT_SIZE_3_WIDTH * 5 + BORDER_PAD, SONAR_FOURTH_LINE_Y}
    };

const int TProximityPanel::MOTOR_BOX_POSITIONS[2][2] =
    {
      /*Left*/ {50, MOTOR_SECOND_LINE_Y},
      /*Right*/ {50 + (TEXT_SIZE_3_WIDTH * 6), MOTOR_SECOND_LINE_Y}
    };

const int TProximityPanel::TEMP_BOX_POSITIONS[TTemperature::NUMBER_TEMPERATURES][2] =
    {
      /*0*/ {TEXT_SIZE_3_WIDTH * (5 + 5 + 5) + BORDER_PAD + 10, TEMP_SECOND_LINE_Y},
      /*1*/ {TEXT_SIZE_3_WIDTH * (5 + 5 + 5) + BORDER_PAD + 10, TEMP_THIRD_LINE_Y}
    };