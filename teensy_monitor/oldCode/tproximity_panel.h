#pragma once

#include "tcontrol_display.h"
#include "ton_off_button.h"
#include "tsonar.h"
#include "ttemperature.h"
#include "ttime_of_flight.h"

class TProximityPanel {
 public:
  static TProximityPanel& singleton();

  bool hasAlert();

  void loop();

  void setup();

  static const uint16_t PANEL_BACKGROUND_COLOR = ILI9341_DARKGREEN;

 private:
  TProximityPanel();

  void loopTimeOfFlight();
  void loopSonar();
  void loopMotor();
  void loopTemperature();

  static bool g_tofAlert;
  static bool g_sonarAlert;
  static bool g_motorCurrentAlert;
  static bool g_temperatureAlert;

  static uint32_t g_lastDisplayUpdateTime;

  static TProximityPanel* g_singleton;

  static TControlDisplay& g_tc;

  static const uint32_t PANEL_UPDATE_DELAY_MS = 100;

  // Proximity sensor range less than this is an alert.
  static const int WARNING_RANGE_MM = 3 * 25.4;

  // Temperature sensor range greater than this is an alert.
  static const int WARNING_TEMP_TENTHS_C = 350;

  static const uint16_t BORDER_PAD = 3;
  static const uint16_t TEXT_SIZE_3_HEIGHT = 16; /* By inspection. */
  static const uint16_t TEXT_SIZE_3_WIDTH = 12;  /* By inspection */
  static const uint16_t TOF_FIRST_LINE_Y = TEXT_SIZE_3_HEIGHT + BORDER_PAD - 10;
  static const uint16_t TOF_SECOND_LINE_Y =
      (TEXT_SIZE_3_HEIGHT * 2) + BORDER_PAD;
  static const uint16_t TOF_THIRD_LINE_Y =
      (TEXT_SIZE_3_HEIGHT * 3) + BORDER_PAD;
  static const uint16_t TOF_FOURTH_LINE_Y =
      (TEXT_SIZE_3_HEIGHT * 4) + BORDER_PAD;
  static const uint16_t TOF_FIFTH_LINE_Y =
      (TEXT_SIZE_3_HEIGHT * 5) + BORDER_PAD;
  static const uint16_t TOF_SIXTH_LINE_Y =
      (TEXT_SIZE_3_HEIGHT * 6) + BORDER_PAD;
  static const int TOF_BOX_POSITIONS[TTimeOfFlight::NUMBER_TIME_OF_FLIGHT][2];

  static const uint16_t SONAR_TITLE_Y =
      TOF_SIXTH_LINE_Y + (TEXT_SIZE_3_HEIGHT * 3) + BORDER_PAD - 10;
  static const uint16_t SONAR_SECOND_LINE_Y =
      SONAR_TITLE_Y + TEXT_SIZE_3_HEIGHT;
  static const uint16_t SONAR_THIRD_LINE_Y =
      SONAR_SECOND_LINE_Y + TEXT_SIZE_3_HEIGHT;
  static const uint16_t SONAR_FOURTH_LINE_Y =
      SONAR_THIRD_LINE_Y + TEXT_SIZE_3_HEIGHT;
  static const int SONAR_BOX_POSITIONS[TSonar::NUMBER_SONARS][2];

  static const uint16_t MOTOR_SECOND_LINE_Y =
      SONAR_FOURTH_LINE_Y + (TEXT_SIZE_3_HEIGHT * 2) + BORDER_PAD - 4;
  static const int MOTOR_BOX_POSITIONS[2][2];

  static const uint16_t TEMP_TITLE_Y =
      TOF_SIXTH_LINE_Y + (TEXT_SIZE_3_HEIGHT * 3) + BORDER_PAD - 10;
  static const uint16_t TEMP_SECOND_LINE_Y =
      TEMP_TITLE_Y + TEXT_SIZE_3_HEIGHT;
  static const uint16_t TEMP_THIRD_LINE_Y =
      TEMP_SECOND_LINE_Y + TEXT_SIZE_3_HEIGHT;
  static const int TEMP_BOX_POSITIONS[TTemperature::NUMBER_TEMPERATURES][2];

};