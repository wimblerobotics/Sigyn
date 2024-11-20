#include "tsd.h"

#include <Arduino.h>
#include <Regexp.h>
#include <SD.h>
#include <stdint.h>

void TSd::log(const char* message) {
  if (g_initialized) {
  char prefix[20]; // Big enough to hold '[1234567.890] '.
  uint32_t now = millis();
  sprintf(prefix, "[%07ld.%03ld] ", now / 1000, now % 1000);
  g_logFile.print(prefix);
  g_logFile.println(message);
  g_logFile.flush();
  }
}

void TSd::loop() {}

static int g_highestExistingLogFileNumber = 0;
void regexpMatchCallback(const char* match, const unsigned int length,
                         const MatchState& matchState) {
  char regexMatchString[10];  // Big enough to hold 5-digit file serial number.
  matchState.GetCapture(regexMatchString, 0);  // Get 0-th match from regexp.
  int logSerialNumberAsInt = atoi(regexMatchString);
  if (logSerialNumberAsInt > g_highestExistingLogFileNumber) {
    g_highestExistingLogFileNumber = logSerialNumberAsInt;
  }
}

void TSd::setup() {
  g_highestExistingLogFileNumber = 0;
  g_initialized = false;
  if (!g_sd.begin(BUILTIN_SDCARD)) {
    Serial.println("[TSd::setup] Unable to access builtin SD card");
  } else {
    File rootDirectory = g_sd.open("/");
    while (true) {
      File nextFileInDirectory = rootDirectory.openNextFile();
      if (!nextFileInDirectory) break;
      char* fileName = nextFileInDirectory.name();
      MatchState matchState;
      matchState.Target(fileName);
      matchState.GlobalMatch("LOG(%d+).TXT", regexpMatchCallback);
    }

    char newLogFileName[20]; // Big enough to hold file name like: LOG12345.TXT.
    sprintf(newLogFileName, "LOG%05d.TXT", ++g_highestExistingLogFileNumber);
    g_logFile = g_sd.open(newLogFileName, FILE_WRITE);
    if (!g_logFile) {
      Serial.print("[TSd::setup] Unable to create new log file: '");
      Serial.print(newLogFileName);
      Serial.println("'");
    } else {
      g_initialized = true;
      millis();
    }
  }
}

TSd::TSd() : TModule() {}

TSd& TSd::singleton() {
  if (!g_singleton) {
    g_singleton = new TSd();
  }

  return *g_singleton;
}

SDClass TSd::g_sd;

bool TSd::g_initialized = false;

File TSd::g_logFile;

TSd* TSd::g_singleton = nullptr;