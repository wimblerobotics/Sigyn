#include "tsd.h"

#include <Arduino.h>
#include <Regexp.h>
#include <SD.h>
#include <stdint.h>

// #include "tmicro_ros.h"

void TSd::flush() {
  unsigned int bytes_to_write = 0;
  if ((data_buffer_.length() > 0)) {
    if (data_buffer_.length() < kChunkSize) {
      bytes_to_write = data_buffer_.length();
    } else {
      bytes_to_write = kChunkSize;
    }

    size_t bytes_written =
        g_logFile_.write(data_buffer_.c_str(), bytes_to_write);
    if (bytes_written > 0) {
      g_logFile_.flush();
    } else {
      // Assume the card has been removed or is failing.
      g_initialized_ = false;
    }

    data_buffer_.remove(0);
  }
}

void TSd::log(const char* message) {
  if (g_initialized_) {
    char log_message[kChunkSize];
    uint32_t now = millis();
    snprintf(log_message, sizeof(log_message), "[%07ld.%03ld] %s\n", now / 1000,
             now % 1000, message);
    data_buffer_ += log_message;
    if (true || (data_buffer_.length() >= kChunkSize)) {
      size_t bytes_written =
          g_logFile_.write(data_buffer_.c_str(), data_buffer_.length());
      if (bytes_written > 0) {
        g_logFile_.flush();
      } else {
        // Assume the card has been removed or is failing.
        g_initialized_ = false;
      }

      data_buffer_.remove(0, kChunkSize);
    }
  }
}

void TSd::loop() {}

void TSd::regexpMatchCallback(const char* match, const unsigned int length,
                              const MatchState& matchState) {
  char regexMatchString[10];  // Big enough to hold 5-digit file serial number.
  matchState.GetCapture(regexMatchString, 0);  // Get 0-th match from regexp.
  int logSerialNumberAsInt = atoi(regexMatchString);
  if (logSerialNumberAsInt > TSd::g_highestExistingLogFileNumber_) {
    TSd::g_highestExistingLogFileNumber_ = logSerialNumberAsInt;
  }
}

void TSd::setup() {}

TSd::TSd() : TModule(TModule::kSd) {
  if (!g_initialized_) {
    data_buffer_.reserve(8192);
    g_highestExistingLogFileNumber_ = 0;
    if (!g_sd_.begin(BUILTIN_SDCARD)) {
      // ERROR Unable to access builtin SD card.
    } else {
      File rootDirectory = g_sd_.open("/");
      while (true) {
        File nextFileInDirectory = rootDirectory.openNextFile();
        if (!nextFileInDirectory) break;
        char fileName[256];
        strncpy(fileName, nextFileInDirectory.name(), sizeof(fileName));
        MatchState matchState;
        matchState.Target(fileName);
        matchState.GlobalMatch("LOG(%d+).TXT", regexpMatchCallback);
      }

      char newLogFileName[20];  // Big enough to hold file name like:
                                // LOG12345.TXT.
      sprintf(newLogFileName, "LOG%05d.TXT", ++g_highestExistingLogFileNumber_);
      g_logFile_ = g_sd_.open(newLogFileName, FILE_WRITE);
    }

    g_initialized_ = true;
  }
}

TSd& TSd::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new TSd();
  }

  return *g_singleton_;
}

SDClass TSd::g_sd_;

int TSd::g_highestExistingLogFileNumber_ = 0;

bool TSd::g_initialized_ = false;

File TSd::g_logFile_;

TSd* TSd::g_singleton_ = nullptr;