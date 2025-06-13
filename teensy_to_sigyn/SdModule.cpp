#include "SdModule.h"

#include <Arduino.h>
#include <Regexp.h>
#include <SdFat.h>
#include <stdint.h>

#include "SerialManager.h"
#include "sdios.h"

// #include "tmicro_ros.h"

void SdModule::flush() {
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

void SdModule::handleDirMessage(const String &message) {
  char msg[16384];  // Buffer for diagnostic messages, large enough to hold
                    // directory listing.
  snprintf(msg, sizeof(msg),
           "[SdModule::handleDirMessage] Received directory "
           "message: %s",
           message.c_str());
  SerialManager::singleton().SendDiagnosticMessage(msg);

  msg[0] = '\0';  // Clear the message buffer.
  snprintf(msg, sizeof(msg), "SDIR:");

  if (!g_initialized_) {
    snprintf(msg + strlen(msg), sizeof(msg) - strlen(msg),
             "ERROR: SD card not initialized\n");
    SerialManager::singleton().SendDiagnosticMessage(msg);
    return;
  }

  // Use cached directory listing - fast operation!
  SerialManager::singleton().SendDiagnosticMessage(
      "[SdModule] Using cached directory listing");

  size_t currentLen = strlen(msg);
  size_t remainingSpace = sizeof(msg) - currentLen - 1;
  size_t cachedLen = cached_directory_listing_.length();

  if (remainingSpace > cachedLen) {
    snprintf(msg + currentLen, remainingSpace, "%s",
             cached_directory_listing_.c_str());
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[SdModule::handleDirMessage] Cached listing too large for buffer");
    snprintf(msg + currentLen, remainingSpace,
             "ERROR: Directory listing too large");
  }

  SerialManager::singleton().SendDiagnosticMessage(msg);
}

void SdModule::handleFileDump(const String &message) {
  char msg[512];
  snprintf(msg, sizeof(msg),
           "[SdModule::handleFileDump] Received file dump request: %s",
           message.c_str());
  SerialManager::singleton().SendDiagnosticMessage(msg);

  if (!g_initialized_) {
    SerialManager::singleton().SendDiagnosticMessage(
        "SDLINE:ERROR: SD card not initialized\r");
    SerialManager::singleton().SendDiagnosticMessage("SDEOF:\n");
    return;
  }

  if (loop_state_ != kDoNothing) {
    SerialManager::singleton().SendDiagnosticMessage(
        "SDLINE:ERROR: File dump already in progress\r");
    SerialManager::singleton().SendDiagnosticMessage("SDEOF:\n");
    return;
  }

  // Extract filename from message (everything after the colon)
  dump_filename_ = message;
  if (dump_filename_.length() == 0) {
    SerialManager::singleton().SendDiagnosticMessage(
        "SDLINE:ERROR: No filename specified\r");
    SerialManager::singleton().SendDiagnosticMessage("SDEOF:\n");
    return;
  }

  // Try to open the file
  dump_file_ = g_sd_.open(dump_filename_.c_str(), FILE_READ);
  if (!dump_file_) {
    snprintf(msg, sizeof(msg), "SDLINE:ERROR: Could not open file '%s'\r",
             dump_filename_.c_str());
    SerialManager::singleton().SendDiagnosticMessage(msg);
    SerialManager::singleton().SendDiagnosticMessage("SDEOF:\n");
    return;
  }

  // Set state to start dumping
  loop_state_ = kDumpingNextLine;
  snprintf(msg, sizeof(msg), "[SdModule] Started dumping file: %s",
           dump_filename_.c_str());
  SerialManager::singleton().SendDiagnosticMessage(msg);
}

void SdModule::log(const char *message) {
  if (g_initialized_) {
    char log_message[kChunkSize];
    uint32_t now = millis();
    snprintf(log_message, sizeof(log_message), "[%07ld.%03ld] %s\n", now / 1000,
             now % 1000, message);
    data_buffer_ += log_message;
    if (data_buffer_.length() >= kChunkSize) {
      size_t bytes_written =
          g_logFile_.write(data_buffer_.c_str(), data_buffer_.length());
      if (bytes_written > 0) {
        g_logFile_.flush();
        last_data_buffer_flush_time_ms_ = millis();
      } else {
        // Assume the card has been removed or is failing.
        g_initialized_ = false;
      }

      data_buffer_.remove(0, kChunkSize);
    }
  }
}

void SdModule::loop() {
  switch (loop_state_) {
    case kDoNothing:
      // Nothing to do, waiting for a file dump request
      if ((millis() - last_data_buffer_flush_time_ms_) > 1000) {
        // Flush every second if there's data
        if (data_buffer_.length() > 0) {
          // Flush any remaining data in the buffer
          size_t bytes_written =
              g_logFile_.write(data_buffer_.c_str(), data_buffer_.length());
          if (bytes_written > 0) {
            g_logFile_.flush();
            data_buffer_.remove(0, bytes_written);
          } else {
            // Assume the card has been removed or is failing.
            g_initialized_ = false;
          }
        }

        last_data_buffer_flush_time_ms_ = millis();
      }
      break;

    case kDumpingNextLine:
     if (dump_file_.available()) {
        // Read one line from the file
        String line = dump_file_.readStringUntil('\n');

        // Remove carriage return if present
        if (line.endsWith("\r")) {
          line.remove(line.length() - 1);
        }

        // Send the line with SDLINE prefix and carriage return
        String response = "SDLINE:" + line + "\r";
        SerialManager::singleton().SendDiagnosticMessage(response.c_str());
      } else {
        // End of file reached
        SerialManager::singleton().SendDiagnosticMessage("SDEOF:\n");
        dump_file_.close();
        loop_state_ = kDoNothing;

        char msg[256];
        snprintf(msg, sizeof(msg), "[SdModule] Completed dumping file: %s",
                 dump_filename_.c_str());
        SerialManager::singleton().SendDiagnosticMessage(msg);
        dump_filename_ = "";
      }
      break;
    default:
      // Unknown state, reset to do nothing
      SerialManager::singleton().SendDiagnosticMessage(
          "[SdModule] ERROR: Unknown loop state, resetting to kDoNothing");
      loop_state_ = kDoNothing;
      dump_file_.close();
      dump_filename_ = "";
      break;
  }
}

void SdModule::regexpMatchCallback(const char *match, const unsigned int length,
                                   const MatchState &matchState) {
  char regexMatchString[10];  // Big enough to hold 5-digit file serial number.
  matchState.GetCapture(regexMatchString, 0);  // Get 0-th match from regexp.
  int logSerialNumberAsInt = atoi(regexMatchString);
  if (logSerialNumberAsInt > SdModule::g_highestExistingLogFileNumber_) {
    SdModule::g_highestExistingLogFileNumber_ = logSerialNumberAsInt;
  }
}

void SdModule::setup() {}

SdModule::SdModule() : TModule() {
  SerialManager::singleton().SendDiagnosticMessage(
      "[SdModule] Starting full initialization...");
  data_buffer_.reserve(8192);
  g_highestExistingLogFileNumber_ = 0;
  cached_directory_listing_ = "";

  // Initialize file dump state machine
  loop_state_ = kDoNothing;
  dump_filename_ = "";

  if (!g_sd_.begin(SdioConfig(FIFO_SDIO))) {
    SerialManager::singleton().SendDiagnosticMessage(
        "[SdModule] ERROR: Failed to initialize SD card");
  } else {
    SerialManager::singleton().SendDiagnosticMessage(
        "[SdModule] SD card initialized, scanning for log files...");
    FsFile rootDirectory = g_sd_.open("/");

    // Scan for highest log file number AND build cached directory listing
    while (true) {
      FsFile nextFileInDirectory = rootDirectory.openNextFile();
      if (!nextFileInDirectory) {
        break;
      }
      char fileName[256];
      nextFileInDirectory.getName(fileName, sizeof(fileName));
      
      // Get file size before closing
      uint32_t fileSize = nextFileInDirectory.size();
      nextFileInDirectory.close();

      // Add to cached directory listing with filename,size format
      if (cached_directory_listing_.length() > 0) {
        cached_directory_listing_ += "\t";
      }
      cached_directory_listing_ += fileName;
      cached_directory_listing_ += ",";
      cached_directory_listing_ += String(fileSize);

      // Check if this is a log file for numbering
      MatchState matchState;
      matchState.Target(fileName);
      matchState.GlobalMatch("LOG(%d+).TXT", regexpMatchCallback);
    }

    rootDirectory.close();
    SerialManager::singleton().SendDiagnosticMessage(
        "[SdModule] File scanning complete, creating log file...");

    char newLogFileName[20];
    sprintf(newLogFileName, "LOG%05d.TXT", ++g_highestExistingLogFileNumber_);
    g_logFile_ = g_sd_.open(newLogFileName, FILE_WRITE);

    // Add the new log file to the cached directory listing with size 0
    if (cached_directory_listing_.length() > 0) {
      cached_directory_listing_ += "\t";
    }
    cached_directory_listing_ += newLogFileName;
    cached_directory_listing_ += ",0";  // New file starts with 0 bytes

    SerialManager::singleton().SendDiagnosticMessage(
        "[SdModule] Log file created successfully");
  }

  g_initialized_ = true;
  SerialManager::singleton().SendDiagnosticMessage(
      "[SdModule] Initialization complete");
  this->log("[SdModule::SdModule] Compiled on: " __DATE__ ", " __TIME__);
}

SdModule &SdModule::singleton() {
  if (!g_singleton_) {
    g_singleton_ = new SdModule();
  }

  return *g_singleton_;
}

SdFs SdModule::g_sd_;

int SdModule::g_highestExistingLogFileNumber_ = 0;

bool SdModule::g_initialized_ = false;

String SdModule::cached_directory_listing_ = "";

SdModule::FileDumpState SdModule::loop_state_ = SdModule::kDoNothing;

FsFile SdModule::dump_file_;

String SdModule::dump_filename_ = "";

FsFile SdModule::g_logFile_;

SdModule *SdModule::g_singleton_ = nullptr;