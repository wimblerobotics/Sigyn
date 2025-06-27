#pragma once
#include <Regexp.h>

#include "SdFat.h"
#include "module.h"
#include "sdios.h"

// A class used to write logging information to an SD memory card.
//
// On setup, the SD card is examined for any existing files with names like
// "LOG12345.TXT", where "12345" is a 5-digit serial number. A new log file
// will be created on the card with a name having a serial number of one higher
// than any existing log file name,or LOG00001.TXT if no existing log files
// were found.

// Every time SdModule::singleton().log() is called, the message is added to a
// string buffer. When the buffer becomes full enough, it is written as one
// large chunck of text to the log file. This is done so that lots of little
// writes are done, which would slow down the outer loop of the Arduino device.
//
// The downside is that if the card is pulled from the Arduino device, the last
// chunk of text won't have been written to the device. Currently, there is a
// fair number of things written to the log file so if you just wait a few
// seconds before pulling the card, the interesting thing you were looking for
// in the log file may have been actually successfully written to the file
// before you pulled the card.
//
// This is intended to be used the the Module software module as part of the
// TeensyMonitor stack. So all you need to do to instantiate and get this module
// going is to call SdModule::singleton() in your ".ino" file before calling
// Module::setup().
//
// If a write to the card fails, perhaps because the card is full or the card
// has been pulled from the Arduino device, further writes are not attempted
// until the system is restarted.

class SdModule : Module {
 public:
  // Singleton constructor.
  static SdModule &singleton();

  // Flush the log file.
  void flush();

  void handleDirMessage(const String &message);

  void handleFileDump(const String &message);

  // Write message to log file.
  void log(const char *message);

 protected:
  // From Module.
  void loop();

  // From Module.
  virtual const char *name() { return "Sd"; }

  void setup();

 private:
  static const unsigned int kChunkSize = 4096;

  // State machine for file dumping
  enum LoopState {
    kDoNothing,
    kDumpingNextLine
  };

  // Private constructor.
  SdModule();

  static void regexpMatchCallback(const char *match, const unsigned int length,
                                  const MatchState &matchState);

  // Used to hold a big chunk of data so writes are fewer.
  String data_buffer_;
  uint32_t last_data_buffer_flush_time_ms_ = 0;

  // The SD card device.
  static SdFs g_sd_;

  // Used to find to highest log file number already on the SD card.
  static int g_highestExistingLogFileNumber_;

  // Has SD device been properly initialized?
  static bool g_initialized_;

  // Cached directory listing (captured during initialization)
  static String cached_directory_listing_;

  // File dump state machine variables
  static LoopState loop_state_;
  static FsFile dump_file_;
  static String dump_filename_;

  // The file handle for the log file on the SD card device.
  static FsFile g_logFile_;

  // Singleton instance.
  static SdModule *g_singleton_;
};