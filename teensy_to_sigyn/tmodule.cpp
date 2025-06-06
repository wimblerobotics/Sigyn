#include "tmodule.h"

#include <Arduino.h>
#include <stdint.h>

#include "tconfiguration.h"
// #include "tmicro_ros.h"
#define DO_TIMING true

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else   // __ARM__
extern char* __brkval;
#endif  // __arm__

TModule::TModule(TModule::Module moduleKind) {
  all_modules_[moduleKind] = this;
  loop_calls_between_get_statistics_calls_ = 0;
  for (size_t i = 0; i < kNumberSlots; i++) {
    duration_stats_[i] = 0.0;
  }
}

void TModule::GetStatistics(char* outString, size_t outStringSize) {
  static uint32_t statTimingStart = micros();
  char statList[2048];

  statList[0] = '\0';

  for (size_t i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      TModule* module = all_modules_[i];
      static size_t MAXLEN = 512;
      char temp[MAXLEN];
      temp[0] = '\0';
      snprintf(temp, MAXLEN,
               "{\"n\":\"%-s\",\"MnMxAv\":[%-2.1f,%-2.1f,%-2.1f]},",
               all_modules_[i]->name(), module->duration_stats_[kMin],
               module->duration_stats_[kMax],
               module->duration_stats_[kSum] / total_do_loop_count_);
      strcat(statList, temp);
      module->loop_calls_between_get_statistics_calls_ = 0;
      module->duration_stats_[kMin] = 10'000'000.0;
      module->duration_stats_[kMax] = -10'000'000.0;
      module->duration_stats_[kSum] = 0.0;
    }
  }

  // Remove trailing comma from previous list.
  if (strlen(statList) > 0) {
    statList[strlen(statList) - 1] = '\0';
  }

  snprintf(outString, outStringSize,
           "{\"loops\":%-ld,\"Ms\":%-2.1f,\"mdls\":[%-s]}",
           total_do_loop_count_, ((micros() * 1.0) - statTimingStart) / 1000.0,
           statList);
  statTimingStart = micros();
  total_do_loop_count_ = 0;
}

void TModule::DoLoop() {
  char diagnostic_message[256];
  if (TM5::kDoDetailDebug) {
    // TMicroRos::singleton().PublishDiagnostic("INFO [TModule::DoLoop] >> enter");
  }

  // {
  //   extern char _ebss[], _heap_end[], *__brkval;
  //   auto sp = (char*)__builtin_frame_address(0);
  //   auto stack = sp - _ebss;
  //   auto heap = _heap_end - __brkval;
  //   snprintf(diagnostic_message, sizeof(diagnostic_message),
  //            "INFO [TModule::DoLoop] free stack kb %d, free heap kb: %d",
  //            stack >> 10, heap >> 10);
  //   TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  // }

  for (size_t i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      TModule* module = all_modules_[i];
      if (TM5::kDoDetailDebug) {
        snprintf(diagnostic_message, sizeof(diagnostic_message),
                 "INFO [TModule::DoLoop] about to loop on: %s",
                 all_modules_[i]->name());
        // TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
      }

      uint32_t start = micros();

      all_modules_[i]->loop();

      float duration_ms = (micros() - start) / 1000.0;
      module->duration_stats_[kSum] += duration_ms;
      if (duration_ms < module->duration_stats_[kMin]) {
        module->duration_stats_[kMin] = duration_ms;
      }

      if (duration_ms > module->duration_stats_[kMax]) {
        module->duration_stats_[kMax] = duration_ms;
      }

      module->loop_calls_between_get_statistics_calls_++;
    }
  }

  total_do_loop_count_++;
  if (TM5::kDoDetailDebug) {
    snprintf(diagnostic_message, sizeof(diagnostic_message),
             "INFO [TModule::DoLoop] << exit, total_do_loop_count: %ld",
             total_do_loop_count_);
    // TMicroRos::singleton().PublishDiagnostic(diagnostic_message);
  }
}

void TModule::DoSetup() {
  if (TM5::kDoDetailDebug) {
    // TMicroRos::singleton().PublishDiagnostic(
    //     "INFO [TModule::DoSetup] >> enter");
  }

  for (int i = 0; i < kNumberModules; i++) {
    if (all_modules_[i] != nullptr) {
      all_modules_[i]->setup();
    }
  }

  if (TM5::kDoDetailDebug) {
    // TMicroRos::singleton().PublishDiagnostic("INFO [TModule::DoSetup] << exit");
  }
}

TModule* TModule::all_modules_[TModule::kNumberModules + 1] = {
    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr};

uint32_t TModule::total_do_loop_count_ = 0;
