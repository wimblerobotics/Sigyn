#include "module.h"

#include "serial_manager.h"

Module::Module() { all_modules_[g_next_module_index_++] = this; }

void Module::GetStatistics(char* outString, unsigned int outStringSize) {
  static uint32_t start_usec = micros();
  char statList[2048];

  statList[0] = '\0';

  for (uint16_t i = 0; i < g_next_module_index_; i++) {
    static uint16_t MAXLEN = 256;
    char temp[MAXLEN];
    Module* module = all_modules_[i];
    temp[0] = '\0';
    snprintf(
        temp, MAXLEN,
        "TModuleStats:{\"n\":\"%-s\",\"MnMxAv\":[%-2.1f,%-2.1f,%-2.1f]},",
        module->getName(), module->statistics_.duration_min_us / 1000.0f,
        module->statistics_.duration_max_us / 1000.0f,
        (module->statistics_.duration_sum_us / 1000.0f) /
            (module->total_do_loop_count_ > 0 ? module->total_do_loop_count_
                                              : 1));
    strcat(statList, temp);

    // Reset the duration stats for the next cycle
    module->statistics_.duration_min_us = MAXFLOAT;
    module->statistics_.duration_max_us = 0.0f;
    module->statistics_.duration_sum_us = 0.0f;
  }

  // Remove trailing comma from previous list.
  if (strlen(statList) > 0) {
    statList[strlen(statList) - 1] = '\0';
  }

  snprintf(
      outString, outStringSize, "{\"loops\":%-ld,\"Ms\":%-2.1f,\"mdls\":[%-s]}",
      total_do_loop_count_, ((micros() * 1.0) - start_usec) / 1000.0, statList);
  start_usec = micros();
  total_do_loop_count_ = 0;
}

void Module::Loop() {
  for (uint16_t i = 0; i < g_next_module_index_; i++) {
    uint32_t start_usec = micros();
    Module* module = all_modules_[i];
    all_modules_[i]->loop();
    uint32_t duration = (micros() - start_usec);
    if (duration < module->statistics_.duration_min_us) {
      module->statistics_.duration_min_us = duration;
    }
    if (duration > module->statistics_.duration_max_us) {
      module->statistics_.duration_max_us = duration;
    }
    module->statistics_.duration_sum_us += duration;
  }

  total_do_loop_count_++;

  static unsigned long last_status_send_tim_ms = millis();
  unsigned long current_time_ms = millis();
  if (current_time_ms - last_status_send_tim_ms >=
      1000 /*TM5::kStatusSendInterval*/) {
    last_status_send_tim_ms = current_time_ms;
    char stats[2048];
    GetStatistics(stats, sizeof(stats));
    SerialManager::singleton().SendDiagnosticMessage(stats);
  }
}

void Module::Setup() {
  for (uint16_t i = 0; i < g_next_module_index_; i++) {
    all_modules_[i]->setup();
  }
}

Module* Module::all_modules_[MAX_NUMBER_OF_MODULES];
uint16_t Module::g_next_module_index_ = 0;
uint32_t Module::total_do_loop_count_ = 0;