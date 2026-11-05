#pragma once
#include <Arduino.h>

// Elapsed microsecond profiler for the application loop; not CPU task percentages.
enum PerfStage : uint8_t {
  PERF_COMM = 0,
  PERF_ADC,
  PERF_PID,
  PERF_TX,
  PERF_STAGE_COUNT
};

void perfLoopBegin(uint32_t startUs);
void perfRecord(PerfStage stage, uint32_t elapsedUs);
void perfLoopEnd(uint32_t bodyUs);
void perfReport();      // SPP-only PERF command, resets measurement interval.
void perfTaskReport();  // SPP-only TASKS command if runtime stats are available.
