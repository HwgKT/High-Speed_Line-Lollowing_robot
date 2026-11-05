#include "perf_monitor.h"
#include "spp_debug.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {
struct Metric {
  uint64_t sumUs = 0;
  uint32_t samples = 0;
  uint32_t minUs = 0;
  uint32_t maxUs = 0;
};

Metric stages[PERF_STAGE_COUNT];
Metric periodMetric; // loop-start to loop-start, including yield/RTOS preemption
Metric bodyMetric;   // firmware loop body, excluding final yield/PERF printing
uint32_t previousLoopStartUs = 0;
bool previousLoopValid = false;

void add(Metric &m, uint32_t us) {
  if (!m.samples || us < m.minUs) m.minUs = us;
  if (us > m.maxUs) m.maxUs = us;
  m.sumUs += us;
  ++m.samples;
}
float mean(const Metric &m) {
  return m.samples ? (float)((double)m.sumUs / m.samples) : 0.0f;
}
void reset(Metric &m) {
  m.sumUs = 0;
  m.samples = 0;
  m.minUs = 0;
  m.maxUs = 0;
}
}

void perfLoopBegin(uint32_t startUs) {
  if (previousLoopValid) add(periodMetric, startUs - previousLoopStartUs);
  previousLoopStartUs = startUs;
  previousLoopValid = true;
}
void perfRecord(PerfStage stage, uint32_t elapsedUs) {
  if (stage < PERF_STAGE_COUNT) add(stages[stage], elapsedUs);
}
void perfLoopEnd(uint32_t bodyUs) {
  add(bodyMetric, bodyUs);
}
void perfReport() {
  const Metric &com = stages[PERF_COMM];
  const Metric &adc = stages[PERF_ADC];
  const Metric &pid = stages[PERF_PID];
  const Metric &tx = stages[PERF_TX];

  // Time not attributed to the timed regions (button, safety, logging, etc.).
  const uint64_t accounted = com.sumUs + adc.sumUs + pid.sumUs + tx.sumUs;
  const uint64_t other = bodyMetric.sumUs > accounted ? bodyMetric.sumUs - accounted : 0;
  const float otherAvg = bodyMetric.samples
      ? (float)((double)other / bodyMetric.samples) : 0.0f;
  const float avgBody = mean(bodyMetric);
  const float avgAdc = mean(adc);
  const float bodyCeilingHz = avgBody > 0 ? 1000000.0f / avgBody : 0.0f;
  const float adcCeilingHz = avgAdc > 0 ? 1000000.0f / avgAdc : 0.0f;
  const float observedPeakHz = periodMetric.samples && periodMetric.minUs
      ? 1000000.0f / periodMetric.minUs : 0.0f;
  const uint32_t jitterPp = periodMetric.samples
      ? periodMetric.maxUs - periodMetric.minUs : 0;

  sppLogPrintf(
      "[PERF] adc_us=%.0f/%lu pid_us=%.0f/%lu comm_us=%.0f/%lu "
      "tx_us=%.0f/%lu busy_us=%.0f/%lu other_us=%.0f n=%lu\n",
      avgAdc, (unsigned long)adc.maxUs,
      mean(pid), (unsigned long)pid.maxUs,
      mean(com), (unsigned long)com.maxUs,
      mean(tx), (unsigned long)tx.maxUs,
      avgBody, (unsigned long)bodyMetric.maxUs,
      otherAvg, (unsigned long)bodyMetric.samples);
  sppLogPrintf(
      "[PERF_LOOP] period_us=%lu/%.0f/%lu jitter_pp_us=%lu "
      "peak_observed_hz=%.1f ceiling_busy_hz=%.1f ceiling_adc_hz=%.1f "
      "pid_samples=%lu tx_samples=%lu\n",
      (unsigned long)periodMetric.minUs, mean(periodMetric),
      (unsigned long)periodMetric.maxUs, (unsigned long)jitterPp,
      observedPeakHz, bodyCeilingHz, adcCeilingHz,
      (unsigned long)pid.samples, (unsigned long)tx.samples);
  for (uint8_t i = 0; i < PERF_STAGE_COUNT; ++i) reset(stages[i]);
  reset(periodMetric);
  reset(bodyMetric);
}
void perfTaskReport() {
#if defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1) && \
    defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1)
  // Manual snapshot: relatively expensive and therefore never in the fast loop.
  constexpr UBaseType_t MAX_TASKS = 32;
  static TaskStatus_t tasks[MAX_TASKS];
  uint32_t totalRunTime = 0;
  const UBaseType_t n = uxTaskGetSystemState(tasks, MAX_TASKS, &totalRunTime);
  if (!n || !totalRunTime) {
    sppLogLine("[TASKS] no runtime counters (or task array too small)");
    return;
  }
  sppLogPrintf("[TASKS] tasks=%u runtime_ticks=%lu since_boot (all cores)\n",
                (unsigned)n, (unsigned long)totalRunTime);
  for (UBaseType_t rank = 0; rank < 8 && rank < n; ++rank) {
    UBaseType_t best = n;
    for (UBaseType_t i = 0; i < n; ++i) {
      if (!tasks[i].ulRunTimeCounter) continue;
      if (best == n || tasks[i].ulRunTimeCounter > tasks[best].ulRunTimeCounter)
        best = i;
    }
    if (best == n) break;
    const float pct = 100.0f * tasks[best].ulRunTimeCounter / totalRunTime;
    sppLogPrintf("[TASKS] %s runtime=%.1f%%\n", tasks[best].pcTaskName, pct);
    tasks[best].ulRunTimeCounter = 0;
  }
#else
  sppLogLine("[TASKS] RTOS runtime stats disabled; per-task CPU share unavailable. "
                 "Use PERF comm/adc/pid/tx wall times.");
#endif
}
