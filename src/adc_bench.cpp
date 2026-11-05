#include "adc_bench.h"
#include "spp_debug.h"
#include "pinout.h"
#include "sensor.h"
#include <Arduino.h>
#include <driver/adc.h>
#include <soc/gpio_struct.h>
#include <math.h>
#include <string.h>

namespace {
constexpr uint8_t kModes = 9;
constexpr uint8_t kScansPerMode = 64;
constexpr uint8_t kSamples[3] = {4, 2, 1};
constexpr uint32_t kMuxMask = (1UL << PIN_MUX_S0) |
                              (1UL << PIN_MUX_S1) |
                              (1UL << PIN_MUX_S2) |
                              (1UL << PIN_MUX_S3);
static_assert(PIN_LINE_SIG == 36, "ADC benchmark assumes GPIO36 = ADC1_CHANNEL_0");
static_assert(PIN_MUX_S0 < 32 && PIN_MUX_S1 < 32 &&
              PIN_MUX_S2 < 32 && PIN_MUX_S3 < 32,
              "Direct GPIO mode requires output pins below 32");

struct Stat {
  uint64_t scanSum = 0, selectUs = 0, settleUs = 0;
  uint64_t discardUs = 0, readUs = 0, sampleDelayUs = 0;
  uint64_t discardDifference = 0;
  uint64_t sum[N_SENSORS] = {}, squares[N_SENSORS] = {};
  uint64_t absToRef = 0;
  uint32_t scanMax = 0, absMax = 0;
  uint32_t mismatch = 0, nearThreshold = 0, invalid = 0;
};

Stat stats[kModes];
uint16_t reference[N_SENSORS] = {};
bool running = false;
bool printing = false;
uint8_t modeIndex = 0, passIndex = 0, reportIndex = 0;

inline void directSelect(uint8_t channel) {
  const uint32_t set = ((channel & 1U) ? (1UL << PIN_MUX_S0) : 0) |
                       ((channel & 2U) ? (1UL << PIN_MUX_S1) : 0) |
                       ((channel & 4U) ? (1UL << PIN_MUX_S2) : 0) |
                       ((channel & 8U) ? (1UL << PIN_MUX_S3) : 0);
  // Clear all, then set selected bits. A 6-us settling delay follows.
  GPIO.out_w1tc = kMuxMask;
  GPIO.out_w1ts = set;
}

inline uint16_t readOnce(uint8_t backend, Stat &s) {
  if (backend == 0) return analogRead(PIN_LINE_SIG);
  const int raw = adc1_get_raw(ADC1_CHANNEL_0);
  if (raw < 0 || raw > 4095) {
    ++s.invalid;
    return 0;
  }
  return (uint16_t)raw;
}

void scan(uint8_t mode, uint16_t *values) {
  Stat &s = stats[mode];
  const uint8_t backend = mode / 3;
  const uint8_t n = kSamples[mode % 3];
  const uint32_t started = micros();
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    uint32_t t = micros();
    if (backend == 2) directSelect(ch);
    else sensor_mux_select(ch);
    s.selectUs += (uint32_t)(micros() - t);

    t = micros();
    delayMicroseconds(6); // Preserve the existing MUX settling requirement.
    s.settleUs += (uint32_t)(micros() - t);

    t = micros();
    const uint16_t discard = readOnce(backend, s);
    s.discardUs += (uint32_t)(micros() - t);

    uint32_t total = 0;
    for (uint8_t j = 0; j < n; ++j) {
      t = micros();
      total += readOnce(backend, s);
      s.readUs += (uint32_t)(micros() - t);
      t = micros();
      delayMicroseconds(3); // Identical post-sample wait in all modes.
      s.sampleDelayUs += (uint32_t)(micros() - t);
    }
    const uint16_t value = (uint16_t)(total / n);
    values[ch] = value;
    s.discardDifference += discard > value ? discard - value : value - discard;
    s.sum[ch] += value;
    s.squares[ch] += (uint64_t)value * value;
  }
  const uint32_t timeUs = micros() - started;
  s.scanSum += timeUs;
  if (timeUs > s.scanMax) s.scanMax = timeUs;
}

void compare(uint8_t mode, const uint16_t *values) {
  Stat &s = stats[mode];
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    const uint16_t v = values[ch], ref = reference[ch];
    const uint32_t d = v > ref ? v - ref : ref - v;
    s.absToRef += d;
    if (d > s.absMax) s.absMax = d;
    if (calibrationReady) {
      const uint16_t threshold = thresholds[ch];
      s.mismatch += ((v >= threshold) != (ref >= threshold));
      const uint32_t distance = ref > threshold ? ref - threshold : threshold - ref;
      s.nearThreshold += distance < 100;
    }
  }
}

void printRow(uint8_t mode) {
  const Stat &s = stats[mode];
  const uint8_t backend = mode / 3;
  const char *name = backend == 0 ? "ARDUINO" : backend == 1 ? "IDF" : "IDF_GPIO";
  const uint8_t samples = kSamples[mode % 3];
  const double scans = kScansPerMode;
  const double observations = (double)kScansPerMode * N_SENSORS;
  double noiseSum = 0.0, maxSd = 0.0;
  for (uint8_t ch = 0; ch < N_SENSORS; ++ch) {
    const double mean = (double)s.sum[ch] / scans;
    const double var = (double)s.squares[ch] / scans - mean * mean;
    const double sigma = sqrt(var > 0 ? var : 0);
    noiseSum += sigma;
    if (sigma > maxSd) maxSd = sigma;
  }
  // Total field is measured from scan entry to exit; phase sums do not
  // include timestamp overhead, result accumulation or loop scheduling.
  sppLogPrintf(
    "[ADC_BENCH] method=%s samples=%u scans=%u scan_us=%.0f/%.0f "
    "mux_us=%.1f settle_us=%.1f discard_us=%.1f read_us=%.1f "
    "wait_us=%.1f noise_adc=%.2f/%.2f mae_adc=%.2f maxerr_adc=%lu "
    "mismatch=%lu/%lu near100=%lu discard_delta=%.1f invalid=%lu\n",
    name, (unsigned)samples, (unsigned)kScansPerMode,
    (double)s.scanSum / scans, (double)s.scanMax,
    (double)s.selectUs / scans, (double)s.settleUs / scans,
    (double)s.discardUs / scans, (double)s.readUs / scans,
    (double)s.sampleDelayUs / scans, noiseSum / N_SENSORS, maxSd,
    (double)s.absToRef / observations, (unsigned long)s.absMax,
    (unsigned long)s.mismatch, (unsigned long)(kScansPerMode * N_SENSORS),
    (unsigned long)s.nearThreshold,
    (double)s.discardDifference / observations,
    (unsigned long)s.invalid);
}
}

bool adcBenchStart() {
  if (running || printing || sensor_is_calibrating()) return false;
  // Match the 12-bit, 11-dB Arduino ESP32 ADC setting.
  // This mode uses only ADC1 channel 0 (GPIO36): no ADC2/Wi-Fi contention.
  if (adc1_config_width(ADC_WIDTH_BIT_12) != ESP_OK ||
      adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11) != ESP_OK)
    return false;
  memset(stats, 0, sizeof(stats));
  memset(reference, 0, sizeof(reference));
  modeIndex = 0;
  passIndex = 0;
  reportIndex = 0;
  running = true;
  sppLogLine("[ADC_BENCH] START modes=9 scans_per_mode=64 pins=GPIO36/17/18/19/23");
  sppLogLine("[ADC_BENCH] Fixed scene required: WHITE, BLACK or EDGE; motors OFF");
  sppLogLine("[ADC_BENCH] Mode 0 ARDUINO/4 is the per-pass reference; no NVS or PID changes");
  return true;
}

bool adcBenchActive() { return running || printing; }

void adcBenchCancel() {
  if (!adcBenchActive()) return;
  running = false;
  printing = false;
  sppLogLine("[ADC_BENCH] CANCELLED");
}

void adcBenchStatus() {
  sppLogPrintf("[ADC_BENCH] state=%s pass=%u/%u mode=%u/%u\n",
      running ? "RUNNING" : printing ? "REPORT" : "IDLE",
      (unsigned)passIndex, (unsigned)kScansPerMode,
      (unsigned)modeIndex, (unsigned)kModes);
}

void adcBenchStep() {
  if (running) {
    uint16_t values[N_SENSORS];
    scan(modeIndex, values);
    if (modeIndex == 0) memcpy(reference, values, sizeof(reference));
    compare(modeIndex, values);
    if (++modeIndex == kModes) {
      modeIndex = 0;
      ++passIndex;
      if (passIndex == kScansPerMode) {
        running = false;
        printing = true;
        reportIndex = 0;
        sppLogLine("[ADC_BENCH] RESULT reference=ARDUINO/4; stationary scene");
        sppLogLine("[ADC_BENCH] noise=stddev across scans; mismatch=changed black/white"
                       " classification vs paired ARDUINO/4; near100=ref within 100 ADC of threshold");
        sppLogLine("[ADC_BENCH] discard_delta=discarded first read vs averaged kept reads"
                       " (settling proxy, NOT proof of crosstalk)");
      }
    }
  } else if (printing) {
    // Print just one row per loop to remain responsive to ESTOP.
    printRow(reportIndex);
    if (++reportIndex >= kModes) {
      printing = false;
      sppLogLine("[ADC_BENCH] DONE; production sensor mode=IDF_GPIO_1");
    }
  }
}
