#pragma once
#include <Arduino.h>

// Non-blocking 9-mode ADC/MUX benchmark. Never writes sensorRaw or NVS.
// Each call to adcBenchStep() performs <= 1 complete 16-channel scan.
bool adcBenchStart();
bool adcBenchActive();
void adcBenchStep();
void adcBenchCancel();
void adcBenchStatus();
