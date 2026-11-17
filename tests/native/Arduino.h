#pragma once
#include <stdint.h>
// Native host clock stub to compile the real PID controller without ESP32.
uint32_t millis(void);
uint32_t micros(void);
