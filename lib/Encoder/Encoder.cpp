#include "Encoder.h"

#include <Arduino.h>

namespace Encoder {
namespace {

constexpr uint8_t kLeftA = 21;
constexpr uint8_t kLeftB = 22;
constexpr uint8_t kRightA = 32;
constexpr uint8_t kRightB = 33;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;

void IRAM_ATTR onLeftEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    leftEncoderCount += digitalRead(kLeftA) == digitalRead(kLeftB) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onRightEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    rightEncoderCount += digitalRead(kRightA) == digitalRead(kRightB) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

}  // namespace

void begin() {
    pinMode(kLeftA, INPUT);
    pinMode(kLeftB, INPUT);
    pinMode(kRightA, INPUT);
    pinMode(kRightB, INPUT);
    attachInterrupt(digitalPinToInterrupt(kLeftA), onLeftEncoderEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kRightA), onRightEncoderEdge, CHANGE);
}

void counts(int32_t& left, int32_t& right) {
    portENTER_CRITICAL(&encoderMux);
    left = leftEncoderCount;
    right = rightEncoderCount;
    portEXIT_CRITICAL(&encoderMux);
}

void reset() {
    portENTER_CRITICAL(&encoderMux);
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    portEXIT_CRITICAL(&encoderMux);
}

}  // namespace Encoder