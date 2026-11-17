#include "encoder.h"
#include "pinout.h"
#include <Arduino.h>
#include <driver/gpio.h>

namespace {

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

IRAM_ATTR inline uint8_t readEncoderState(uint8_t pinA, uint8_t pinB) {
    return (static_cast<uint8_t>(gpio_get_level(static_cast<gpio_num_t>(pinA))) << 1)
         | static_cast<uint8_t>(gpio_get_level(static_cast<gpio_num_t>(pinB)));
}

// State encoding is (A << 1) | B. A valid one-bit transition adds or subtracts
// one tick. Two-bit jumps are invalid/noisy transitions and are ignored.
IRAM_ATTR inline int8_t quadratureDelta(uint8_t previous, uint8_t next) {
    const uint8_t changed = previous ^ next;
    const bool channelsEqual = ((previous >> 1) & 1U) == (previous & 1U);

    if (changed == 1U) return channelsEqual ? -1 : 1;
    if (changed == 2U) return channelsEqual ? 1 : -1;
    return 0;
}

void IRAM_ATTR onLeftEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    const uint8_t next = readEncoderState(PIN_ENCODER_L1, PIN_ENCODER_L2);
    leftCount += static_cast<int32_t>(quadratureDelta(leftState, next)) * ENCODER_LEFT_SIGN;
    leftState = next;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onRightEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    const uint8_t next = readEncoderState(PIN_ENCODER_R1, PIN_ENCODER_R2);
    rightCount += static_cast<int32_t>(quadratureDelta(rightState, next)) * ENCODER_RIGHT_SIGN;
    rightState = next;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

} // namespace

void encoder_init(void) {
    pinMode(PIN_ENCODER_L1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_L2, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R1, INPUT_PULLUP);
    pinMode(PIN_ENCODER_R2, INPUT_PULLUP);

    portENTER_CRITICAL(&encoderMux);
    leftCount = 0;
    rightCount = 0;
    leftState = readEncoderState(PIN_ENCODER_L1, PIN_ENCODER_L2);
    rightState = readEncoderState(PIN_ENCODER_R1, PIN_ENCODER_R2);
    portEXIT_CRITICAL(&encoderMux);

    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L1), onLeftEncoderEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_L2), onLeftEncoderEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R1), onRightEncoderEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_R2), onRightEncoderEdge, CHANGE);
}

void encoder_get_counts(int32_t* left, int32_t* right) {
    portENTER_CRITICAL(&encoderMux);
    if (left != nullptr) *left = leftCount;
    if (right != nullptr) *right = rightCount;
    portEXIT_CRITICAL(&encoderMux);
}

void encoder_reset_counts(void) {
    portENTER_CRITICAL(&encoderMux);
    leftCount = 0;
    rightCount = 0;
    leftState = readEncoderState(PIN_ENCODER_L1, PIN_ENCODER_L2);
    rightState = readEncoderState(PIN_ENCODER_R1, PIN_ENCODER_R2);
    portEXIT_CRITICAL(&encoderMux);
}
