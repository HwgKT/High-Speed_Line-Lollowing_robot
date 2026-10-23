#include "Motor.h"

#include <Arduino.h>

namespace Motor {
namespace {

constexpr uint8_t kPins[] = {26, 25, 27, 13};

void setMotor(uint8_t forwardPin, uint8_t reversePin, int speed) {
    speed = constrain(speed, -255, 255);
    analogWrite(forwardPin, 0);
    analogWrite(reversePin, 0);
    if (speed > 0) {
        analogWrite(forwardPin, speed);
    } else if (speed < 0) {
        analogWrite(reversePin, -speed);
    }
}

}  // namespace

void begin() {
    for (uint8_t pin : kPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void stop() {
    for (uint8_t pin : kPins) {
        analogWrite(pin, 0);
        digitalWrite(pin, LOW);
    }
}

void setSpeeds(int left, int right) {
    setMotor(kPins[0], kPins[1], left);
    setMotor(kPins[2], kPins[3], right);
}

}  // namespace Motor