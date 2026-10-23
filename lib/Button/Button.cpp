#include "Button.h"

#include <Arduino.h>

#include "Motor.h"

namespace Button {
namespace {

bool forwardHeld = false;
bool backwardHeld = false;
bool leftHeld = false;
bool rightHeld = false;

void applyManualMotion(int baseSpeed, int pwmLimit) {
    const int left = (forwardHeld ? 1 : 0) - (backwardHeld ? 1 : 0)
        - (leftHeld ? 1 : 0) + (rightHeld ? 1 : 0);
    const int right = (forwardHeld ? 1 : 0) - (backwardHeld ? 1 : 0)
        + (leftHeld ? 1 : 0) - (rightHeld ? 1 : 0);
    Motor::setSpeeds(
        constrain(left * baseSpeed, -pwmLimit, pwmLimit),
        constrain(right * baseSpeed, -pwmLimit, pwmLimit));
}

}  // namespace

bool handle(char buttonCode, bool pressed, int baseSpeed, int pwmLimit) {
    switch (buttonCode) {
        case '5': forwardHeld = pressed; break;
        case '6': backwardHeld = pressed; break;
        case '7': leftHeld = pressed; break;
        case '8': rightHeld = pressed; break;
        default: return false;
    }
    applyManualMotion(baseSpeed, pwmLimit);
    return true;
}

void clear() {
    forwardHeld = false;
    backwardHeld = false;
    leftHeld = false;
    rightHeld = false;
    Motor::stop();
}

}  // namespace Button