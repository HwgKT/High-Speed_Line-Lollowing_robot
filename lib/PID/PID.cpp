#include "PID.h"

#include <Arduino.h>

#include <cmath>

#include "Motor.h"
#include "Sensor.h"

namespace PID {
namespace {

constexpr uint32_t kUpdateIntervalMs = 10;
float proportionalGain = 18.0f;
float integralGain = 0.0f;
float derivativeGain = 0.25f;
int currentBaseSpeed = 150;
int currentPwmLimit = 255;
bool lineFollowing = false;
float integralError = 0.0f;
float previousError = 0.0f;
uint32_t lastUpdateAt = 0;

}  // namespace

void reset() {
    integralError = 0.0f;
    previousError = 0.0f;
    lastUpdateAt = 0;
}

void update() {
    if (!lineFollowing || Sensor::isCalibrating()) {
        return;
    }
    const uint32_t now = millis();
    if (now - lastUpdateAt < kUpdateIntervalMs) {
        return;
    }
    const float deltaTime = lastUpdateAt == 0
        ? kUpdateIntervalMs / 1000.0f
        : (now - lastUpdateAt) / 1000.0f;
    lastUpdateAt = now;

    int32_t position = 0;
    if (!Sensor::calculateLinePosition(position)) {
        reset();
        Motor::stop();
        return;
    }

    const float error = (position - 7500) / 1000.0f;
    integralError = constrain(integralError + error * deltaTime, -100.0f, 100.0f);
    const float derivative = (error - previousError) / deltaTime;
    previousError = error;
    const float correction = proportionalGain * error
        + integralGain * integralError + derivativeGain * derivative;
    const int leftSpeed = constrain(
        static_cast<int>(std::lround(currentBaseSpeed + correction)),
        -currentPwmLimit, currentPwmLimit);
    const int rightSpeed = constrain(
        static_cast<int>(std::lround(currentBaseSpeed - correction)),
        -currentPwmLimit, currentPwmLimit);
    Motor::setSpeeds(leftSpeed, rightSpeed);
}

void setFollowing(bool enabled) { lineFollowing = enabled; }
bool isFollowing() { return lineFollowing; }
float kp() { return proportionalGain; }
float ki() { return integralGain; }
float kd() { return derivativeGain; }
int baseSpeed() { return currentBaseSpeed; }
int pwmLimit() { return currentPwmLimit; }
void setKp(float value) { proportionalGain = value; }
void setKi(float value) { integralGain = value; }
void setKd(float value) { derivativeGain = value; }
void setBaseSpeed(int value) { currentBaseSpeed = value; }
void setPwmLimit(int value) { currentPwmLimit = value; }

}  // namespace PID