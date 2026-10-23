#include "Sensor.h"

#include <Arduino.h>

#include <algorithm>

namespace Sensor {
namespace {

constexpr uint8_t kMuxSignalPin = 36;
constexpr uint8_t kMuxAddressPins[] = {17, 18, 19, 23};
constexpr uint32_t kSampleIntervalMs = 20;
constexpr uint32_t kCalibrationDurationMs = 1000;
constexpr uint32_t kMovingCalibrationDurationMs = 6000;

uint16_t sensorRaw[kSensorCount] = {};
uint16_t whiteCalibration[kSensorCount] = {};
uint16_t blackCalibration[kSensorCount] = {};
uint16_t movingMinimum[kSensorCount] = {};
uint16_t movingMaximum[kSensorCount] = {};
bool hasWhiteCalibration = false;
bool hasBlackCalibration = false;
bool calibrated = false;
String currentCalibrationState = "PENDING";
CalibrationMode calibrationMode = CalibrationMode::None;
uint32_t calibrationStartedAt = 0;
uint32_t lastCalibrationSampleAt = 0;
uint16_t calibrationSampleCount = 0;
uint32_t calibrationSums[kSensorCount] = {};

}  // namespace

void begin() {
    for (uint8_t pin : kMuxAddressPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(kMuxSignalPin, INPUT);
}

void read() {
    for (uint8_t index = 0; index < kSensorCount; ++index) {
        for (uint8_t bit = 0; bit < 4; ++bit) {
            digitalWrite(kMuxAddressPins[bit], (index >> bit) & 0x01);
        }
        delayMicroseconds(5);
        sensorRaw[index] = analogRead(kMuxSignalPin);
    }
}

bool seesLine(uint8_t index) {
    if (!calibrated || index >= kSensorCount) {
        return false;
    }
    const uint16_t midpoint = (whiteCalibration[index] + blackCalibration[index]) / 2;
    return blackCalibration[index] < whiteCalibration[index]
        ? sensorRaw[index] < midpoint
        : sensorRaw[index] > midpoint;
}

bool calculateLinePosition(int32_t& position) {
    int32_t weightedTotal = 0;
    uint32_t signalTotal = 0;
    for (uint8_t index = 0; index < kSensorCount; ++index) {
        if (!calibrated) {
            continue;
        }
        const int difference = static_cast<int>(whiteCalibration[index])
            - static_cast<int>(blackCalibration[index]);
        const int range = abs(difference);
        const int response = range < 5 ? 0 : constrain(
            difference > 0
                ? (static_cast<int>(whiteCalibration[index]) - sensorRaw[index]) * 1000 / range
                : (static_cast<int>(sensorRaw[index]) - whiteCalibration[index]) * 1000 / range,
            0, 1000);
        if (response > 50) {
            weightedTotal += response * static_cast<int>(index) * 1000;
            signalTotal += response;
        }
    }
    if (signalTotal == 0) {
        return false;
    }
    position = weightedTotal / static_cast<int32_t>(signalTotal);
    return true;
}

void beginCalibration(CalibrationMode mode) {
    calibrationMode = mode;
    calibrationStartedAt = millis();
    lastCalibrationSampleAt = 0;
    calibrationSampleCount = 0;
    for (uint8_t index = 0; index < kSensorCount; ++index) {
        calibrationSums[index] = 0;
        if (mode == CalibrationMode::Moving) {
            movingMinimum[index] = 4095;
            movingMaximum[index] = 0;
        }
    }

    if (mode == CalibrationMode::White) {
        currentCalibrationState = "WHITE_BEGIN";
    } else if (mode == CalibrationMode::Black) {
        currentCalibrationState = "BLACK_BEGIN";
    } else {
        currentCalibrationState = "CALIBRATING";
    }
}

void updateCalibration() {
    if (calibrationMode == CalibrationMode::None) {
        return;
    }

    const uint32_t now = millis();
    if (now - lastCalibrationSampleAt < kSampleIntervalMs) {
        return;
    }
    lastCalibrationSampleAt = now;
    read();

    if (calibrationMode == CalibrationMode::Moving) {
        for (uint8_t index = 0; index < kSensorCount; ++index) {
            movingMinimum[index] = std::min(movingMinimum[index], sensorRaw[index]);
            movingMaximum[index] = std::max(movingMaximum[index], sensorRaw[index]);
        }
        if (now - calibrationStartedAt >= kMovingCalibrationDurationMs) {
            for (uint8_t index = 0; index < kSensorCount; ++index) {
                blackCalibration[index] = movingMinimum[index];
                whiteCalibration[index] = movingMaximum[index];
            }
            hasBlackCalibration = true;
            hasWhiteCalibration = true;
            calibrated = true;
            calibrationMode = CalibrationMode::None;
            currentCalibrationState = "MOVE_DONE";
        }
        return;
    }

    for (uint8_t index = 0; index < kSensorCount; ++index) {
        calibrationSums[index] += sensorRaw[index];
    }
    ++calibrationSampleCount;
    if (now - calibrationStartedAt < kCalibrationDurationMs) {
        return;
    }

    for (uint8_t index = 0; index < kSensorCount; ++index) {
        const uint16_t average = calibrationSums[index] / calibrationSampleCount;
        if (calibrationMode == CalibrationMode::White) {
            whiteCalibration[index] = average;
        } else {
            blackCalibration[index] = average;
        }
    }

    const bool whiteMode = calibrationMode == CalibrationMode::White;
    if (whiteMode) {
        hasWhiteCalibration = true;
        currentCalibrationState = "WHITE_DONE";
    } else {
        hasBlackCalibration = true;
        currentCalibrationState = "BLACK_DONE";
    }
    calibrated = hasWhiteCalibration && hasBlackCalibration;
    calibrationMode = CalibrationMode::None;
    if (calibrated) {
        currentCalibrationState = "READY";
    }
}

bool hasCalibration() { return calibrated; }
bool isCalibrating() { return calibrationMode != CalibrationMode::None; }
const String& calibrationState() { return currentCalibrationState; }
uint16_t raw(uint8_t index) { return index < kSensorCount ? sensorRaw[index] : 0; }

}  // namespace Sensor