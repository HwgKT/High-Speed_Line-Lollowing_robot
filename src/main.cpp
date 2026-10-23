#include <Arduino.h>

#include "BLEcontrol.h"
#include "Button.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"
#include "Sensor.h"

namespace {

constexpr uint32_t kTelemetryIntervalMs = 250;

}  // namespace

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    Sensor::begin();
    Motor::begin();
    Encoder::begin();
    Motor::stop();
    Sensor::read();
    BLEControl::begin();
}

void loop() {
    if (BLEControl::takeDisconnectEvent()) {
        PID::setFollowing(false);
        PID::reset();
        Button::clear();
    }

    BLEControl::processCommands();
    Sensor::updateCalibration();
    if (!Sensor::isCalibrating()) {
        Sensor::read();
    }
    PID::update();

    static uint32_t lastTelemetryAt = 0;
    const uint32_t now = millis();
    if (now - lastTelemetryAt >= kTelemetryIntervalMs) {
        lastTelemetryAt = now;
        BLEControl::sendTelemetry();
    }
    delay(1);
}