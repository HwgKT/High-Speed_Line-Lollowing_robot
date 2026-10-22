#include <Arduino.h>
#include <NimBLEDevice.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string>

namespace {

constexpr char kBleName[] = "ESP32_Robot";
constexpr char kUartServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kUartRxUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kUartTxUuid[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

constexpr uint8_t kSensorCount = 16;
constexpr uint8_t kMuxSignalPin = 36;
constexpr uint8_t kMuxAddressPins[] = {17, 18, 19, 23};
constexpr uint8_t kMotorPins[] = {26, 25, 27, 13};
constexpr uint8_t kEncoderLeftA = 21;
constexpr uint8_t kEncoderLeftB = 22;
constexpr uint8_t kEncoderRightA = 32;
constexpr uint8_t kEncoderRightB = 33;
constexpr uint32_t kTelemetryIntervalMs = 250;
constexpr uint32_t kCalibrationSampleIntervalMs = 20;
constexpr uint32_t kCalibrationDurationMs = 1000;
constexpr uint32_t kMovingCalibrationDurationMs = 6000;
constexpr uint32_t kPidIntervalMs = 10;
constexpr float kDefaultKp = 18.0f;
constexpr float kDefaultKi = 0.0f;
constexpr float kDefaultKd = 0.25f;
constexpr int kDefaultSpeed = 150;
constexpr int kDefaultPwmLimit = 255;

struct BleCommand {
    char text[64];
};

QueueHandle_t commandQueue = nullptr;
NimBLECharacteristic* txCharacteristic = nullptr;
volatile bool bleConnected = false;
volatile bool disconnectPending = false;

uint16_t sensorRaw[kSensorCount] = {};
uint16_t whiteCalibration[kSensorCount] = {};
uint16_t blackCalibration[kSensorCount] = {};
uint16_t movingMinimum[kSensorCount] = {};
uint16_t movingMaximum[kSensorCount] = {};
bool hasWhiteCalibration = false;
bool hasBlackCalibration = false;
bool hasCalibration = false;
String calibrationState = "PENDING";

enum class CalibrationMode : uint8_t { None, White, Black, Moving };
CalibrationMode calibrationMode = CalibrationMode::None;
uint32_t calibrationStartedAt = 0;
uint32_t lastCalibrationSampleAt = 0;
uint16_t calibrationSampleCount = 0;
uint32_t calibrationSums[kSensorCount] = {};

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t leftEncoderCount = 0;
volatile int32_t rightEncoderCount = 0;

bool forwardHeld = false;
bool backwardHeld = false;
bool leftHeld = false;
bool rightHeld = false;
float pidKp = kDefaultKp;
float pidKi = kDefaultKi;
float pidKd = kDefaultKd;
int baseSpeed = kDefaultSpeed;
int pwmLimit = kDefaultPwmLimit;
bool lineFollowing = false;
float integralError = 0.0f;
float previousError = 0.0f;
uint32_t lastPidUpdateAt = 0;

void IRAM_ATTR onLeftEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    leftEncoderCount += digitalRead(kEncoderLeftA) == digitalRead(kEncoderLeftB) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void IRAM_ATTR onRightEncoderEdge() {
    portENTER_CRITICAL_ISR(&encoderMux);
    rightEncoderCount += digitalRead(kEncoderRightA) == digitalRead(kEncoderRightB) ? 1 : -1;
    portEXIT_CRITICAL_ISR(&encoderMux);
}

void stopMotors() {
    for (uint8_t pin : kMotorPins) {
        analogWrite(pin, 0);
        digitalWrite(pin, LOW);
    }
}

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

void applyManualMotion() {
    const int left = (forwardHeld ? 1 : 0) - (backwardHeld ? 1 : 0)
        - (leftHeld ? 1 : 0) + (rightHeld ? 1 : 0);
    const int right = (forwardHeld ? 1 : 0) - (backwardHeld ? 1 : 0)
        + (leftHeld ? 1 : 0) - (rightHeld ? 1 : 0);
    setMotor(kMotorPins[0], kMotorPins[1], constrain(left * baseSpeed, -pwmLimit, pwmLimit));
    setMotor(kMotorPins[2], kMotorPins[3], constrain(right * baseSpeed, -pwmLimit, pwmLimit));
}

void sendBleLine(const String& line);

void clearManualMotion() {
    forwardHeld = false;
    backwardHeld = false;
    leftHeld = false;
    rightHeld = false;
    stopMotors();
}

void resetPid() {
    integralError = 0.0f;
    previousError = 0.0f;
    lastPidUpdateAt = 0;
}

void sendPidSettings() {
    sendBleLine("P:" + String(pidKp, 2));
    sendBleLine("I:" + String(pidKi, 2));
    sendBleLine("D:" + String(pidKd, 2));
    sendBleLine("V:" + String(baseSpeed));
    sendBleLine("W:" + String(pwmLimit));
    sendBleLine(String("FOLLOW:") + (lineFollowing ? "1" : "0"));
}

void readSensors() {
    for (uint8_t index = 0; index < kSensorCount; ++index) {
        for (uint8_t bit = 0; bit < 4; ++bit) {
            digitalWrite(kMuxAddressPins[bit], (index >> bit) & 0x01);
        }
        delayMicroseconds(5);
        sensorRaw[index] = analogRead(kMuxSignalPin);
    }
}

bool sensorSeesLine(uint8_t index) {
    if (!hasCalibration) {
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
        if (!hasCalibration) {
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

void updateLineFollower() {
    if (!lineFollowing || calibrationMode != CalibrationMode::None) {
        return;
    }
    const uint32_t now = millis();
    if (now - lastPidUpdateAt < kPidIntervalMs) {
        return;
    }
    const float deltaTime = lastPidUpdateAt == 0
        ? kPidIntervalMs / 1000.0f
        : (now - lastPidUpdateAt) / 1000.0f;
    lastPidUpdateAt = now;

    int32_t position = 0;
    if (!calculateLinePosition(position)) {
        resetPid();
        stopMotors();
        return;
    }

    const float error = (position - 7500) / 1000.0f;
    integralError = constrain(integralError + error * deltaTime, -100.0f, 100.0f);
    const float derivative = (error - previousError) / deltaTime;
    previousError = error;
    const float correction = pidKp * error + pidKi * integralError + pidKd * derivative;
    const int leftSpeed = constrain(
        static_cast<int>(std::lround(baseSpeed + correction)), -pwmLimit, pwmLimit);
    const int rightSpeed = constrain(
        static_cast<int>(std::lround(baseSpeed - correction)), -pwmLimit, pwmLimit);
    setMotor(kMotorPins[0], kMotorPins[1], leftSpeed);
    setMotor(kMotorPins[2], kMotorPins[3], rightSpeed);
}

void sendBleLine(const String& line) {
    if (txCharacteristic != nullptr && bleConnected) {
        txCharacteristic->setValue((line + '\n').c_str());
        txCharacteristic->notify();
        delay(1);
    }
}

void sendTelemetry() {
    String digitalLine = "D:";

    for (uint8_t index = 0; index < kSensorCount; ++index) {
        const bool onLine = sensorSeesLine(index);
        digitalLine += onLine ? '1' : '0';
    }

    int32_t position = 7500;
    calculateLinePosition(position);

    for (uint8_t start = 0; start < kSensorCount; start += 3) {
        String analogLine = "A" + String(start) + ':';
        for (uint8_t offset = 0; offset < 3 && start + offset < kSensorCount; ++offset) {
            if (offset > 0) {
                analogLine += ',';
            }
            analogLine += String(sensorRaw[start + offset]);
        }
        sendBleLine(analogLine);
    }

    int32_t leftCount;
    int32_t rightCount;
    portENTER_CRITICAL(&encoderMux);
    leftCount = leftEncoderCount;
    rightCount = rightEncoderCount;
    portEXIT_CRITICAL(&encoderMux);

    sendBleLine(digitalLine);
    sendBleLine("POS:" + String(position));
    sendBleLine("ERR:" + String(position - 7500));
    sendBleLine("ENC_L:" + String(leftCount));
    sendBleLine("ENC_R:" + String(rightCount));
    sendBleLine("CAL:" + calibrationState);
}

void beginCalibration(CalibrationMode mode) {
    lineFollowing = false;
    resetPid();
    clearManualMotion();
    sendBleLine("FOLLOW:0");
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
        calibrationState = "WHITE_BEGIN";
    } else if (mode == CalibrationMode::Black) {
        calibrationState = "BLACK_BEGIN";
    } else {
        calibrationState = "CALIBRATING";
    }
    sendBleLine("CAL:" + calibrationState);
}

void updateCalibration() {
    if (calibrationMode == CalibrationMode::None) {
        return;
    }

    const uint32_t now = millis();
    if (now - lastCalibrationSampleAt < kCalibrationSampleIntervalMs) {
        return;
    }
    lastCalibrationSampleAt = now;
    readSensors();

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
            hasCalibration = true;
            calibrationMode = CalibrationMode::None;
            calibrationState = "MOVE_DONE";
            sendBleLine("CAL:" + calibrationState);
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
        calibrationState = "WHITE_DONE";
    } else {
        hasBlackCalibration = true;
        calibrationState = "BLACK_DONE";
    }
    hasCalibration = hasWhiteCalibration && hasBlackCalibration;
    calibrationMode = CalibrationMode::None;
    if (hasCalibration) {
        calibrationState = "READY";
    }
    sendBleLine("CAL:" + calibrationState);
}

void setMotorButton(char buttonCode, bool pressed) {
    if (pressed && lineFollowing) {
        lineFollowing = false;
        resetPid();
        sendBleLine("FOLLOW:0");
    }
    switch (buttonCode) {
        case '5': forwardHeld = pressed; break;
        case '6': backwardHeld = pressed; break;
        case '7': leftHeld = pressed; break;
        case '8': rightHeld = pressed; break;
        default: return;
    }
    applyManualMotion();
}

bool parseFloatSetting(const String& command, float minimum, float maximum, float& value) {
    const char* input = command.c_str() + 2;
    char* end = nullptr;
    const float parsed = strtof(input, &end);
    if (end == input || *end != '\0' || !std::isfinite(parsed)
        || parsed < minimum || parsed > maximum) {
        return false;
    }
    value = parsed;
    return true;
}

void handleCommand(const String& command) {
    if (command.startsWith("!B") && command.length() >= 4) {
        setMotorButton(command[2], command[3] == '1');
    } else if (command == "STOP") {
        lineFollowing = false;
        resetPid();
        clearManualMotion();
        sendBleLine("FOLLOW:0");
    } else if (command.startsWith("P:") || command.startsWith("I:")
        || command.startsWith("D:")) {
        float value = 0.0f;
        const char key = command[0];
        const float minimum = 0.0f;
        const float maximum = key == 'I' ? 10.0f : 100.0f;
        if (!parseFloatSetting(command, minimum, maximum, value)) {
            sendBleLine("LOG:Gia tri PID khong hop le");
            return;
        }
        if (key == 'P') pidKp = value;
        else if (key == 'I') pidKi = value;
        else pidKd = value;
        sendBleLine("ACK:" + command);
    } else if (command.startsWith("V:") || command.startsWith("W:")) {
        const int value = command.substring(2).toInt();
        if (command.length() <= 2 || value < 0 || value > 255
            || String(value) != command.substring(2)) {
            sendBleLine("LOG:Toc do hoac PWM phai tu 0 den 255");
            return;
        }
        if (command[0] == 'V') baseSpeed = value;
        else pwmLimit = value;
        sendBleLine("ACK:" + command);
    } else if (command == "FOLLOW:1") {
        if (!hasCalibration) {
            sendBleLine("LOG:Can calib cam bien truoc khi chay bam line");
            return;
        }
        clearManualMotion();
        resetPid();
        lineFollowing = true;
        sendBleLine("FOLLOW:1");
    } else if (command == "FOLLOW:0") {
        lineFollowing = false;
        resetPid();
        clearManualMotion();
        sendBleLine("FOLLOW:0");
    } else if (command == "CAL_WHITE") {
        beginCalibration(CalibrationMode::White);
    } else if (command == "CAL_BLACK") {
        beginCalibration(CalibrationMode::Black);
    } else if (command == "CAL_MOVE") {
        beginCalibration(CalibrationMode::Moving);
    } else if (command == "ENC_RESET") {
        portENTER_CRITICAL(&encoderMux);
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        portEXIT_CRITICAL(&encoderMux);
        sendBleLine("ACK:ENC_RESET");
    } else if (command == "SYNC") {
        readSensors();
        sendTelemetry();
        sendPidSettings();
        sendBleLine("STATE:READY");
    } else {
        sendBleLine("LOG:Unknown command " + command);
    }
}

class RxCallbacks final : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic) override {
        const std::string value = characteristic->getValue();
        BleCommand command = {};
        size_t length = 0;
        for (char character : value) {
            if (character == '\n' || character == '\r') {
                if (length > 0) {
                    command.text[length] = '\0';
                    if (commandQueue != nullptr) {
                        xQueueSend(commandQueue, &command, 0);
                    }
                    command = {};
                    length = 0;
                }
            } else if (length < sizeof(command.text) - 1) {
                command.text[length++] = character;
            }
        }
        if (length > 0) {
            command.text[length] = '\0';
            if (commandQueue != nullptr) {
                xQueueSend(commandQueue, &command, 0);
            }
        }
    }
};

class ServerCallbacks final : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*) override {
        bleConnected = true;
        disconnectPending = false;
    }

    void onDisconnect(NimBLEServer*) override {
        bleConnected = false;
        disconnectPending = true;
        NimBLEDevice::startAdvertising();
    }
};

void beginBle() {
    commandQueue = xQueueCreate(8, sizeof(BleCommand));
    NimBLEDevice::init(kBleName);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer* server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    NimBLEService* service = server->createService(kUartServiceUuid);
    txCharacteristic = service->createCharacteristic(kUartTxUuid, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic* rxCharacteristic = service->createCharacteristic(
        kUartRxUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rxCharacteristic->setCallbacks(new RxCallbacks());
    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(kUartServiceUuid);
    advertising->setScanResponse(true);
    NimBLEDevice::startAdvertising();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    for (uint8_t pin : kMuxAddressPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(kMuxSignalPin, INPUT);
    for (uint8_t pin : kMotorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
    pinMode(kEncoderLeftA, INPUT);
    pinMode(kEncoderLeftB, INPUT);
    pinMode(kEncoderRightA, INPUT);
    pinMode(kEncoderRightB, INPUT);
    attachInterrupt(digitalPinToInterrupt(kEncoderLeftA), onLeftEncoderEdge, CHANGE);
    attachInterrupt(digitalPinToInterrupt(kEncoderRightA), onRightEncoderEdge, CHANGE);

    stopMotors();
    readSensors();
    beginBle();
}

void loop() {
    if (disconnectPending) {
        disconnectPending = false;
        lineFollowing = false;
        resetPid();
        clearManualMotion();
    }

    BleCommand command = {};
    while (commandQueue != nullptr && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
        handleCommand(String(command.text));
    }

    updateCalibration();
    if (calibrationMode == CalibrationMode::None) {
        readSensors();
    }
    updateLineFollower();

    static uint32_t lastTelemetryAt = 0;
    const uint32_t now = millis();
    if (now - lastTelemetryAt >= kTelemetryIntervalMs) {
        lastTelemetryAt = now;
        sendTelemetry();
    }
    delay(1);
}
