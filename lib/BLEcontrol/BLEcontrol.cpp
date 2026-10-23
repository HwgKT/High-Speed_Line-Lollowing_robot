#include "BLEcontrol.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <cmath>
#include <cstdlib>
#include <string>

#include "Button.h"
#include "Encoder.h"
#include "PID.h"
#include "Sensor.h"

namespace BLEControl {
namespace {

constexpr char kBleName[] = "ESP32_Robot";
constexpr char kUartServiceUuid[] = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kUartRxUuid[] = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
constexpr char kUartTxUuid[] = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
constexpr uint32_t kSerialCommandTimeoutMs = 1500;

struct BleCommand {
    char text[64];
};

QueueHandle_t commandQueue = nullptr;
NimBLECharacteristic* txCharacteristic = nullptr;
volatile bool bleConnected = false;
volatile bool disconnectPending = false;
uint32_t lastSerialCommandAt = 0;
bool serialSessionActive = false;

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

void startSensorCalibration(Sensor::CalibrationMode mode) {
    PID::setFollowing(false);
    PID::reset();
    Button::clear();
    sendLine("FOLLOW:0");
    Sensor::beginCalibration(mode);
    sendLine("CAL:" + Sensor::calibrationState());
}

void handleCommand(const String& command) {
    if (command.startsWith("!B") && command.length() >= 4) {
        const bool pressed = command[3] == '1';
        if (pressed && PID::isFollowing()) {
            PID::setFollowing(false);
            PID::reset();
            sendLine("FOLLOW:0");
        }
        Button::handle(command[2], pressed, PID::baseSpeed(), PID::pwmLimit());
    } else if (command == "STOP") {
        PID::setFollowing(false);
        PID::reset();
        Button::clear();
        sendLine("FOLLOW:0");
    } else if (command.startsWith("P:") || command.startsWith("I:")
        || command.startsWith("D:")) {
        float value = 0.0f;
        const char key = command[0];
        const float maximum = key == 'I' ? 10.0f : 100.0f;
        if (!parseFloatSetting(command, 0.0f, maximum, value)) {
            sendLine("LOG:Gia tri PID khong hop le");
            return;
        }
        if (key == 'P') PID::setKp(value);
        else if (key == 'I') PID::setKi(value);
        else PID::setKd(value);
        sendLine("ACK:" + command);
    } else if (command.startsWith("V:") || command.startsWith("W:")) {
        const int value = command.substring(2).toInt();
        if (command.length() <= 2 || value < 0 || value > 255
            || String(value) != command.substring(2)) {
            sendLine("LOG:Toc do hoac PWM phai tu 0 den 255");
            return;
        }
        if (command[0] == 'V') PID::setBaseSpeed(value);
        else PID::setPwmLimit(value);
        sendLine("ACK:" + command);
    } else if (command == "FOLLOW:1") {
        if (!Sensor::hasCalibration()) {
            sendLine("LOG:Can calib cam bien truoc khi chay bam line");
            return;
        }
        Button::clear();
        PID::reset();
        PID::setFollowing(true);
        sendLine("FOLLOW:1");
    } else if (command == "FOLLOW:0") {
        PID::setFollowing(false);
        PID::reset();
        Button::clear();
        sendLine("FOLLOW:0");
    } else if (command == "CAL_WHITE") {
        startSensorCalibration(Sensor::CalibrationMode::White);
    } else if (command == "CAL_BLACK") {
        startSensorCalibration(Sensor::CalibrationMode::Black);
    } else if (command == "CAL_MOVE") {
        startSensorCalibration(Sensor::CalibrationMode::Moving);
    } else if (command == "ENC_RESET") {
        Encoder::reset();
        sendLine("ACK:ENC_RESET");
    } else if (command == "SYNC") {
        Sensor::read();
        sendTelemetry();
        sendPidSettings();
        sendLine("STATE:READY");
    } else if (command == "HEARTBEAT") {
        return;
    } else {
        sendLine("LOG:Unknown command " + command);
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

}  // namespace

void begin() {
    commandQueue = xQueueCreate(8, sizeof(BleCommand));
    if (commandQueue == nullptr) {
        Serial.println("BLE command queue creation failed");
        return;
    }

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

bool takeDisconnectEvent() {
    if (!disconnectPending) {
        return false;
    }
    disconnectPending = false;
    return true;
}

void processCommands() {
    BleCommand command = {};
    while (commandQueue != nullptr && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
        handleCommand(String(command.text));
    }

    static char serialCommand[sizeof(BleCommand::text)] = {};
    static size_t serialLength = 0;
    static bool discardSerialCommand = false;
    while (Serial.available() > 0) {
        const int incoming = Serial.read();
        if (incoming < 0) {
            break;
        }
        const char character = static_cast<char>(incoming);
        if (character == '\n' || character == '\r') {
            if (!discardSerialCommand && serialLength > 0) {
                serialCommand[serialLength] = '\0';
                lastSerialCommandAt = millis();
                serialSessionActive = true;
                handleCommand(String(serialCommand));
            }
            serialLength = 0;
            discardSerialCommand = false;
        } else if (!discardSerialCommand) {
            if (serialLength < sizeof(serialCommand) - 1) {
                serialCommand[serialLength++] = character;
            } else {
                serialLength = 0;
                discardSerialCommand = true;
                sendLine("LOG:Serial command too long");
            }
        }

        if (serialSessionActive && millis() - lastSerialCommandAt >= kSerialCommandTimeoutMs) {
            serialSessionActive = false;
            PID::setFollowing(false);
            PID::reset();
            Button::clear();
            sendLine("STATE:SERIAL_TIMEOUT");
        }
    }
}

void sendLine(const String& line) {
    if (Serial.availableForWrite() >= static_cast<int>(line.length() + 2)) {
        Serial.println(line);
    }
    if (txCharacteristic != nullptr && bleConnected) {
        txCharacteristic->setValue((line + '\n').c_str());
        txCharacteristic->notify();
        delay(1);
    }
}

void sendPidSettings() {
    sendLine("P:" + String(PID::kp(), 2));
    sendLine("I:" + String(PID::ki(), 2));
    sendLine("D:" + String(PID::kd(), 2));
    sendLine("V:" + String(PID::baseSpeed()));
    sendLine("W:" + String(PID::pwmLimit()));
    sendLine(String("FOLLOW:") + (PID::isFollowing() ? "1" : "0"));
}

void sendTelemetry() {
    String digitalLine = "D:";
    for (uint8_t index = 0; index < Sensor::kSensorCount; ++index) {
        digitalLine += Sensor::seesLine(index) ? '1' : '0';
    }

    int32_t position = 7500;
    Sensor::calculateLinePosition(position);

    for (uint8_t start = 0; start < Sensor::kSensorCount; start += 3) {
        String analogLine = "A" + String(start) + ':';
        for (uint8_t offset = 0; offset < 3 && start + offset < Sensor::kSensorCount; ++offset) {
            if (offset > 0) {
                analogLine += ',';
            }
            analogLine += String(Sensor::raw(start + offset));
        }
        sendLine(analogLine);
    }

    int32_t leftCount = 0;
    int32_t rightCount = 0;
    Encoder::counts(leftCount, rightCount);

    sendLine(digitalLine);
    sendLine("POS:" + String(position));
    sendLine("ERR:" + String(position - 7500));
    sendLine("ENC_L:" + String(leftCount));
    sendLine("ENC_R:" + String(rightCount));
    sendLine("CAL:" + Sensor::calibrationState());
}

}  // namespace BLEControl
