#pragma once
#include <Arduino.h>
#include <stdint.h>
// Original ESP32 Bluetooth Classic Serial Port Profile.
// Packet = 0xAA 0x55 + packed 69-byte payload + XOR(payload).
// Python struct: "<I16H2i2h5fB" (72-byte complete frame).
#pragma pack(push, 1)
struct Telemetry {
  uint32_t t_ms;
  uint16_t adc[16];
  int32_t encL, encR;
  int16_t pwmL, pwmR;
  float err, p, i, d, out;
  uint8_t mode; // 0: manual, 1: auto PID
};
#pragma pack(pop)
static_assert(sizeof(Telemetry)==69, "SPP frame payload must remain 69 bytes");
typedef void (*BtCommandHandler)(const String &command);
bool btBegin(const char *deviceName = "LineRobot");
bool btConnected();
bool btSendTelemetry(const Telemetry &t); // nonblocking queue, returns enqueue result
void btSendLine(const String &line);
void btPollCommands(BtCommandHandler handler);
struct BtTxStats { uint32_t queued, sent, dropped, failed, pending; };
BtTxStats btGetTxStats();
void btDropPendingTelemetry();
