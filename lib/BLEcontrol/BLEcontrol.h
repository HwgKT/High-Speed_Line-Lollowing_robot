#ifndef BLECONTROL_H
#define BLECONTROL_H

#include <Arduino.h>

namespace BLEControl {

void begin();
bool takeDisconnectEvent();
void processCommands();
void sendLine(const String& line);
void sendPidSettings();
void sendTelemetry();

}  // namespace BLEControl

#endif