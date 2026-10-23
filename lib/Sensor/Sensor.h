#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

namespace Sensor {

constexpr uint8_t kSensorCount = 16;

enum class CalibrationMode : uint8_t { None, White, Black, Moving };

void begin();
void read();
bool seesLine(uint8_t index);
bool calculateLinePosition(int32_t& position);
void beginCalibration(CalibrationMode mode);
void updateCalibration();
bool hasCalibration();
bool isCalibrating();
const String& calibrationState();
uint16_t raw(uint8_t index);

}  // namespace Sensor

#endif