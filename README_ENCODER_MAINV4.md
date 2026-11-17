# Encoder x4 integration (mainv2 to mainv4)

## Scope
The signed quadrature x4 decoder is copied unchanged from `mainv2` (`include/encoder.h` and `src/encoder.cpp`). `mainv4` keeps its current line-following PID, 16-channel ADC, Bluetooth Classic SPP framing and 100 Hz target telemetry (STREAM-selectable). Encoder interrupts run independently of the PID loop.

## Wiring and sign
| Wheel | Channel A | Channel B | Direction sign |
| --- | --- | --- | --- |
| Left | GPIO21 | GPIO22 | +1 |
| Right | GPIO32 | GPIO33 | -1 |

The signs are inherited from mainv2 and require physical verification. Both values should increase when their respective wheels rotate in the robot-forward direction. If not, reverse the relevant `ENCODER_*_SIGN` in `include/pinout.h`.

Counts now use signed x4 quadrature ticks, so an identical wheel rotation can produce approximately four times the old mainv4 count (which counted only rising edges on A). Avoid assuming a ticks/revolution value without confirming motor and gear specifications.

## Commands and telemetry
- `ENC` or `ENCODER`: read signed left/right accumulated counts.
- `ENC_RESET` or `RESET_ENCODER`: zero both counters and resynchronize A/B states. Prefer resetting with the motors stopped.
- USB `[DIAG]` messages (1 Hz) include `encL` and `encR`.
- Windows `robot_debug.py` shows both signed tick counts and estimated speeds in ticks/s, includes **Reset ENC**, and retains `encL`/`encR` in the CSV.
- The SPP binary frame stays exactly 72 bytes: AA 55 + 69-byte `<I16H2i2h5fB` payload + one XOR checksum. Data fields are `t_ms, adc[16], encL, encR, pwmL, pwmR, error, P, I, D, output, mode`.

The encoder values are sampled for output whether Auto PID is enabled or not. **They are not fed back to the PID controller**: PID still operates on the line sensor error.

## Hardware acceptance test
1. Build `pio run -e esp32dev`. Flash the ESP32 with the robot supported so the wheels are lifted.
2. Open USB Serial at 115200, send `ENC_RESET`, then `ENC`; confirm two zeros.
3. Manually rotate the left wheel forward and backward. `encL` should increase and decrease, while `encR` stays stable. Repeat for the right wheel.
4. Pair `LineRobot` via Bluetooth Classic on Windows and connect its outgoing SPP COM port in `robot_debug.py`. Confirm count and speed updates and telemetry up to 100 Hz (verify measured tx Hz).
5. After line-sensor calibration, run Auto PID with the robot safely supported. Confirm encoder counts continue updating in the App alongside PWM and PID error; log a CSV.
6. Stop, verify count stability, and reset before the next run.

## Limitations
- Actual encoder direction, noise and maximum interrupt rate require real hardware testing.
- Encoder tick/s is not RPM or m/s unless ticks/revolution and wheel dimensions are known.
- `mainv4` selects Bluetooth Classic in `platformio.ini` and excludes the BLE implementation; the Web BLE UI cannot receive its telemetry without a separate BLE configuration.
