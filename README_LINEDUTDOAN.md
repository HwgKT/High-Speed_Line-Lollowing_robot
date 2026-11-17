# MAP-specific 100ms dashed-line bridge (branch: linedutdoan)

> **mainHighSpeed:** This file documents the inherited GAP-bridge baseline.
> The first-straight PWM trapezoid is implemented on this branch; see
> [README_MAINHIGHSPEED.md](README_MAINHIGHSPEED.md) for current commands and test instructions.
>
## Scope
This experiment is based on current mainv10 (2026-10-31, commit e7e31dc). It does NOT add any high-speed 255-PWM profile or change Kp/Ki/Kd, SEARCH PWM, sharp-turn, finish-line stop, ADC, or the 72-byte SPP telemetry frame.

### Firmware MAP modes
The Python app already offers MAP 1, MAP 2, MAP 3 and separate CSV/TXT folders. On this branch, pressing START sends `AUTO:1`, `AUTO:2` or `AUTO:3` to ESP32, so the firmware MAP and logging destination match atomically.

| MAP | Relative encoder window (ticks) | GAP timeout |
| --- | ---: | ---: |
| 1 | 1500–2100 | 100 ms |
| 2 | 1050–1750 | 100 ms |
| 3 | 1000–1700 | 100 ms |

Relative encoder = ((left count - left count at AUTO) + (right count - right count at AUTO)) / 2. No global encoder reset. These are provisional values inferred from 2026-10-31 logs, not independently calibrated physical distances.

Manual SPP commands: `MAP` to read selection and window, `MAP:1` (or 2/3) while stopped to choose, followed by `AUTO`, or `AUTO:1`/`AUTO:2`/`AUTO:3` to select and start in one step. `AUTO` without any MAP selection is rejected by the firmware. `STATUS` and `DIAG` include MAP and relative encoder progress; `DIAG case=11` indicates an active GAP bridge.

### Motor/line state
- Only after actual line lock and only while `turnState==TURN_NONE`, an all-white reading inside that MAP's encoder window begins GAP_BRIDGE.
- Hold **the last NORMAL_PID governed forward PWM** equally on both wheels (no stored steering correction), for at most 100 ms. The timeout begins with the first white sample, not after `LINE_LOSS_MS` (30 ms).
- When line returns, resume existing PID immediately, suppressing stale D derivative for four frames.
- If still all white at 100 ms, enter the existing SEARCH_LEFT / SEARCH_RIGHT pivot at +/-85 (not the five-second startup grace). A timed-out GAP cannot restart while SEARCH is active.
- Window membership is checked on GAP entry only. Crossing the window boundary during the 100 ms does not prematurely abort this single GAP.
- One-shot SPP text events `GAP_START`, `GAP_RECOVERED`, `GAP_TIMEOUT_SEARCH` include map, runEnc, PWM L/R, and timestamp; the existing Python app captures these in the MAP-specific TXT. The binary telemetry format is unchanged.
- Outside the window, preserve the original 30ms DASHED_LINE (max forward 95), startup-coast and SEARCH behaviors.
- Full-black three-sample finish, STOP and ESTOP continue to stop motor output. No change to log packet structure.

### Historical first-straight speed windows — proposal ONLY
Fifteen existing map logs show the first major line deviation at roughly 2532–2578 relative ticks and first pivot at 2596–2642; track-specific dash positions differ. Do NOT enable fixed PWM 255 solely on these observations.

For a future separately tested acceleration experiment:
- MAP 1: candidate high-speed *test* interval 300–1900; reduce before 2350.
- MAP 2: candidate high-speed *test* interval 300–1650; reduce before 2350.
- MAP 3: candidate high-speed *test* interval 300–1600; reduce before 2350.
Tune brake onset using real encoder speed and measured deceleration, not a fixed assumed distance. Preserve a safety margin before ~2530 ticks. Testing at PWM 180, then 220, then 255 is recommended.

### Verification
Native regression added: `tests/native/gap_bridge_smoke.cpp`. Compile as:
```sh
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke
./gap_bridge_smoke
```
ESP32 build: `pio run -e esp32dev`. Real-hardware validation still required. Verify 16-channel logic, actual encoder sign, Bluetooth MAP response, 100 ms recovery and STOP/E-STOP with wheels lifted before full-speed testing.
