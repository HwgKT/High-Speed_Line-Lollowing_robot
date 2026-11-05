# Bluetooth Classic SPP (mainv4)
Target: original ESP32 DevKit V1, PlatformIO esp32dev and Bluetooth Classic SPP.

Windows: pair device "LineRobot" and open its outgoing SPP COM port in Python app. USB debug 115200 8N1 and newline-terminated ASCII commands.

LED onboard GPIO2: blinking each 400 ms when disconnected; solid when SPP connected. If inverted, change DEBUG_LED_ACTIVE_HIGH in include/pinout.h.

Bluetooth telemetry: header AA 55 + packed 69-byte payload + 1 XOR checksum = 72 bytes. Python struct <I16H2i2h5fB, targeted every 10 ms (100 Hz by default, configurable via STREAM:0/20/50/100). ADC readings are RAW (0..4095), not calibrated 0..1000. PC-calibrated displays are independent of ESP calibration.

Both USB and SPP commands: PING, HELP, STATUS, PID, PWM, ENC, ENC_RESET, P, AUTO/START, STOP, ESTOP, RESET_ESTOP, M, F, B, L, R, KP:x, KI:x, KD:x, BASE:n, V:n, CAL_WHITE, CAL_BLACK, CAL_STATUS, CAL_DUMP. Old presets were removed. A binary packet is never sent to USB serial; diagnostic text is printed once per second.

IMPORTANT PID D-term: Based on d3529b7, the D formula is still Kd*(e[k]-e[k-1])/measuredDt where measuredDt is in seconds from micros(). Invalid dt and the first derivative sample after mode/case/Kd changes are skipped. Kd defaults to ZERO. All built-in presets force Kd=0. Old Kd=3.4/3.9/10 must NOT be reused without re-tuning. Only debug P/I/D getters were added; D formula is unchanged.

Calibration: firmware sweeps 4 seconds; failure restores the previous valid Min/Max. Encoder counting is imported from mainv2: interrupt-based quadrature x4 on GPIO 21/22 (left) and GPIO 32/33 (right), with signed cumulative ticks. mainv2 sign convention is left +1, right -1; verify the signs on the physical robot. Counts are transmitted at the configured stream rate (100 Hz default) in both manual and auto PID modes, displayed as signed ticks and smoothed tick/s in the Windows App, and saved with CSV. ENC and ENC_RESET commands work over SPP and USB. This does not introduce a new wheel-speed PID. mainv4 still uses Bluetooth Classic SPP, not Web BLE. See README_ENCODER_MAINV4.md. Manual movement expires after 750 ms with no repeat command, for safety. SPP disconnect stops Auto and motors.

Smoke test:
1. python -m platformio run -e esp32dev
2. python -m platformio run -e esp32dev -t upload
3. python -m platformio device monitor -p COM_USB -b 115200
4. Send PING with newline; expect PONG.
5. Pair LineRobot on Windows; LED solid and USB log [BT] CONNECTED.
6. Observe [DIAG], telemetry up to ~100 Hz (actual rate depends on loop timing and SPP link), and PID: Kd=0.00 before running any motor.
7. `loop()` has no `delay(5)` (only `yield()`); USB `[DIAG]`, `[PERF]`, and `[PERF_LOOP]` report loop frequency and stage timing. The SPP scheduler skips missed slots, never bursts stale frames. Start with wheels off the ground. Build and hardware behavior have not yet been verified.

## Runtime profiling
- `[PERF]`: `adc_us`, `pid_us`, `comm_us`, `tx_us`, `busy_us` are **average/maximum microseconds**. `other_us` is the remaining loop work (buttons, log output, safety). During calibration, `adc_us` also includes statistics and occasionally NVS writes.
- `[PERF_LOOP]`: `period_us=min/mean/max` measures start-to-start intervals (including scheduler interruptions, yield, and USB log spikes); `jitter_pp_us=max-min`; `peak_observed_hz=1e6/minPeriod`. `ceiling_busy_hz=1e6/averageBodyTime` and `ceiling_adc_hz=1e6/averageAdcTime` are **optimistic calculated ceilings**, not demonstrated sustained PID frequencies. `pid_samples=0` means AUTO was off.
- USB and SPP command `TASKS` requests FreeRTOS runtime counters **only if compiled into the framework**; otherwise it prints an explicit unavailable message. Stage timings measure elapsed wall time, not per-task CPU percentages.
- Record 10–20 samples in three conditions: Bluetooth disconnected idle; Bluetooth connected `STREAM:100`; and AUTO enabled with wheels raised. Compare mean/maximum times and jitter before changing ADC averaging or PID frequency.


## Line follow and latched E-STOP

The Windows app has **BẮT ĐẦU DÒ LINE** (`AUTO`) and **DỪNG KHẨN CẤP** (`ESTOP`), plus an explicit **Mở khóa E-STOP** (`RESET_ESTOP`). E-STOP stops PWM and AUTO immediately when the command is processed and latches motion inhibition. AUTO and manual F/B/L/R respond `ERR: ESTOP_LATCHED` until reset. `RESET_ESTOP` does not start either motor. `STATUS` includes `estop=0/1`; confirmation is `ESTOP: LATCHED`. The ESP32 still stops on SPP disconnection. This software stop is not a hardware power cut and cannot guarantee emergency protection if the radio link, firmware or driver fails. 72-byte binary telemetry remains unchanged.

## ADC/MUX read benchmarking (safe, non-blocking)
- On USB 115200 terminal (or Bluetooth command terminal), send `ADC_BENCH`. It always stops motor output and AUTO. While it runs, firmware denies AUTO/manual movement/calibration, and still parses ESTOP between complete scans. Send `ADC_BENCH_STOP` to cancel or `ADC_BENCH_STATUS` to see progress. The 72-byte SPP protocol stays unchanged; telemetry is paused only during the benchmark.
- It sweeps **9 configurations**: `ARDUINO` (digitalWrite + analogRead), `IDF` (digitalWrite + adc1_get_raw on ADC1_CH0/GPIO36), and `IDF_GPIO` (direct GPIO output registers + adc1_get_raw), each for `samples=4,2,1`. The first read after each MUX switch is always discarded, 6 us MUX settle and 3 us after each retained sample are preserved.
- It measures 64 full scans of all 16 eyes per configuration, interleaved by pass. `ARDUINO/4` is the paired reference on each pass. **Keep the sensor bar perfectly stationary** in three separate runs on white, black and a black/white edge. Avoid changing illumination or touching the sensor bar during a run.
- Nine `[ADC_BENCH]` lines on **USB Serial** report full scan `scan_us=avg/max`, breakdown `mux_us, settle_us, discard_us, read_us, wait_us` in microseconds per scan, `noise_adc=mean/max per-eye SD` across repeated scans, `mae_adc/maxerr_adc` vs paired reference, `mismatch=count/1024` vs the existing NVS per-eye thresholds, `near100` reference values within 100 raw ADC counts of a threshold, `discard_delta` first discarded ADC vs kept average (an **imperfect proxy**, not proof, of MUX settling), and `invalid` number of ADC1 read failures.
- The benchmark does **not** change production `sensorRaw`, calibration in NVS or production scan algorithm. `ADC_BENCH` does configure ADC1 width to 12 bits and GPIO36 attenuation to 11dB to match Arduino's usual default; compare white/black references to rule out offset changes.
- `mismatch=0` on uniform white/black alone is insufficient to certify the PID; test an edge where some eyes sit close to their thresholds. The most promising mode is one that reduces scan time **and** keeps noise, threshold mismatches and carryover proxy acceptable. Do not select merely the smallest `scan_us`.

## Production 16-channel ADC mode (selected fastest tested configuration)
- Production sensor reads now use **IDF_GPIO/1**: ADC1_CH0 at GPIO36, 12 bits, attenuation 11 dB, direct GPIO writes to select MUX channels on GPIO17/18/19/23, 6-us settle, one discarded sample, one retained `adc1_get_raw()` and 3-us post-sample wait. This matches the fastest of nine bench modes.
- In the stationary hardware benchmark, ARDUINO/4 took ~12,232 us per 16 eyes versus IDF_GPIO/1 ~1,615 us, a 7.57x reduction in measured scan duration. These are **bench scan-only**, not proven sustained PID/BT loop Hz; edge-of-line detection at speed still requires validation.
- The ADC driver initializes explicitly at startup. USB prints `[ADC] mode=IDF_GPIO_1 ready=1 ...`; `STATUS` includes `adc=IDF_GPIO_1 adc_ok=1 adc_err=0`, also shown on the Python app. Initialization or read fault disables auto/manual movement and stops active motors. Reset the ESP32 to recover from a latched ADC fault.
- Previous NVS white/black levels, per-sensor 40%-contrast thresholds, PID algorithms, ESP32 Classic SPP telemetry frame (72 bytes), app CSV and E-STOP are unchanged. Verify calibration levels in the live 16-eye view after reflash; recalibrate if any eye has shifted enough to cross its threshold.
- `ADC_BENCH` is still available unchanged for further comparison and continues using its own ARDUINO/4 baseline. Previously the benchmark printed 'firmware keeps production ADC=ARDUINO/4'; that final message now refers to the old mode and will be updated to the selected production mode.
- Measure USB `[PERF] adc_us` and `[DIAG] loop=...Hz` with BT disconnected, then with SPP at 100 Hz and AUTO (wheels raised). The theoretical ADC scan ceiling (~619 Hz) is not a demonstrated control-loop rate.


## SPP-only 300 Hz telemetry and full-session export (October 2026)
- USB serial initialization, USB command polling and scheduled diagnostic prints are disabled. Use Bluetooth SPP commands through the Python app. Firmware still uploads through the USB connector.
- Telemetry defaults to STREAM:300; supported STREAM:0/20/50/100/200/300. `micros()` scheduler targets 3333 us between frames; each binary frame remains 72 bytes. 300 frames/sec means 21,600 bytes/sec payload+framing (~172.8 kbps) before link overhead. The actual receive rate depends on SPP throughput and may be lower.
- Main loop only enqueues telemetry (12-frame bounded FreeRTOS queue). A Core-0 background task owns `BluetoothSerial.write`, which can block under congestion. `DIAG` reports loop/pid Hz, queued/sent rates and dropped/pending frames; `PERF` reports ADC/PID/COMM/TX timing and jitter on demand, never periodically. Click **Đo TX/Loop** in the app (prefer twice 5 seconds apart).
- Python app defaults to 300 Hz and shows **RX Hz** (actual frame decode in reader thread), **APP Hz** (samples processed in GUI), `q` (pending GUI queue), `dropUI`, and `CRC` errors. GUI screen refresh remains ~33 Hz; it is not the telemetry rate.
- **Xuất CSV** now exports the **entire app session**, not just the last 15,000 frames. **Xuất TXT** exports the entire terminal log since app startup or the last explicit **Xóa**. Both use auto-created temporary disk-backed session files; these are deleted when the app closes. **Ghi CSV** still writes an explicit persistent CSV file continuously for measurements that must survive crashes/app shutdown. Large exports take disk space and time.
- App history deques and the visible terminal remain bounded for UI responsiveness, but no longer limit session export. `ADC_BENCH` and calibration debug text are routed through SPP. No hardware measurement at 300 Hz has yet been reported; test with wheels raised.


### Live loop-frequency display in Python app
- App automatically requests DIAG over Bluetooth SPP every two seconds while connected, and displays actual firmware LOOP, PID and SPP TX (sent-to-stack) rates in a dedicated status strip. Actual received packet rate remains in the RX counter. PID=0 Hz is normal when AUTO is off.
- Manual Do TX/Loop button still requests DIAG and PERF; its DIAG resets the timing window. First measurement after reconnect can include disconnected time; use subsequent stable measurements. Stale indicators reset after six seconds without DIAG. No USB Serial logging is re-enabled.


## Continuous 300 Hz SPP CSV recorder
- The Python app creates a persistent logs/linerobot_300hz_TIMESTAMP.csv automatically on first Bluetooth connection, then continuously appends each decoded CRC-valid 72-byte SPP frame using a dedicated writer thread. Capturing happens in the SPP reader thread, BEFORE delivering frames to the Tkinter GUI queue; UI lag cannot suppress logging.
- A bounded recorder queue holds up to 8192 frames; it never waits on disk writes in the receiver. CSV writer flushes to disk at least once each second and drains the queue on normal shutdown. The app status reports persisted row count, pending queue, dropCSV (queue overflow), and disk errors. Compare that to the ESP32 DIAG and Python RX Hz. At 300 frames/s, expect about 300 CSV rows/s if all frames reach the PC.
- CSV columns remain: receive host_iso, ESP32 t_ms, all adc0..adc15, optional cal0..cal15 and logic0..logic15 (filled after NVS calibration download), encL, encR, pwmL, pwmR, err, p, i, d, out, mode, logic16. Early frames before NVS download leave calibrated/logic fields blank, not the raw or control fields.
- Ghi CSV button now pauses/resumes background recording; recording begins automatically, without clicking. Xuất CSV copies a point-in-time snapshot of the persistent session file in the background. The original file stays in logs/ after app exit. The separate terminal TXT export is unchanged.


### Live CSV Hz + prioritized Bluetooth replies
- CSV status additionally displays the actual disk writer throughput in rows/s, total persisted rows, queue depth and dropped rows. This is separate from firmware SPP TX Hz, Python RX Hz and GUI APP Hz.
- Incoming text responses (including ESTOP acknowledgments and periodic DIAG) use a separate high-priority Python queue. GUI telemetry backlog cannot block the SPP reader when it needs to forward these responses. Binary telemetry still goes to disk first, then UI.
- Compact Tkinter control row/buttons and responsive sensor graph widths complement the two equal dashboard columns and dedicated lower-right terminal.


## One CSV per START/STOP, simplified controls
- Only **BẮT ĐẦU DÒ LINE** and **DỪNG** are shown in the app. The former E-STOP and RESET_ESTOP dashboard buttons were removed; Escape now sends the normal STOP command. Firmware still supports ESTOP/RESET_ESTOP manually over SPP for safety diagnostics, but normal STOP is not a physical emergency cutoff.
- The dashboard run timer counts cumulative time to 1/100-second precision (`HH:MM:SS.hh`) while firmware-confirmed AUTO is active, pauses when AUTO stops or the SPP connection is lost, and can be reset independently without changing motor state. Timing uses the host monotonic clock and is session-only.
- Bluetooth connection alone does not create CSV. Every START immediately creates a new persistent `logs/linerobot_300hz_TIMESTAMP.csv` and begins capturing every valid SPP frame before sending AUTO. After ESP32 confirms STOP with `MOTOR: STOP`, the app detaches the logger, flushes any queued rows asynchronously and closes that CSV, keeping Bluetooth connected.
- Each subsequent START generates a new file; export copies the latest file. Disconnect, failed AUTO, ADC errors, external ESTOP, or STOP without acknowledgment close the current CSV with a clearly labeled reason. A 2-second STOP-ack timeout does not prove the motors have stopped.
- Data columns and 300Hz SPP stream remain unchanged. CSV status reports actual write rate, samples and drops during each run. Existing `run_csv_lifecycle.py` coordinates the per-run writer threads.


### Compact upper-right quarter, no scroll bar
- The dashboard's right half now uses a fixed 50/50 vertical split: the upper-right quarter holds all START/STOP, PWM, PID and calibration controls in a compact stack **without** a Canvas or scrolling; the lower-right quarter remains the embedded terminal. Reduced fonts, paddings, compact PWM/PID groups and shortened calibration labels reclaim vertical space. The 16-sensor dashboard remains in the left half.
- The terminal retains its own vertical/horizontal scroll bars (for long logs); the request to remove scrolling applies to the upper control dashboard. Firmware, SPP stream rate 300 Hz and per-run CSV lifecycle are untouched.


### CSV metadata: 16 per-eye raw calibration thresholds
- Every new run file contains one extra CSV row immediately below the 61-column header. The row's host_iso field is CAL_THRESHOLDS, t_ms is empty, and adc0..adc15 contain the sixteen raw threshold values TH0..TH15, with all remaining columns blank. Telemetry rows are unchanged.
- Values are parsed from firmware's CAL_DATA eye WHITE BLACK TH response, verified per sensor, and retained exactly rather than using a shared raw threshold. ESP32 uses TH_i = WHITE_i + ceil(0.4*(BLACK_i-WHITE_i)).
- START waits for a complete validated NVS CAL_DUMP so the row cannot accidentally contain placeholders; otherwise it re-requests CAL_DUMP rather than opening an incomplete CSV. Every START creates one new file; STOP closes it as before.
- For pandas telemetry analysis use pandas.read_csv(path, skiprows=[1]), or filter out the row where host_iso is CAL_THRESHOLDS. To inspect the thresholds, read the second row under adc0..adc15.
- This is an app-only CSV change; the firmware and SPP 300-Hz stream are unchanged.


## PID stability fix - five requested items (29 Oct 2026)
1. Keep all 16 independently calibrated NVS thresholds, no recalibration or threshold change; TH_i remains WHITE_i + ceil(0.4*(BLACK_i-WHITE_i)).
2. Logic black/white remains sensorRaw[i] >= thresholds[i] through sensor_is_black(i), independently for each eye.
3. Analog CAL_i=0..1000 still weights the 16-eye centroid, but only the eyes passing the SAME per-eye TH_i logic participate. Removes the old conflict CAL>=150 for centroid versus CAL>=400 for line detection.
4. AUTO must observe stable black detection for 10ms before starting/relocking. If a previously acquired line is lost during the first 5 seconds of AUTO, the robot drives straight at no more than PWM 65 until the line returns or the grace window expires. After that, 40ms without line pivots in place toward the last known line position at PWM 85 per wheel (170 PWM differential); search hands back to PID after three consecutive reliable samples (a centered sensor or an adjacent sensor pair), not a single far-edge trigger. The PID handoff suppresses D for four samples and ramps forward target from PWM 65 over four samples. Search is bounded to 2 seconds, then motors stop as a safety fallback.
5. Visible corners stay under PID control instead of switching to the fixed sharp-turn arc. PID may reverse the inner wheel. The governor sets PID target speed directly from line error (`baseSpeed * (1 - 0.45 * min(|error|/30, 1))`), with a minimum target of 45 PWM (or `baseSpeed` when lower). Outer-four-sensor detections reduce forward target by up to 40% early while steering authority scales with either outer-sensor severity or error; the allowed peak PID wheel command rises from governed speed to `maxSpeed` by error 60. During brief line loss, the forward arc uses its separate sharp-turn PWM profile; persistent loss falls back to bounded pivot search at PWM 85 per wheel. There is no timed acceleration after line reacquisition. A broad line retains its forward behavior; the normal finish stop latches only when all 16 sensors report black on 3 consecutive PID samples (resetting the counter if any sensor reads white). E-STOP, ADC faults, AUTO disable and the bounded search timeout remain independent safety stops.
The existing on-demand SPP DIAG reply now includes case=, line=, lost=, reacq= while keeping all old fields; Python app still decodes them. Stream default is now 300Hz (72-byte frames). Binary payload/CSV schema unchanged.
Hardware acceptance: raise wheels for first boot and verify pivot direction at low speed. With sensors all on white, initial AUTO does not move until a real line is seen steadily for 10ms. A confirmed loss during the first 5 seconds of AUTO drives straight at no more than PWM 65; afterward the robot pivots in place for at most 2 seconds to search, then stops safely if no line is found. Verify sensor order, motor polarity and E-STOP before higher speeds.
Known unresolved investigation: prior CSV showed near-duplicate sensors 4/12,5/13,6/14,7/15. Calibration and MUX wiring should be checked independently of PID.

## Three-map experimental logging (mainv10)

- Before START, explicitly select **MAP 1**, **MAP 2**, or **MAP 3** in the Python dashboard status bar. START stays disabled until a map is selected.
- Each START creates a uniquely timestamped CSV under `logs/map_01/`, `logs/map_02/`, or `logs/map_03/` according to the selected map. A per-run TXT terminal transcript with the same filename stem is saved automatically alongside it.
- Map buttons are locked while AUTO is running, START is pending, CSV is active, or STOP acknowledgment is pending. Changes apply only to future runs.
- Export CSV / Export TXT default to the folder and filename of the last run. CSV retains the same 61 columns and the `CAL_THRESHOLDS` row; do not mix this metadata row with actual samples. Existing root-level logs are left untouched.
- Map selection is solely data labeling in the Python app. It does not change ESP32 firmware, PID parameters, thresholds, motor commands, or Bluetooth frames. To collect an automatically grouped CSV, START must be issued from the Python app.
