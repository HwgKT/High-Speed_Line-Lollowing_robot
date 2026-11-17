// Build: g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
//        src/pid_controller.cpp tests/native/gap_bridge_smoke.cpp -o gap_bridge_smoke
// Run: ./gap_bridge_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t ticksUs = 0;
uint32_t millis(void) { return ticksUs / 1000; }
uint32_t micros(void) { return ticksUs; }
static int32_t simLeft = 0, simRight = 0;

extern "C" {
uint16_t sensorRaw[N_SENSORS] = {};
uint16_t minValues[N_SENSORS] = {};
uint16_t maxValues[N_SENSORS] = {};
uint16_t thresholds[N_SENSORS] = {};
bool calibrationReady = true;
bool isCalibrating = false;
static int16_t pwmL = 0, pwmR = 0;

bool sensor_adc_ready(void) { return true; }
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i < N_SENSORS && sensorRaw[i] >= thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i >= N_SENSORS) return 0;
    const uint16_t v = sensorRaw[i];
    if (v <= minValues[i]) return 0;
    if (v >= maxValues[i]) return 1000;
    return uint16_t(uint32_t(v - minValues[i]) * 1000U /
                    uint32_t(maxValues[i] - minValues[i]));
}
uint16_t sensor_get_line_bits(void) {
    uint16_t mask = 0;
    for (uint8_t i = 0; i < N_SENSORS; ++i)
        if (sensor_is_black(i)) mask |= uint16_t(1U << i);
    return mask;
}
void encoder_get_counts(int32_t* left, int32_t* right) {
    if (left) *left = simLeft;
    if (right) *right = simRight;
}
void motor_set_speeds(int16_t left, int16_t right) {
    assert(left >= -255 && left <= 255);
    assert(right >= -255 && right <= 255);
    pwmL = left; pwmR = right;
}
void motor_stop(void) { pwmL = pwmR = 0; }
}

static void stepMs(unsigned duration) {
    for (unsigned i = 0; i < duration; i += 2) {
        ticksUs += 2000U;
        pid_line_follow_step();
    }
}
static void setLine(void) {
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
    sensorRaw[7] = sensorRaw[8] = 2000;
}
static void setWhite(void) {
    for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
}
static void beginMap(uint8_t map, int32_t origin, int16_t peak=220, int16_t base=120) {
    pid_set_auto_mode(false);
    pid_init(1.6f, 0.0f, 0.8f, base);
    simLeft = origin;
    simRight = origin + 700;
    assert(pid_select_map(map));
    // MAP selection loads hardware defaults. Explicit test scenarios
    // override them afterward to exercise older and newer PWM profiles.
    pid_set_base_speed(base);
    assert(pid_set_high_speed_peak(peak));
    setLine();
    pid_set_auto_mode(true);
    assert(pid_get_run_encoder_progress() == 0);
    stepMs(14);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    assert(pwmL == base && pwmR == base);
}
static void advanceTo(int32_t relativeTicks, int32_t origin) {
    simLeft = origin + relativeTicks;
    simRight = origin + 700 + relativeTicks;
    assert(pid_get_run_encoder_progress() == relativeTicks);
}
int main(void) {
    for (int i = 0; i < N_SENSORS; ++i) {
        minValues[i] = 1000;
        maxValues[i] = 2000;
        thresholds[i] = 1400;
    }
    pid_init(1.6f, 0.0f, 0.8f, 120);
    assert(pid_get_kp() == 1.6f);
    assert(pid_get_ki() == 0.0f);
    assert(pid_get_kd() == 0.8f);
    assert(!pid_select_map(0));
    assert(!pid_select_map(4));
    // Boot defaults must work without an app for physical GPIO34 START.
    assert(pid_select_map(1));
    assert(pid_get_base_speed() == 130);
    assert(pid_get_high_speed_peak() == 240);
    assert(pid_select_map(2));
    assert(pid_get_base_speed() == 160);
    assert(pid_get_high_speed_peak() == 240);
    assert(pid_select_map(3));
    assert(pid_get_base_speed() == 150);
    assert(pid_get_high_speed_peak() == 240);
    // Re-selecting the SAME map preserves user changes from the app.
    pid_set_base_speed(145);
    assert(pid_set_high_speed_peak(245));
    assert(pid_select_map(3));
    assert(pid_get_base_speed() == 145);
    assert(pid_get_high_speed_peak() == 245);
    // Changing map restores that map's independent defaults.
    assert(pid_select_map(1));
    assert(pid_get_base_speed() == 130);
    assert(pid_get_high_speed_peak() == 240);

    beginMap(1, 50000);
    int32_t from = 0, to = 0;
    pid_get_gap_window(&from, &to);
    assert(from == 1500 && to == 2100);
    advanceTo(1650, 50000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 2);
    assert(pwmL == 220 && pwmR == 220);
    setWhite();
    stepMs(60);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    assert(pwmL == 220 && pwmR == 220);  // hold last forward PWM, not 95
    assert(!pid_select_map(2));          // no MAP change mid-run
    setLine();
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1750, 50000);
    setWhite();
    stepMs(92);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    stepMs(12); // 100ms measured from first white frame, not from 30ms loss
    assert(pid_get_current_case() == LINE_CASE_SHARP_RIGHT);
    assert(pwmL == 85 && pwmR == -85);
    stepMs(20);
    assert(pid_get_current_case() != LINE_CASE_GAP_BRIDGE); // cannot restart

    // A new run resets relative origin and timer; one window per MAP.
    beginMap(2, 70000);
    pid_get_gap_window(&from, &to);
    assert(from == 1050 && to == 1750);
    advanceTo(1130, 70000);
    setWhite();
    stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1640, 70000);
    setWhite(); stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);

    beginMap(3, 90000);
    pid_get_gap_window(&from, &to);
    assert(from == 1000 && to == 1700);
    advanceTo(1090, 90000);
    setWhite(); stepMs(30);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    advanceTo(1690, 90000);
    setWhite(); stepMs(20);
    advanceTo(1750, 90000); // crossing the upper boundary must not abort
    stepMs(50);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    setLine(); stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);

    // All MAPs share first-straight brake 1800->2300, then HOLD PWM 110.
    // Finishing the ramp must NOT return to BASE at encoder 2300.
    for (uint8_t map = 1; map <= 3; ++map) {
        const int32_t origin = 210000 + int32_t(map) * 10000;
        beginMap(map, origin);
        assert(pid_get_high_speed_stage() == 1);
        advanceTo(100, origin);
        stepMs(2);
        assert(pwmL >= 153 && pwmL <= 154 && pwmL == pwmR);
        advanceTo(300, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 2);
        assert(pwmL == 220 && pwmR == 220);
        advanceTo(1800, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 3);
        assert(pwmL == 220 && pwmR == 220);
        advanceTo(2050, origin);
        stepMs(2);
        assert(pwmL == 165 && pwmL == pwmR);
        advanceTo(2250, origin);
        stepMs(2);
        assert(pwmL == 121 && pwmL == pwmR);
        advanceTo(2300, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6); // HOLD
        assert(pwmL == 110 && pwmR == 110);
        advanceTo(2500, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        assert(pwmL == 110 && pwmR == 110);

        // Encoder threshold alone must not release when no line is visible.
        advanceTo(3099, origin);
        setLine();
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        advanceTo(3100, origin);
        setWhite();
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6); // position defaults to zero, but no valid line
        assert(pwmL == 110 && pwmR == 110);

        // A visible off-center line is insufficient; a centered line releases
        // on the very first PID sample (no 30-frame dwell / corner flag).
        for (int i = 0; i < N_SENSORS; ++i) sensorRaw[i] = 1000;
        sensorRaw[10] = 2000;
        stepMs(2);
        assert(pid_get_high_speed_stage() == 6);
        setLine();
        stepMs(2);
        assert(pid_get_high_speed_stage() == 4);
        assert(pwmL == 110 && pwmR == 110);
        // Completion is sticky; encoder reversal cannot restart the profile.
        advanceTo(1900, origin);
        stepMs(2);
        assert(pid_get_high_speed_stage() == 4);
    }

    // Only Peak 240..255 brakes earlier at E1400. The two original
    // windows remain untouched: Peak <230 starts E1800, 230..239 E1600.
    constexpr int16_t testPeaks[] = {229, 230, 239, 240, 255};
    for (uint8_t map = 1; map <= 3; ++map) {
        for (const int16_t peak : testPeaks) {
            const int32_t origin = 410000 + int32_t(map)*10000 + int32_t(peak)*100;
            beginMap(map, origin, peak);
            const int32_t expectedStart = peak >= 240 ? 1400 :
                                          peak >= 230 ? 1600 : 1800;
            assert(pid_get_high_speed_brake_start() == expectedStart);
            advanceTo(expectedStart - 1, origin);
            stepMs(2);
            assert(pid_get_high_speed_stage() == 2); // before BRAKE
            assert(pwmL == peak && pwmR == peak);
            advanceTo(expectedStart, origin);
            stepMs(2);
            assert(pid_get_high_speed_stage() == 3); // at BRAKE start
            assert(pwmL == peak && pwmR == peak);
            advanceTo(expectedStart + 100, origin);
            stepMs(2);
            assert(pid_get_high_speed_stage() == 3);
            assert(pwmL < peak && pwmR < peak);
            assert(pwmL > 110 && pwmR > 110);
            advanceTo(2300, origin);
            stepMs(2);
            assert(pid_get_high_speed_stage() == 6); // unchanged HOLD
            assert(pwmL == 110 && pwmR == 110);
        }
    }

    // MAP 1 has a later dashed segment (~1985-2058) inside BRAKE.
    // Its existing 100ms GAP_BRIDGE must still work while decelerating.
    beginMap(1, 400000);
    advanceTo(1985, 400000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 3); // BRAKE
    const int16_t gapBrakePwm = pwmL;
    assert(gapBrakePwm > 110 && gapBrakePwm < 220);
    setWhite();
    stepMs(60);
    assert(pid_get_current_case() == LINE_CASE_GAP_BRIDGE);
    assert(pwmL == gapBrakePwm && pwmR == gapBrakePwm);
    advanceTo(2058, 400000);
    setLine();
    stepMs(2);
    assert(pid_get_current_case() == LINE_CASE_NORMAL_PID);
    assert(pid_get_high_speed_stage() == 3);

    // Peak is configurable only while stopped. Low peaks brake to 110 when peak >= 110.
    pid_set_auto_mode(false);
    assert(pid_set_high_speed_peak(180));
    assert(pid_get_high_speed_peak() == 180);
    assert(!pid_set_high_speed_peak(99));
    assert(!pid_set_high_speed_peak(256));
    assert(pid_select_map(1));
    simLeft=300000; simRight=300700;
    setLine(); pid_set_auto_mode(true);
    stepMs(14);
    simLeft+=300; simRight+=300;
    stepMs(2);
    assert(pwmL==180 && pwmR==180);
    assert(!pid_set_high_speed_peak(220)); // frozen during AUTO
    simLeft+=1500; simRight+=1500; // 1800
    stepMs(2);
    assert(pid_get_high_speed_stage()==3);
    assert(pwmL==180 && pwmR==180);
    simLeft+=250; simRight+=250; // 2050
    stepMs(2);
    assert(pwmL==145 && pwmR==145);
    simLeft+=250; simRight+=250; // 2300
    stepMs(2);
    assert(pid_get_high_speed_stage()==6);
    assert(pwmL==110 && pwmR==110);

    // HOLD=110 is a genuine target when BASE=100 or BASE=140;
    // it must not silently become min(110, BASE).
    beginMap(2, 610000, 220, 100);
    advanceTo(2300, 610000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 6);
    assert(pwmL == 110 && pwmR == 110);
    beginMap(2, 620000, 220, 140);
    advanceTo(2300, 620000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 6);
    assert(pwmL == 110 && pwmR == 110);

    // A peak below 110 must never accelerate during a supposed BRAKE.
    beginMap(2, 630000, 100, 100);
    advanceTo(2300, 630000);
    stepMs(2);
    assert(pid_get_high_speed_stage() == 6);
    assert(pwmL == 100 && pwmR == 100);

    // Out of the MAP 1 window: preserve the existing 30ms + startup coast.
    beginMap(1, 150000);
    advanceTo(1200, 150000);
    setWhite(); stepMs(14);
    assert(pid_get_current_case() == LINE_CASE_DASHED_LINE);
    stepMs(32);
    assert(pid_get_current_case() == LINE_CASE_STARTUP_COAST);
    assert(pwmL == 95 && pwmR == 95);
    pid_set_auto_mode(false);
    assert(pwmL == 0 && pwmR == 0);
    puts("PASS: MAP 1/2/3 GAP and HS BRAKE 1800(<230)/1600(230-239)/1400(240-255), HOLD110");
    return 0;
}
