#include "pid_controller.h"
#include "encoder.h"
#include "sensor.h"
#include "motor.h"
#include <Arduino.h>
#include <math.h>

// Sensor 0..7 are left of center; 8..15 are right of center.
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f,
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f
};

// Optimised tune for sharp-turn responsiveness.  Gains are still live-tuneable
// via the SPP KP:/KI:/KD: commands.
static PID_Config_t pidConfig = {1.6f, 0.0f, 0.8f, 100, 255, -100};
static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0, integralError = 0, lastCorrection = 0;
static float lastPTerm = 0, lastITerm = 0, lastDTerm = 0;
static float lastKnownPosition = 0;
static uint32_t lastComputeTimeUs = 0;
static bool derivativeHistoryValid = false;
static float lastDashedCorrection = 0;   // remembered PID output for dashed-line coast

namespace {
constexpr uint32_t LINE_ACQUIRE_MS = 10;
constexpr uint32_t LINE_LOSS_MS = 30;           // was 40 — faster loss detection reduces stutter duration
constexpr uint32_t LINE_SEARCH_TIMEOUT_MS = 2000;
constexpr uint32_t STARTUP_LOSS_GRACE_MS = 5000;
// Finish requires ALL 16 sensors to see black on 3 consecutive PID samples.
constexpr uint8_t FULL_LINE_MIN_SENSORS = N_SENSORS;
constexpr uint8_t FINISH_CONFIRM_FRAMES = 3;
constexpr int16_t DROPOUT_FORWARD_PWM = 95;    // was 65 — higher coast speed maintains momentum over dashed lines
constexpr int16_t LINE_SEARCH_PWM = 85;
constexpr uint8_t SEARCH_REACQUIRE_FRAMES = 3;
constexpr uint8_t REACQUIRE_DERIVATIVE_HOLD_FRAMES = 4;
constexpr uint8_t REACQUIRE_SPEED_RAMP_FRAMES = 4;
// Named-track dashed-line coast. Encoder positions are relative to AUTO start.
// These are provisional windows extracted from the 2026-10-31 map logs.
constexpr uint32_t GAP_TIMEOUT_MS = 100;
// First straight trapezoid in relative motor-encoder ticks.
// An experimental open-loop PWM envelope, not closed-loop wheel velocity.
constexpr int32_t HS_RAMP_END_ENC = 300;
// Shared first-straight corner profile for MAP 1/2/3.
constexpr int32_t HS_BRAKE_START_ENC = 1800;        // Peak 100..229
constexpr int32_t HS_EARLY_BRAKE_START_ENC = 1600;  // Peak 230..239
constexpr int16_t HS_EARLY_BRAKE_PEAK_THRESHOLD = 230;
constexpr int32_t HS_VERY_EARLY_BRAKE_START_ENC = 1400; // Peak 240..255
constexpr int16_t HS_VERY_EARLY_BRAKE_PEAK_THRESHOLD = 240;
constexpr int32_t HS_BRAKE_END_ENC = 2300;
constexpr int32_t HS_HOLD_MIN_END_ENC = 3100;
constexpr float HS_CORNER_HOLD_PWM = 110.0f;
constexpr float HS_HOLD_CENTER_ERROR = 15.0f; // valid line position near center
int16_t highSpeedPeakPwm = 240; // All three MAPs default to peak PWM 240.
enum HighSpeedStage : uint8_t {
    HS_DISABLED=0, HS_ACCEL=1, HS_CRUISE=2, HS_BRAKE=3,
    HS_FINISHED=4, HS_ABORTED=5, HS_CORNER_HOLD=6
};
HighSpeedStage highSpeedStage = HS_DISABLED;
bool highSpeedAborted = false;
int32_t highSpeedMaxProgress = 0;

static int32_t highSpeedBrakeStart() {
    // Peak can only change when AUTO is disabled: the window remains fixed
    // throughout a run for all three map profiles.
    if (highSpeedPeakPwm >= HS_VERY_EARLY_BRAKE_PEAK_THRESHOLD)
        return HS_VERY_EARLY_BRAKE_START_ENC;
    return highSpeedPeakPwm >= HS_EARLY_BRAKE_PEAK_THRESHOLD
        ? HS_EARLY_BRAKE_START_ENC : HS_BRAKE_START_ENC;
}

static float highSpeedForwardTarget(int32_t ticks, float basePwm) {
    const float peak = fmaxf(basePwm, (float)highSpeedPeakPwm);
    // HOLD target is independent of BASE (e.g. BASE=100, HOLD=110).
    // Never turn braking into acceleration if HS_PEAK is below 110.
    const float holdPwm = fminf(HS_CORNER_HOLD_PWM, peak);
    if (ticks <= 0) return basePwm;
    if (ticks < HS_RAMP_END_ENC) {
        return basePwm + (peak - basePwm) *
            ((float)ticks / (float)HS_RAMP_END_ENC);
    }
    const int32_t brakeStart = highSpeedBrakeStart();
    if (ticks < brakeStart) return peak;
    if (ticks < HS_BRAKE_END_ENC) {
        return peak +
            (holdPwm - peak) *
            ((float)(ticks - brakeStart) /
             (float)(HS_BRAKE_END_ENC - brakeStart));
    }
    // Keep low forward thrust until encoder progress and a valid centered
    // line sample satisfy the HOLD release conditions in pid_detect_case().
    // HS_FINISHED bypasses this envelope and returns to normal BASE.
    return holdPwm;
}
struct GapWindow { int32_t begin; int32_t end; };
constexpr GapWindow GAP_WINDOWS[3] = {
    {1500, 2100}, // MAP 1
    {1050, 1750}, // MAP 2
    {1000, 1700}  // MAP 3
};
uint8_t selectedMap = 0;  // setup() selects MAP 1; MAP:2/3 may override.
constexpr int16_t MAP_DEFAULT_BASE_PWM[3] = {130, 160, 150};
constexpr int16_t MAP_DEFAULT_HS_PEAK_PWM = 240;
int32_t runEncoderOriginL = 0, runEncoderOriginR = 0;
bool gapActive = false;
uint32_t gapStartMs = 0;
int16_t gapHoldPwm = 0, lastNormalForwardPwm = 0;

static int32_t runEncoderProgress() {
    int32_t left = 0, right = 0;
    encoder_get_counts(&left, &right);
    const int64_t sum = ((int64_t)left - runEncoderOriginL) +
                        ((int64_t)right - runEncoderOriginR);
    return (int32_t)(sum / 2);
}
constexpr float OUTER_SENSOR_SPEED_REDUCTION = 0.40f;

// A sharp turn is entered only when the centroid reaches the outer region.
// If the line is lost, AUTO pivots in place to search for it.
constexpr float SHARP_TURN_POSITION = 40.0f;          // was 43 — enter turn earlier
constexpr float SHARP_TURN_POSITION_WITH_OUTER_EYES = 28.0f; // was 31
constexpr uint32_t SHARP_TURN_PROFILE_MS = 700;
constexpr float TURN_CENTERED_POSITION = 22.0f;
constexpr uint8_t TURN_CENTERED_FRAMES = 2;
constexpr uint16_t TURN_CENTER_SENSOR_MASK = 0x03C0U;
constexpr uint8_t TURN_CENTER_SENSOR_MIN = 2;
constexpr int16_t TURN_PWM_MIN = 100;           // was 95 — slightly more aggressive
constexpr int16_t TURN_PWM_MAX = 185;           // was 170 — allow faster outer wheel
constexpr float TURN_CURVE_START_POSITION = 28.0f;
constexpr float TURN_CURVE_END_POSITION = 75.0f;
constexpr float TURN_CURVE_INNER_FACTOR_OPEN = 0.75f;
constexpr float TURN_CURVE_INNER_FACTOR_TIGHT = 0.35f;

constexpr float SPEED_GOVERNOR_FULL_ERROR = 30.0f;
constexpr float SPEED_GOVERNOR_REDUCTION = 0.45f;
constexpr float PID_CORRECTION_BASE_MULTIPLIER = 6.0f;
constexpr float OUTER_STEERING_START_ERROR = 20.0f;
constexpr float OUTER_STEERING_FULL_ERROR = 60.0f;

enum TurnState : uint8_t {
    TURN_NONE,
    TURN_LEFT,
    TURN_RIGHT,
    SEARCH_LEFT,
    SEARCH_RIGHT
};
TurnState turnState = TURN_NONE;
TurnState lastTurnDirection = TURN_RIGHT;
uint32_t turnStartedMs = 0;
uint32_t searchStartedMs = 0;
uint32_t autoStartedMs = 0;
uint32_t turnMissingSinceMs = 0;
uint8_t centeredTurnFrames = 0;
uint8_t derivativeWarmupFrames = 0;
uint8_t reacquireSpeedFrames = 0;
bool turnMissing = false;
bool startupLossActive = false;
bool finishLineLatched = false;
uint8_t fullLineFrames = 0;

bool lineLocked = false, previouslyLocked = false;
bool acquiring = false, missing = false;
uint32_t acquireSinceMs = 0, missingSinceMs = 0;
uint32_t confirmedLosses = 0, reacquisitions = 0;
float governedForwardSpeed = 0.0f;

static int16_t clampPwm(float value) {
    if (value <= 0.0f) return 0;
    if (value >= pidConfig.maxSpeed) return pidConfig.maxSpeed;
    return (int16_t)value;
}
static int16_t clampPwmSigned(float value) {
    if (value >= pidConfig.maxSpeed) return pidConfig.maxSpeed;
    if (value <= -pidConfig.maxSpeed) return -pidConfig.maxSpeed;
    return (int16_t)value;
}
static uint8_t bitCount16(uint16_t value) {
    return (uint8_t)__builtin_popcount((unsigned)value);
}
static float outerSensorSeverity(uint16_t mask) {
    float severity = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
        if (mask & (uint16_t)(1U << i)) {
            const float sensorSeverity = (float)(4U - i) * 0.25f;
            if (sensorSeverity > severity) severity = sensorSeverity;
        }
        const uint8_t rightSensor = (uint8_t)(15U - i);
        if (mask & (uint16_t)(1U << rightSensor)) {
            const float sensorSeverity = (float)(4U - i) * 0.25f;
            if (sensorSeverity > severity) severity = sensorSeverity;
        }
    }
    return severity;
}
static bool hasAdjacentLineSensors(uint16_t mask) {
    return (mask & (uint16_t)(mask >> 1)) != 0;
}
static void limitMotorCommandPeak(float& left, float& right, float limit) {
    const float peak = fmaxf(fabsf(left), fabsf(right));
    if (peak > limit && peak > 0.0f) {
        const float scale = limit / peak;
        left *= scale;
        right *= scale;
    }
}
static float updateForwardSpeed(float absErr) {
    const float errorRatio = fminf(absErr / SPEED_GOVERNOR_FULL_ERROR, 1.0f);
    float targetSpeed = (float)pidConfig.baseSpeed *
        (1.0f - SPEED_GOVERNOR_REDUCTION * errorRatio);
    const float minimumSpeed = (float)(pidConfig.baseSpeed < 45
        ? pidConfig.baseSpeed : 45);
    if (targetSpeed < minimumSpeed) targetSpeed = minimumSpeed;
    if (targetSpeed > pidConfig.maxSpeed) targetSpeed = pidConfig.maxSpeed;
    governedForwardSpeed = targetSpeed;
    return governedForwardSpeed;
}
static void clearPidState() {
    lastError = integralError = lastCorrection = 0;
    lastPTerm = lastITerm = lastDTerm = 0;
    derivativeHistoryValid = false;
    derivativeWarmupFrames = 0;
}
static void resetLineTracking() {
    lineLocked = previouslyLocked = acquiring = missing = false;
    acquireSinceMs = missingSinceMs = 0;
    lastKnownPosition = 0;
    turnState = TURN_NONE;
    lastTurnDirection = TURN_RIGHT;
    turnStartedMs = 0;
    searchStartedMs = 0;
    turnMissingSinceMs = 0;
    centeredTurnFrames = 0;
    derivativeWarmupFrames = 0;
    reacquireSpeedFrames = 0;
    turnMissing = false;
    startupLossActive = false;
    finishLineLatched = false;
    fullLineFrames = 0;
    lastDashedCorrection = 0;
    governedForwardSpeed = 0.0f;
    gapActive = false;
    gapStartMs = 0;
    gapHoldPwm = lastNormalForwardPwm = 0;
    highSpeedStage = HS_DISABLED;
    highSpeedAborted = false;
    highSpeedMaxProgress = 0;
}

// Sharp turn: outer wheel PWM adapts to how long we've been turning.
// Start faster, ramp down as timeout approaches to avoid overshooting.
static int16_t sharpTurnPwm() {
    const uint32_t elapsed = (uint32_t)(millis() - turnStartedMs);
    // First 200 ms: full power.  Then linearly taper to 70% by timeout.
    float frac = 1.0f;
    if (elapsed > 200) {
        frac = 1.0f - 0.3f * (float)(elapsed - 200) / (float)(SHARP_TURN_PROFILE_MS - 200);
        if (frac < 0.7f) frac = 0.7f;
    }
    int32_t speed = (int32_t)((float)(pidConfig.baseSpeed + 40) * frac);
    if (speed < TURN_PWM_MIN) speed = TURN_PWM_MIN;
    if (speed > TURN_PWM_MAX) speed = TURN_PWM_MAX;
    return (int16_t)speed;
}
static int16_t sharpTurnInnerPwm(int16_t outer, float position) {
    float severity = (fabsf(position) - TURN_CURVE_START_POSITION) /
                     (TURN_CURVE_END_POSITION - TURN_CURVE_START_POSITION);
    if (severity < 0.0f) severity = 0.0f;
    if (severity > 1.0f) severity = 1.0f;
    const float factor = TURN_CURVE_INNER_FACTOR_OPEN -
        severity * (TURN_CURVE_INNER_FACTOR_OPEN - TURN_CURVE_INNER_FACTOR_TIGHT);
    return (int16_t)((float)outer * factor);
}
static bool isSearchTurn() {
    return turnState == SEARCH_LEFT || turnState == SEARCH_RIGHT;
}
static bool isLeftTurn() {
    return turnState == TURN_LEFT || turnState == SEARCH_LEFT;
}
static TurnState searchStateFor(TurnState direction) {
    return direction == TURN_LEFT ? SEARCH_LEFT : SEARCH_RIGHT;
}
} // namespace

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = (baseSpeed < 0) ? 0 : (baseSpeed > 255 ? 255 : baseSpeed);
    pidConfig.minSpeed = 0;
    autoMode = false;
    selectedMap = 0;
    highSpeedPeakPwm = MAP_DEFAULT_HS_PEAK_PWM;
    currentCase = LINE_CASE_STOPPED;
    clearPidState();
    resetLineTracking();
    confirmedLosses = reacquisitions = 0;
    lastComputeTimeUs = micros();
}
void pid_set_tunings(float kp, float ki, float kd) {
    if (pidConfig.Kd != kd) derivativeHistoryValid = false;
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}
void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = speed < 0 ? 0 : (speed > 255 ? 255 : speed);
}
void pid_set_auto_mode(bool enabled) {
    autoMode = enabled;
    currentCase = LINE_CASE_STOPPED;
    resetLineTracking();
    clearPidState();
    lastComputeTimeUs = micros();
    autoStartedMs = millis();
    if (enabled) {
        encoder_get_counts(&runEncoderOriginL, &runEncoderOriginR);
        highSpeedStage = selectedMap >= 1 && selectedMap <= 3 ? HS_ACCEL : HS_DISABLED;
    }
    motor_stop(); // Never start blind; wait for stable black-line detection.
}
bool pid_is_auto_mode(void) { return autoMode; }
bool pid_select_map(uint8_t mapId) {
    if (autoMode || mapId < 1 || mapId > 3) return false;
    // Apply the track defaults only when changing MAP, not when the Python
    // app resends AUTO:<same map> after an intentional manual adjustment.
    if (selectedMap != mapId) {
        pidConfig.baseSpeed = MAP_DEFAULT_BASE_PWM[mapId - 1];
        highSpeedPeakPwm = MAP_DEFAULT_HS_PEAK_PWM;
    }
    selectedMap = mapId;
    return true;
}
uint8_t pid_get_selected_map(void) { return selectedMap; }
uint8_t pid_get_high_speed_stage(void) { return (uint8_t)highSpeedStage; }
bool pid_set_high_speed_peak(int16_t pwm) {
    if (autoMode || pwm < 100 || pwm > 255 || pwm < pidConfig.baseSpeed)
        return false;
    highSpeedPeakPwm = pwm;
    return true;
}
int16_t pid_get_high_speed_peak(void) { return highSpeedPeakPwm; }
int32_t pid_get_high_speed_brake_start(void) { return highSpeedBrakeStart(); }
int32_t pid_get_run_encoder_progress(void) { return runEncoderProgress(); }
void pid_get_run_encoder_origins(int32_t* left, int32_t* right) {
    if (left) *left = runEncoderOriginL;
    if (right) *right = runEncoderOriginR;
}
void pid_get_gap_window(int32_t* begin, int32_t* end) {
    const GapWindow w = selectedMap >= 1 && selectedMap <= 3
                      ? GAP_WINDOWS[selectedMap - 1] : GapWindow{0, 0};
    if (begin) *begin = w.begin;
    if (end) *end = w.end;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    if (!outPosition || !outBlackCount) return false;
    float weighted = 0;
    uint32_t strength = 0;
    uint8_t blackCount = 0;
    if (calibrationReady) {
        for (int i = 0; i < N_SENSORS; i++) {
            if (!sensor_is_black(i)) continue;
            const uint16_t cal = sensor_get_calibrated(i);
            weighted += (float)cal * SENSOR_WEIGHTS[i];
            strength += cal;
            ++blackCount;
        }
    }
    *outBlackCount = blackCount;
    if (blackCount > 0 && strength > 0) {
        *outPosition = weighted / (float)strength;
        lastKnownPosition = *outPosition;
        return true;
    }
    *outPosition = lastKnownPosition; // Position memory is not line evidence.
    return false;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady || !sensor_adc_ready()) return LINE_CASE_STOPPED;
    if (finishLineLatched) return LINE_CASE_STOPPED;
    const uint32_t now = millis();
    const uint16_t mask = sensor_get_line_bits();
    // Stage uses monotonic maximum forward progress, not time or raw encoder
    // sum; wheel reversal during SEARCH must not reactivate earlier zones.
    if (selectedMap >= 1 && selectedMap <= 3 && !highSpeedAborted) {
        const int32_t progress = runEncoderProgress();
        if (progress > highSpeedMaxProgress) highSpeedMaxProgress = progress;
        if (isSearchTurn() && highSpeedMaxProgress < HS_BRAKE_END_ENC) {
            // An unexpected pivot before braking is complete cancels HS.
            highSpeedAborted = true;
            highSpeedStage = HS_ABORTED;
        } else if (highSpeedStage == HS_FINISHED) {
            // Completion is sticky; encoder reversal cannot restart HS.
        } else if (highSpeedMaxProgress >= HS_BRAKE_END_ENC) {
            // Release immediately when both requested conditions are met:
            // current encoder progress has passed the threshold and a valid
            // line measurement is near center. blackCount>0 prevents a false
            // release because absent-line samples only return lastKnownPosition,
            // which may still be near center and is not current line evidence.
            const bool validCenteredLine =
                blackCount > 0 && fabsf(position) <= HS_HOLD_CENTER_ERROR;
            highSpeedStage =
                (progress >= HS_HOLD_MIN_END_ENC && validCenteredLine)
                    ? HS_FINISHED : HS_CORNER_HOLD;
        } else if (highSpeedMaxProgress >= highSpeedBrakeStart()) {
            highSpeedStage = HS_BRAKE;
        } else if (highSpeedMaxProgress >= HS_RAMP_END_ENC) {
            highSpeedStage = HS_CRUISE;
        } else {
            highSpeedStage = HS_ACCEL;
        }
    }

    if (bitCount16(mask) == FULL_LINE_MIN_SENSORS) {
        if (fullLineFrames < FINISH_CONFIRM_FRAMES) ++fullLineFrames;
        if (fullLineFrames >= FINISH_CONFIRM_FRAMES) {
            finishLineLatched = true;
            turnState = TURN_NONE;
            centeredTurnFrames = 0;
            return LINE_CASE_STOPPED;
        }
    } else {
        fullLineFrames = 0;
    }

    // GAP_BRIDGE has priority over startup grace and the usual 30ms dropout.
    // One timer per continuous white segment; no reentry after its timeout.
    if (gapActive) {
        if (blackCount == 0) {
            if ((uint32_t)(now - gapStartMs) < GAP_TIMEOUT_MS)
                return LINE_CASE_GAP_BRIDGE;
            gapActive = false;
            startupLossActive = false;
            lineLocked = false;
            acquiring = missing = false;
            ++confirmedLosses;
            turnState = searchStateFor(lastKnownPosition < 0.0f ? TURN_LEFT :
                                       lastKnownPosition > 0.0f ? TURN_RIGHT :
                                       lastTurnDirection);
            searchStartedMs = now;
            centeredTurnFrames = 0;
            clearPidState();
            return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
        }
        // Line returns before 100 ms: resume existing PID without stale D term.
        gapActive = false;
        missing = false;
        derivativeHistoryValid = false;
        derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
    }

    if (blackCount == 0 && turnState == TURN_NONE && lineLocked &&
        !startupLossActive && selectedMap >= 1 && selectedMap <= 3) {
        const int32_t progress = runEncoderProgress();
        const GapWindow w = GAP_WINDOWS[selectedMap - 1];
        if (progress >= w.begin && progress <= w.end) {
            gapActive = true;
            gapStartMs = now;
            gapHoldPwm = lastNormalForwardPwm > 0
                         ? lastNormalForwardPwm : clampPwm(pidConfig.baseSpeed);
            return LINE_CASE_GAP_BRIDGE;
        }
    }

    if (startupLossActive) {
        if (blackCount > 0) {
            startupLossActive = false;
            missing = false;
        } else if (uint32_t(now - autoStartedMs) < STARTUP_LOSS_GRACE_MS) {
            return LINE_CASE_STARTUP_COAST;
        } else {
            startupLossActive = false;
        }
    }

    // Keep PID steering while a corner remains visible; pivot only after line loss.
    if (turnState != TURN_NONE) {
        if (isSearchTurn()) {
            const bool reliableReacquisition =
                (blackCount >= 2 && hasAdjacentLineSensors(mask)) ||
                (blackCount > 0 && fabsf(position) <= TURN_CENTERED_POSITION);
            if (reliableReacquisition) {
                if (centeredTurnFrames < SEARCH_REACQUIRE_FRAMES)
                    ++centeredTurnFrames;
            } else {
                centeredTurnFrames = 0;
            }
            if (centeredTurnFrames >= SEARCH_REACQUIRE_FRAMES) {
                turnState = TURN_NONE;
                centeredTurnFrames = 0;
                missing = false;
                turnMissing = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES;
                reacquireSpeedFrames = REACQUIRE_SPEED_RAMP_FRAMES;
                lineLocked = true;
                if (previouslyLocked) ++reacquisitions;
                previouslyLocked = true;
                if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;
                return LINE_CASE_NORMAL_PID;
            }
        } else {
            const bool narrowCenteredLine =
                blackCount > 0 && fabsf(position) <= TURN_CENTERED_POSITION &&
                blackCount <= 6 && (mask & 0xF00FU) == 0;
            const bool wideCenteredLine =
                bitCount16(mask & TURN_CENTER_SENSOR_MASK) >= TURN_CENTER_SENSOR_MIN;
            if (narrowCenteredLine || wideCenteredLine) {
                if (centeredTurnFrames < TURN_CENTERED_FRAMES)
                    ++centeredTurnFrames;
            } else {
                centeredTurnFrames = 0;
            }
            if (centeredTurnFrames >= TURN_CENTERED_FRAMES) {
                turnState = TURN_NONE;
                centeredTurnFrames = 0;
                missing = false;
                derivativeHistoryValid = false;
                derivativeWarmupFrames = REACQUIRE_DERIVATIVE_HOLD_FRAMES; // guard against D-spike on turn exit
                lineLocked = true;
                if (previouslyLocked) ++reacquisitions;
                previouslyLocked = true;
                if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;
                return LINE_CASE_NORMAL_PID;
            }
        }

        if (!isSearchTurn() && blackCount == 0) {
            if (!turnMissing) {
                turnMissing = true;
                turnMissingSinceMs = now;
            } else if (uint32_t(now - turnMissingSinceMs) >= LINE_LOSS_MS) {
                turnState = searchStateFor(turnState);
                searchStartedMs = now;
                centeredTurnFrames = 0;
                turnMissing = false;
                clearPidState();
                return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
            }
        } else {
            turnMissing = false;
        }

        if (isSearchTurn() &&
            uint32_t(now - searchStartedMs) >= LINE_SEARCH_TIMEOUT_MS) {
            turnState = TURN_NONE;
            centeredTurnFrames = 0;
            lineLocked = false;
            acquiring = missing = false;
            turnMissing = false;
            ++confirmedLosses;
            return LINE_CASE_LOST_LINE;
        }
        if (!isSearchTurn() && blackCount > 0)
            return LINE_CASE_NORMAL_PID;
        return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
    }

    if (blackCount > 0) {
        missing = false;
        if (!lineLocked) {
            if (!acquiring) {
                acquiring = true;
                acquireSinceMs = now;
            }
            if (uint32_t(now - acquireSinceMs) < LINE_ACQUIRE_MS)
                return LINE_CASE_LOST_LINE;
            lineLocked = true;
            acquiring = false;
            if (previouslyLocked) ++reacquisitions;
            previouslyLocked = true;
        }

        // Cross-line takes priority over corner classification.
        if (bitCount16(mask) >= 11) return LINE_CASE_CROSS_LINE;

        const uint8_t leftOuter = bitCount16(mask & 0x000FU);
        const uint8_t rightOuter = bitCount16(mask & 0xF000U);
        const bool sharpLeft =
            position <= -SHARP_TURN_POSITION ||
            (position <= -SHARP_TURN_POSITION_WITH_OUTER_EYES && leftOuter >= 2);
        const bool sharpRight =
            position >= SHARP_TURN_POSITION ||
            (position >= SHARP_TURN_POSITION_WITH_OUTER_EYES && rightOuter >= 2);

        if (sharpLeft || sharpRight) {
            turnState = sharpLeft ? TURN_LEFT : TURN_RIGHT;
            lastTurnDirection = turnState;
            turnStartedMs = now;
            centeredTurnFrames = 0;
            turnMissing = false;
            clearPidState();
            return LINE_CASE_NORMAL_PID;
        }
        return LINE_CASE_NORMAL_PID;
    }

    acquiring = false;
    if (!lineLocked) return LINE_CASE_LOST_LINE;
    if (!missing) {
        missing = true;
        missingSinceMs = now;
    }
    if (uint32_t(now - missingSinceMs) < LINE_LOSS_MS)
        return LINE_CASE_DASHED_LINE;
    if (previouslyLocked &&
        uint32_t(now - autoStartedMs) < STARTUP_LOSS_GRACE_MS) {
        startupLossActive = true;
        return LINE_CASE_STARTUP_COAST;
    }
    lineLocked = false;
    missing = false;
    ++confirmedLosses;
    turnState = searchStateFor(lastKnownPosition < 0.0f ? TURN_LEFT :
                               lastKnownPosition > 0.0f ? TURN_RIGHT :
                               lastTurnDirection);
    searchStartedMs = now;
    centeredTurnFrames = 0;
    clearPidState();
    return isLeftTurn() ? LINE_CASE_SHARP_LEFT : LINE_CASE_SHARP_RIGHT;
}

void pid_line_follow_step(void) {
    if (!autoMode) return;
    float pos = 0;
    uint8_t blackCount = 0;
    const bool good = pid_calculate_position(&pos, &blackCount);
    const LineCase_t prev = currentCase;
    currentCase = pid_detect_case(pos, blackCount);
    const uint32_t nowUs = micros();
    const float measuredDt = (float)(nowUs - lastComputeTimeUs) * 1e-6f;
    const bool dtValid = measuredDt > 0 && measuredDt <= 0.1f;
    const float dt = dtValid ? measuredDt : 0.01f;
    lastComputeTimeUs = nowUs;
    if (prev != LINE_CASE_NORMAL_PID || currentCase != LINE_CASE_NORMAL_PID)
        derivativeHistoryValid = false;

    if (currentCase == LINE_CASE_SHARP_LEFT) {
        clearPidState();
        if (turnState == SEARCH_LEFT) {
            motor_set_speeds(-LINE_SEARCH_PWM, LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(inner, outer);
        }
    } else if (currentCase == LINE_CASE_SHARP_RIGHT) {
        clearPidState();
        if (turnState == SEARCH_RIGHT) {
            motor_set_speeds(LINE_SEARCH_PWM, -LINE_SEARCH_PWM);
        } else {
            const int16_t outer = sharpTurnPwm();
            const int16_t inner = sharpTurnInnerPwm(outer, pos);
            motor_set_speeds(outer, inner);
        }
    } else if (currentCase == LINE_CASE_NORMAL_PID && good) {
        const float err = pos;
        const float absErr = fabsf(err);

        // Non-linear P gain: ramps up quadratically as error increases.
        // This makes the robot react gently near center but aggressively near edges.
        const float p = pidConfig.Kp * (1.0f + 0.018f * absErr) * err;

        // Integral with anti-windup: tighter clamp + decay when error crosses zero
        if ((integralError > 0 && err < 0) || (integralError < 0 && err > 0))
            integralError *= 0.5f;  // fast decay on zero-crossing to avoid overshoot
        integralError += err * dt;
        if (integralError > 80.0f) integralError = 80.0f;
        if (integralError < -80.0f) integralError = -80.0f;
        const float it = pidConfig.Ki * integralError;

        // Derivative on error with low-pass filter to reduce noise spikes
        float d = 0.0f;
        if (derivativeWarmupFrames > 0) {
            --derivativeWarmupFrames;
        } else if (pidConfig.Kd != 0 && derivativeHistoryValid && dtValid) {
            const float rawD = pidConfig.Kd * (err - lastError) / measuredDt;
            // First-order low-pass: alpha=0.6 balances responsiveness vs noise
            d = 0.6f * rawD + 0.4f * lastDTerm;
            // Hard clamp: prevents catastrophic D-spikes when re-entering PID after a turn
            constexpr float D_CLAMP = 200.0f;
            if (d >  D_CLAMP) d =  D_CLAMP;
            if (d < -D_CLAMP) d = -D_CLAMP;
        }
        lastError = err;
        derivativeHistoryValid = true;
        lastPTerm = p;
        lastITerm = it;
        lastDTerm = d;
        lastCorrection = p + it + d;

        float base = updateForwardSpeed(absErr);
        // Replace only the forward-speed target on the first straight. Keep
        // existing PID steering, error governor, outer-eye limits, and SEARCH.
        // For BASE:120, command envelope is 120 -> 255 -> 120.
        const bool highSpeedStraight =
            (highSpeedStage == HS_ACCEL || highSpeedStage == HS_CRUISE) &&
            turnState == TURN_NONE;
        const bool highSpeedDecelOrHold =
            highSpeedStage == HS_BRAKE ||
            highSpeedStage == HS_CORNER_HOLD;
        // Do not drop the low-speed target merely because TURN_RIGHT was
        // detected: NORMAL_PID can continue steering through the corner.
        // Dedicated sharp-turn/search cases retain their pivot commands.
        if (highSpeedStraight || highSpeedDecelOrHold) {
                const float profile = highSpeedForwardTarget(
                    highSpeedMaxProgress, (float)pidConfig.baseSpeed);
                const float errorRatio = fminf(
                    absErr / SPEED_GOVERNOR_FULL_ERROR, 1.0f);
                const float minimumSpeed = (float)(pidConfig.baseSpeed < 45
                    ? pidConfig.baseSpeed : 45);
                base = fmaxf(minimumSpeed,
                    profile * (1.0f - SPEED_GOVERNOR_REDUCTION * errorRatio));
                if (base > pidConfig.maxSpeed) base = pidConfig.maxSpeed;
                governedForwardSpeed = base;
        }
        const uint16_t sensorMask = sensor_get_line_bits();
        const float edgeSeverity = outerSensorSeverity(sensorMask);
        if (edgeSeverity > 0.0f) {
            const float minimumSpeed = (float)(pidConfig.baseSpeed < 45
                ? pidConfig.baseSpeed : 45);
            const float edgeSpeed = fmaxf(
                minimumSpeed,
                (float)pidConfig.baseSpeed *
                    (1.0f - OUTER_SENSOR_SPEED_REDUCTION * edgeSeverity));
            if (edgeSpeed < base) base = edgeSpeed;
            governedForwardSpeed = base;
        }
        if (reacquireSpeedFrames > 0) {
            const float rampProgress =
                (float)(REACQUIRE_SPEED_RAMP_FRAMES -
                        reacquireSpeedFrames) /
                (float)(REACQUIRE_SPEED_RAMP_FRAMES - 1U);
            const float rampSpeed = (float)DROPOUT_FORWARD_PWM +
                (governedForwardSpeed - (float)DROPOUT_FORWARD_PWM) *
                    rampProgress;
            if (rampSpeed < base) base = rampSpeed;
            --reacquireSpeedFrames;
        }
        const float correctionLimit = base * PID_CORRECTION_BASE_MULTIPLIER;
        float correction = lastCorrection;
        if (correction > correctionLimit) correction = correctionLimit;
        if (correction < -correctionLimit) correction = -correctionLimit;

        // Preserve the current governed forward PWM for map-specific gaps.
        lastNormalForwardPwm = clampPwm(base);
        // Save correction for dashed-line coast
        lastDashedCorrection = correction;

        float leftCommand = base + correction;
        float rightCommand = base - correction;
        const float outerErrorRatio = fminf(
            fmaxf((absErr - OUTER_STEERING_START_ERROR) /
                  (OUTER_STEERING_FULL_ERROR - OUTER_STEERING_START_ERROR),
                  0.0f),
            1.0f);
        const float steeringSeverity = fmaxf(outerErrorRatio, edgeSeverity);
        const float steeringPeakLimit = base +
            ((float)pidConfig.maxSpeed - base) * steeringSeverity;
        limitMotorCommandPeak(leftCommand, rightCommand, steeringPeakLimit);
        motor_set_speeds(clampPwmSigned(leftCommand), clampPwmSigned(rightCommand));
    } else if (currentCase == LINE_CASE_CROSS_LINE) {
        clearPidState();
        const int16_t speed = clampPwm((float)pidConfig.baseSpeed * 0.7f);
        motor_set_speeds(speed, speed);
    } else if (currentCase == LINE_CASE_GAP_BRIDGE) {
        motor_set_speeds(gapHoldPwm, gapHoldPwm);
    } else if (currentCase == LINE_CASE_DASHED_LINE) {
        // Coast at elevated speed through brief line gaps (dashed line / intersection).
        // Use a heavily-damped correction so the robot goes mostly straight instead of
        // following the last turn direction, which causes sideways stutter.
        const float coastBase = fminf(governedForwardSpeed,
                                      (float)DROPOUT_FORWARD_PWM);
        const float coastCorrection = lastDashedCorrection * 0.3f; // was 0.7f — less sideways drift
        float leftCommand = coastBase + coastCorrection;
        float rightCommand = coastBase - coastCorrection;
        limitMotorCommandPeak(leftCommand, rightCommand, coastBase * 1.5f); // was coastBase — allow outer wheel to stay up
        motor_set_speeds(clampPwmSigned(leftCommand),
                         clampPwmSigned(rightCommand));
    } else if (currentCase == LINE_CASE_STARTUP_COAST) {
        const int16_t speed = pidConfig.baseSpeed < DROPOUT_FORWARD_PWM
            ? pidConfig.baseSpeed : DROPOUT_FORWARD_PWM;
        motor_set_speeds(speed, speed);
    } else {
        clearPidState();
        motor_stop(); // Safe stop after confirmed line loss or any sensor fault.
    }
}

LineCase_t pid_get_current_case(void) { return currentCase; }
bool pid_line_is_confirmed(void) { return lineLocked; }
bool pid_finish_line_latched(void) { return finishLineLatched; }
uint32_t pid_get_confirmed_line_loss_count(void) { return confirmedLosses; }
uint32_t pid_get_line_reacquisition_count(void) { return reacquisitions; }

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID: return "CASE 1: Bam Line PID";
        case LINE_CASE_SHARP_LEFT: return "CASE 2: Cua Gat Trai (90 deg)";
        case LINE_CASE_SHARP_RIGHT: return "CASE 3: Cua Gat Phai (90 deg)";
        case LINE_CASE_LOST_LINE: return "CASE 4: Mat Line - STOP";
        case LINE_CASE_CROSS_LINE: return "CASE 5: Vach Ngang / Nga Tu";
        case LINE_CASE_T_LEFT: return "CASE 6: Nga 3 Trai";
        case LINE_CASE_T_RIGHT: return "CASE 7: Nga 3 Phai";
        case LINE_CASE_STOPPED: return "CASE 8: Dung Xe";
        case LINE_CASE_DASHED_LINE: return "CASE 9: Pending loss - forward only";
        case LINE_CASE_STARTUP_COAST: return "CASE 10: Startup loss - straight";
        case LINE_CASE_GAP_BRIDGE: return "CASE 11: Map GAP bridge 100ms";
        default: return "Khong Xac Dinh";
    }
}
float pid_get_last_error(void) { return lastError; }
float pid_get_correction(void) { return lastCorrection; }
float pid_get_term_p(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastPTerm : 0.0f; }
float pid_get_term_i(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastITerm : 0.0f; }
float pid_get_term_d(void) { return currentCase == LINE_CASE_NORMAL_PID ? lastDTerm : 0.0f; }
float pid_get_kp(void) { return pidConfig.Kp; }
float pid_get_ki(void) { return pidConfig.Ki; }
float pid_get_kd(void) { return pidConfig.Kd; }
int16_t pid_get_base_speed(void) { return pidConfig.baseSpeed; }
