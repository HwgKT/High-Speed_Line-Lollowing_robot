// Compile on a PC without any ESP32 board:
// g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude src/pid_controller.cpp tests/native/pid_stability_smoke.cpp -o pid_stability_smoke
// ./pid_stability_smoke
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"

static uint32_t clockUs=0;
uint32_t millis(void) { return clockUs/1000; }
uint32_t micros(void) { return clockUs; }

extern "C" {
uint16_t sensorRaw[N_SENSORS]={};
uint16_t minValues[N_SENSORS]={};
uint16_t maxValues[N_SENSORS]={};
uint16_t thresholds[N_SENSORS]={};
bool calibrationReady=true;
bool isCalibrating=false;
// Linked PID controller now reads an encoder snapshot at each AUTO start.
void encoder_get_counts(int32_t* l, int32_t* r) { if(l)*l=0; if(r)*r=0; }

bool sensor_adc_ready(void) {return true;}
bool sensor_is_black(uint8_t i) {
    return calibrationReady && i<N_SENSORS && sensorRaw[i]>=thresholds[i];
}
uint16_t sensor_get_calibrated(uint8_t i) {
    if (!calibrationReady || i>=N_SENSORS) return 0;
    const uint16_t x=sensorRaw[i], w=minValues[i], b=maxValues[i];
    if (x<=w) return 0;
    if (x>=b) return 1000;
    return (uint32_t)(x-w)*1000U/(b-w);
}
uint16_t sensor_get_line_bits(void) {
    uint16_t mask=0;
    for(uint8_t i=0;i<N_SENSORS;i++)
        if(sensor_is_black(i))mask|=(uint16_t)(1U<<i);
    return mask;
}
static int16_t leftPwm=0,rightPwm=0;
void motor_set_speeds(int16_t left,int16_t right) {
    assert(left>=-220 && right>=-220);
    assert(left<=220 && right<=220);
    if(left<0 || right<0) {
        const LineCase_t current=pid_get_current_case();
        assert(current==LINE_CASE_SHARP_LEFT || current==LINE_CASE_SHARP_RIGHT ||
               current==LINE_CASE_DASHED_LINE || current==LINE_CASE_NORMAL_PID);
        if(current==LINE_CASE_SHARP_LEFT) assert(left<0 && right>0);
        if(current==LINE_CASE_SHARP_RIGHT) assert(left>0 && right<0);
    }
    leftPwm=left;rightPwm=right;
}
void motor_stop(void) {leftPwm=rightPwm=0;}
}

static void stepMs(unsigned ms) {
    for(unsigned elapsed=0;elapsed<ms;elapsed+=2) {
        clockUs+=2000;
        pid_line_follow_step();
    }
}
static void allWhite(void) {
    for(int i=0;i<N_SENSORS;i++)sensorRaw[i]=1000;
}
static void onlyEye(unsigned eye, uint16_t raw) {
    allWhite();
    sensorRaw[eye]=raw;
}
static void setPattern(const char *bits) {
    for(unsigned i=0;i<N_SENSORS;i++)
        sensorRaw[i]=bits[i]=='1'?2000:1000;
}
int main(void) {
    for(int i=0;i<N_SENSORS;i++){
        minValues[i]=1000;maxValues[i]=2000;thresholds[i]=1400;
    }
    pid_init(2.2f,0,0,120);
    pid_set_auto_mode(true);
    pid_line_follow_step();
    assert(leftPwm==0 && rightPwm==0);

    // 1) Analog CAL=350, logic=0: must NOT start or count as a line.
    onlyEye(8,1350);
    stepMs(20);
    assert(!pid_line_is_confirmed());
    assert(leftPwm==0 && rightPwm==0);

    // 2) Valid black must persist for >=10ms before PID drives.
    onlyEye(8,1700);
    stepMs(8);
    assert(leftPwm==0 && rightPwm==0);
    stepMs(6);
    assert(pid_line_is_confirmed());
    assert(leftPwm>0 && rightPwm>=0);

    // Startup loss stays straight at a bounded speed, then recovers on line.
    allWhite();
    stepMs(60);
    assert(pid_get_current_case()==LINE_CASE_STARTUP_COAST);
    assert(leftPwm==65 && rightPwm==65);
    onlyEye(8,1700);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(pid_line_is_confirmed());

    // Outside the startup grace window, a brief missed sensor run coasts
    // with the damped PID correction before the existing pivot search.
    stepMs(5000);
    allWhite();
    stepMs(10);
    assert(pid_get_current_case()==LINE_CASE_DASHED_LINE);
    assert(leftPwm>0 && rightPwm>0);
    stepMs(32);
    assert(!pid_line_is_confirmed());
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    assert(pid_get_confirmed_line_loss_count()==1);

    // 4) Pivot search requires stable line evidence, then soft-starts PID.
    pid_set_tunings(2.2f,0.0f,2.0f);
    onlyEye(8,1600);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(pid_line_is_confirmed());
    assert(pid_get_line_reacquisition_count()==1);
    assert(leftPwm>0 && rightPwm>0 && leftPwm<=65 && rightPwm<=65);
    assert(pid_get_term_d()==0.0f);
    setPattern("0000000000110000");
    stepMs(6);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(pid_get_term_d()==0.0f);
    pid_set_tunings(2.2f,0.0f,0.0f);

    // Visible corners stay under PID control instead of using a fixed arc.
    setPattern("0000001000001100");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm>rightPwm);
    onlyEye(8,1600);
    stepMs(4);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);

    // Outermost-sensor errors allow full PID steering authority up to maxSpeed.
    onlyEye(15,2000);
    stepMs(14);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm>0 && rightPwm<0);
    assert(leftPwm==220 && rightPwm>=-220 && rightPwm<0);
    assert(rightPwm<=-150);
    assert(leftPwm-rightPwm>=370);

    // Keep the arc through a brief line gap; then pivot to search.
    allWhite();
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm>rightPwm && rightPwm>0);
    assert(leftPwm>66 && leftPwm<=185);
    stepMs(620);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    onlyEye(15,2000);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    assert(leftPwm==85 && rightPwm==-85);
    setPattern("0000000000000011");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_RIGHT);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(pid_line_is_confirmed());
    assert(leftPwm>0 && rightPwm<0);
    onlyEye(8,1700);
    stepMs(4);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);

    // A wide junction can be seen while PID tracks a sharp turn.
    onlyEye(0,2000);
    stepMs(14);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(rightPwm>leftPwm && leftPwm<0);
    allWhite();
    stepMs(350);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    setPattern("1101111111110000");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    setPattern("0111111111111100");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==84 && rightPwm==84);

    // A pivot search that finds no line stops safely after two seconds.
    onlyEye(0,2000);
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(rightPwm>leftPwm && leftPwm<0);
    allWhite();
    stepMs(44);
    assert(pid_get_current_case()==LINE_CASE_SHARP_LEFT);
    assert(leftPwm==-85 && rightPwm==85);
    stepMs(2010);
    assert(pid_get_current_case()==LINE_CASE_LOST_LINE);
    assert(leftPwm==0 && rightPwm==0);

    // The speed governor follows line error directly, without a timed
    // post-reacquisition acceleration ramp.
    pid_set_auto_mode(false);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    const int16_t centeredPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    assert(centeredPwm==120);

    onlyEye(10,1700);
    stepMs(20);
    const int16_t cornerPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    assert(cornerPwm < centeredPwm && cornerPwm >= 80 && cornerPwm <= 95);

    setPattern("0000000110000000");
    stepMs(10);
    const int16_t recoveringPwm = leftPwm > rightPwm ? leftPwm : rightPwm;
    assert(recoveringPwm==centeredPwm);

    // A wide crossing with two white sensors is not a finish marker.
    setPattern("1111111111111100");
    stepMs(50);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==84 && rightPwm==84);

    // 15/16 black is a wide crossing, NOT a finish marker (even for >3 frames).
    setPattern("1111111111111110");
    stepMs(10);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==84 && rightPwm==84);

    // One full-black sample cannot stop; any non-full sample resets the count.
    setPattern("1111111111111111");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    setPattern("1111111111111110");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);

    // Two consecutive 16/16 black samples are not enough.
    setPattern("1111111111111111");
    stepMs(4);
    assert(pid_get_current_case()==LINE_CASE_CROSS_LINE);
    assert(leftPwm==84 && rightPwm==84);

    // The third consecutive full-black PID sample latches the finish stop.
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_STOPPED);
    assert(leftPwm==0 && rightPwm==0);
    onlyEye(8,1700);
    stepMs(20);
    assert(pid_get_current_case()==LINE_CASE_STOPPED);
    assert(leftPwm==0 && rightPwm==0);

    // Outer sensors reduce forward speed before centroid error reaches the
    // normal governor limit.
    pid_set_auto_mode(false);
    pid_init(0.0f,0.0f,0.0f,120);
    setPattern("0000000110000000");
    pid_set_auto_mode(true);
    stepMs(12);
    assert(leftPwm==120 && rightPwm==120);
    setPattern("0000000110000001");
    stepMs(2);
    assert(pid_get_current_case()==LINE_CASE_NORMAL_PID);
    assert(leftPwm==72 && rightPwm==72);

    // 6) Disabling AUTO immediately stops both motors.
    pid_set_auto_mode(false);
    assert(leftPwm==0 && rightPwm==0);
    puts("PASS: line acquire/loss, startup straight grace, PID corner tracking, reverse-wheel steering, pivot search, speed governor and finish stop");
    return 0;
}
