#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include "pinout.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid_controller.h"
#include "bt_telemetry.h"
#include "perf_monitor.h"
#include "adc_bench.h"
#include "spp_debug.h"

namespace {
// 300 Hz * 72 B = 21.6 kB/s. USB debug fully disabled.
constexpr uint32_t BLINK_MS=400;
// 128 calibration frames take about 0.2s at ~650 Hz. Blink periods must be
// shorter than calibration acquisition, not hundreds of ms per ADC sample.
constexpr uint32_t CAL_WHITE_LED_BLINK_MS=40; // fast, GPIO2 toggle interval
constexpr uint32_t CAL_BLACK_LED_BLINK_MS=90; // slower, GPIO2 toggle interval
constexpr uint32_t MANUAL_TIMEOUT_MS=750; // prevents uncontrolled manual driving
// GPIO34 START accepts one or more black sensors among the six central
// eyes S5..S10. This widens the allowable placement around the line.
constexpr uint16_t BUTTON34_START_CENTER_MASK=0x07E0U;
bool sppStarted=false, btPrevious=false, manualActive=false, ledOn=false;
bool estopLatched=false; // Latching software E-STOP; only RESET_ESTOP can clear.
uint8_t manualSpeed=120;
uint16_t streamHz=300;
uint32_t streamPeriodUs=1000000UL/300;
uint32_t lastTxUs=0,lastDiagUs=0,lastBlink=0,lastManual=0;
uint32_t loopCount=0,pidCount=0,txCount=0,skippedSlots=0;
BtTxStats lastTxStats{};
// Each boot requires physical presses WHITE -> BLACK -> START.
// A successful START keeps the button usable for subsequent runs; calibration
// can also be restarted explicitly through CAL_WHITE via SPP.
enum Button34Phase : uint8_t {
  BUTTON34_WHITE_NEXT,
  BUTTON34_WHITE_CAPTURING,
  BUTTON34_BLACK_NEXT,
  BUTTON34_BLACK_CAPTURING,
  BUTTON34_START_NEXT,
  BUTTON34_START_REUSE
};
Button34Phase button34Phase=BUTTON34_WHITE_NEXT;
uint32_t lastButton34LedMs=0;

void ledWrite(bool on) {
  ledOn=on;
  digitalWrite(PIN_DEBUG_LED,on==(DEBUG_LED_ACTIVE_HIGH!=0)?HIGH:LOW);
}
void setButton34Phase(Button34Phase phase) {
  button34Phase=phase;
  lastButton34LedMs=millis();
  // Only illuminate the LED during an active calibration capture.
  // CAL_WHITE_DONE and CAL_OK_SAVED turn it OFF until the next press.
  ledWrite(phase==BUTTON34_WHITE_CAPTURING ||
           phase==BUTTON34_BLACK_CAPTURING);
}
void reply(const String &line) { btSendLine(line); }
void safeStop() {
  pid_set_auto_mode(false); motor_stop(); manualActive=false;
}
bool parseGain(const String &s,float &val) {
  if(!s.length())return false;
  char *end=nullptr;
  float v=strtof(s.c_str(),&end);
  if(end==s.c_str()||*end||!isfinite(v)||v<0||v>50)return false;
  val=v;return true;
}
bool parseSpeed(const String &s,int &val) {
  if(!s.length())return false;
  char *end=nullptr;
  long v=strtol(s.c_str(),&end,10);
  if(end==s.c_str()||*end||v<0||v>255)return false;
  val=(int)v;return true;
}
void showPid() {
  char b[100];
  snprintf(b,sizeof(b),"PID: Kp=%.2f Ki=%.2f Kd=%.2f Base=%d",
           pid_get_kp(),pid_get_ki(),pid_get_kd(),pid_get_base_speed());
  reply(b);
}
void showCalib() {
  reply(String("CAL_STATUS READY=")+(calibrationReady?"1":"0")+
        " STATE="+String((int)sensor_calibration_state()));
}
void command(const String &raw) {
  String s=raw;s.trim();s.toUpperCase();
  if(!s.length())return;
  // Highest-priority commands: E-STOP cuts PWM and prevents all new motion.
  if(s=="ESTOP"||s=="E_STOP"||s=="EMERGENCY_STOP"){
    estopLatched=true; safeStop();adcBenchCancel();
    reply("ESTOP: LATCHED");return;
  }
  if(s=="RESET_ESTOP"){
    safeStop();estopLatched=false; // Unlock only: motors remain stopped.
    reply("ESTOP: CLEARED (MOTORS OFF)");return;
  }
  // USB command echo intentionally disabled.
  if(s=="PING"){reply("PONG");return;}
  if(s=="?"||s=="HELP"){
    reply("HELP: PING STATUS DIAG PERF PID HS_PEAK HS_PEAK:100-255 TASKS ADC_BENCH ADC_BENCH_STOP ADC_BENCH_STATUS AUTO AUTO:1/2/3 MAP MAP:1/2/3 START STOP ESTOP RESET_ESTOP PWM ENC ENC_RESET P CAL_WHITE CAL_BLACK CAL_STATUS CAL_DUMP KP:x KI:x KD:x BASE:n STREAM:0/20/50/100/200/300 F B L R");
    return;
  }
  if(s=="STREAM"){
    reply(String("STREAM Hz=")+streamHz+" FRAME_BYTES=72");return;
  }
  if(s.startsWith("STREAM:")){
    const String textHz=s.substring(7);
    const int hz=textHz.toInt();
    const bool valid=(textHz==String(hz))&&
       (hz==0||hz==20||hz==50||hz==100||hz==200||hz==300);
    if(!valid){reply("ERR: STREAM_RATE_USE_0_20_50_100_200_300");return;}
    streamHz=(uint16_t)hz;
    streamPeriodUs=streamHz?1000000UL/streamHz:0;
    lastTxUs=micros();
    btDropPendingTelemetry();
    reply(String("STREAM Hz=")+streamHz+" FRAME_BYTES=72");return;
  }
  if(s=="DIAG"){
    const uint32_t t=micros(),dt=t-lastDiagUs;
    const float sec=dt?dt*1e-6f:1.0f;
    const BtTxStats tx=btGetTxStats();
    int32_t encL=0,encR=0;encoder_get_counts(&encL,&encR);
    char b[290];
    snprintf(b,sizeof(b),
      "DIAG: bt=%d auto=%d target=%u loop=%.1fHz pid=%.1fHz queued=%.1fHz sent=%.1fHz "
      "drop=%lu failed=%lu missed=%lu pending=%lu encL=%ld encR=%ld "
      "case=%u line=%u lost=%lu reacq=%lu map=%u runEnc=%ld hs=%u peak=%d",
      btConnected()?1:0,pid_is_auto_mode()?1:0,(unsigned)streamHz,
      loopCount/sec,pidCount/sec,
      (tx.queued-lastTxStats.queued)/sec,(tx.sent-lastTxStats.sent)/sec,
      (unsigned long)(tx.dropped-lastTxStats.dropped),
      (unsigned long)(tx.failed-lastTxStats.failed),
      (unsigned long)skippedSlots,(unsigned long)tx.pending,
      (long)encL,(long)encR,
      (unsigned)pid_get_current_case(),pid_line_is_confirmed()?1U:0U,
      (unsigned long)pid_get_confirmed_line_loss_count(),
      (unsigned long)pid_get_line_reacquisition_count(),
      (unsigned)pid_get_selected_map(),(long)pid_get_run_encoder_progress(),
      (unsigned)pid_get_high_speed_stage(),pid_get_high_speed_peak());
    lastDiagUs=t;loopCount=pidCount=txCount=0;
    lastTxStats=tx;skippedSlots=0;
    reply(b);return;
  }
  if(s=="PERF"){perfReport();return;}
  if(s=="TASKS"){perfTaskReport();return;}
  if(s=="ADC_BENCH"){
    if(sensor_is_calibrating()){reply("ERR: ADC_BENCH_CALIB_ACTIVE");return;}
    safeStop();
    reply(adcBenchStart()?"ADC_BENCH: STARTED (SPP RESULTS)":
                          "ERR: ADC_BENCH_BUSY_OR_INIT_FAILED");
    return;
  }
  if(s=="ADC_BENCH_STOP"){adcBenchCancel();reply("ADC_BENCH: STOPPED");return;}
  if(s=="ADC_BENCH_STATUS"){adcBenchStatus();return;}
  if(s=="PID"){showPid();return;}
  if(s=="HS_PEAK"){
    char b[160];
    snprintf(b,sizeof(b),
      "HS_PEAK: PWM=%d BASE=%d RAMP_END=300 BRAKE_START=%ld BRAKE_END=2300 HOLD_PWM=110 HOLD_MIN_END=3100",
      pid_get_high_speed_peak(),pid_get_base_speed(),
      (long)pid_get_high_speed_brake_start());
    reply(b);return;
  }
  if(s.startsWith("HS_PEAK:")){
    if(pid_is_auto_mode()||manualActive){
      reply("ERR: HS_PEAK_REQUIRES_STOPPED");return;
    }
    int value;
    const String rawPwm=s.substring(8);
    if(!parseSpeed(rawPwm,value)||value<100 ||
       !pid_set_high_speed_peak((int16_t)value)){
      reply("ERR: HS_PEAK_RANGE_100_TO_255_AND_GE_BASE");return;
    }
    command("HS_PEAK");return;
  }
  if(s=="PWM"){
    char b[110];snprintf(b,sizeof(b),"PWM: L=%d R=%d Err=%.2f Out=%.2f",
      motor_get_left_speed(),motor_get_right_speed(),
      pid_get_last_error(),pid_get_correction());
    reply(b);return;
  }
  if(s=="ENC"||s=="ENCODER"){
    int32_t l=0,r=0;
    encoder_get_counts(&l,&r);
    char b[92];
    snprintf(b,sizeof(b),"ENC: L=%ld R=%ld (signed x4)",(long)l,(long)r);
    reply(b);return;
  }
  if(s=="ENC_RESET"||s=="RESET_ENCODER"){
    encoder_reset_counts();
    reply("ENC_RESET: OK");return;
  }
  if(s=="STATUS"){
    char b[240];snprintf(b,sizeof(b),"STATUS: bt=%d auto=%d estop=%d calib=%d calibrating=%d case=%d pwmL=%d pwmR=%d adc=%s adc_ok=%d adc_err=%lu map=%u runEnc=%ld hs=%u peak=%d",
      btConnected()?1:0,pid_is_auto_mode()?1:0,estopLatched?1:0,
      calibrationReady?1:0,sensor_is_calibrating()?1:0,(int)pid_get_current_case(),
      motor_get_left_speed(),motor_get_right_speed(),sensor_adc_mode(),
      sensor_adc_ready()?1:0,(unsigned long)sensor_adc_error_count(),
      (unsigned)pid_get_selected_map(),(long)pid_get_run_encoder_progress(),
      (unsigned)pid_get_high_speed_stage(),pid_get_high_speed_peak());
    reply(b);return;
  }
  if(s=="P"||s=="SENSORS"){
    String b="SENSORS: ";
    for(int i=0;i<N_SENSORS;++i)b+=(sensor_is_black(i)?'1':'0');
    reply(b);return;
  }
  // Explicit track profile, independent of the Python CSV destination.
  if(s=="MAP"){
    int32_t gapBegin=0,gapEnd=0;
    pid_get_gap_window(&gapBegin,&gapEnd);
    char b[100];
    snprintf(b,sizeof(b),"MAP: SELECTED %u GAP=%ld-%ld TIMEOUT=100ms",
      (unsigned)pid_get_selected_map(),(long)gapBegin,(long)gapEnd);
    reply(b);return;
  }
  if(s.startsWith("MAP:")){
    const String idText=s.substring(4);
    const int id=idText.toInt();
    if(id<1||id>3||idText!=String(id)){
      reply("ERR: MAP_USE_1_2_3");return;
    }
    if(pid_is_auto_mode()||manualActive){
      reply("ERR: MAP_REQUIRES_STOPPED");return;
    }
    pid_select_map((uint8_t)id);
    command("MAP");return;
  }
  if(s=="S"||s=="STOP"||s=="M"||s=="MANUAL"){
    safeStop();adcBenchCancel();
    // Prioritize the stop acknowledgment over stale 100-Hz telemetry.
    btDropPendingTelemetry();
    reply(s=="M"||s=="MANUAL"?"MODE: MANUAL":"MOTOR: STOP");return;
  }
  if(s=="A"||s=="AUTO"||s=="START"||s.startsWith("AUTO:")){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(estopLatched){reply("ERR: ESTOP_LATCHED");return;}
    if(pid_is_auto_mode()){reply("ERR: AUTO_ALREADY_RUNNING");return;}
    if(!sensor_adc_ready()){reply("ERR: ADC_NOT_READY");return;}
    if(sensor_is_calibrating()||!calibrationReady||sensor_calibration_state()==CAL_WHITE_READY){reply("ERR: CALIB_REQUIRED_OR_PENDING");return;}
    // AUTO:1/2/3 sets the firmware track atomically with START, avoiding
    // a mismatch between the Python logging MAP and the active controller.
    if(s.startsWith("AUTO:")){
      const String idText=s.substring(5);
      const int id=idText.toInt();
      if(id<1||id>3||idText!=String(id)){
        reply("ERR: MAP_USE_1_2_3");return;
      }
      if(!pid_select_map((uint8_t)id)){
        reply("ERR: MAP_SELECT_FAILED");return;
      }
    }
    if(pid_get_selected_map()==0){
      reply("ERR: MAP_NOT_SELECTED");return;
    }
    manualActive=false;
    pid_set_auto_mode(true);
    reply("MODE: AUTO PID");
    int32_t runL=0,runR=0;
    pid_get_run_encoder_origins(&runL,&runR); // exactly the PID's AUTO origin
    char runInfo[180];
    snprintf(runInfo,sizeof(runInfo),
       "RUN_ORIGIN map=%u encL0=%ld encR0=%ld base=%d peak=%d kp=%.2f ki=%.2f kd=%.2f t_ms=%lu",
       (unsigned)pid_get_selected_map(),(long)runL,(long)runR,
       pid_get_base_speed(),pid_get_high_speed_peak(),
       pid_get_kp(),pid_get_ki(),pid_get_kd(),
       (unsigned long)millis());
    reply(runInfo);
    char rampEvent[160];
    snprintf(rampEvent,sizeof(rampEvent),
      "HS_STAGE RAMP map=%u runEnc=0 base=%d peak=%d brake=%ld end=2300 hold=110 minEnd=3100 t_ms=%lu",
      (unsigned)pid_get_selected_map(),pid_get_base_speed(),
      pid_get_high_speed_peak(),(long)pid_get_high_speed_brake_start(),
      (unsigned long)millis());
    reply(rampEvent);
    return;
  }
  // Fixed-polarity 16-channel calibration: WHITE, then BLACK.
  if(s=="C"||s=="W"||s=="CALIB"||s=="CAL_WHITE"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(sensor_is_calibrating()){reply("CAL_BUSY");return;}
    safeStop();
    if(sensor_capture_white()){
      setButton34Phase(BUTTON34_WHITE_CAPTURING);
      reply("CAL_WHITE_ACCEPTED");
    }else reply("CAL_BUSY");
    return;
  }
  if(s=="CAL_BLACK"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(sensor_calibration_state()!=CAL_WHITE_READY){reply("CAL_NEED_WHITE");return;}
    safeStop();
    if(sensor_capture_black()){
      setButton34Phase(BUTTON34_BLACK_CAPTURING);
      reply("CAL_BLACK_ACCEPTED");
    }else reply("CAL_BUSY");
    return;
  }
  if(s=="CAL_STATUS"){showCalib();return;}
  if(s=="CAL_DUMP"){
    showCalib();
    if(!calibrationReady){reply("CAL_NO_PROFILE");return;}
    reply("CAL_DUMP_BEGIN");
    for(int i=0;i<N_SENSORS;++i){
      char line[80];
      snprintf(line,sizeof(line),"CAL_DATA %d %u %u %u",
          i,minValues[i],maxValues[i],thresholds[i]);
      reply(line);
    }
    reply("CAL_DUMP_DONE");return;
  }
  if(s=="K"||s=="STOP_CALIB"||s=="SWEEP"){reply("CAL_USE_WHITE_THEN_BLACK");return;}
  if(s=="INV"||s=="INVERT"){reply("CAL_FIXED_POLARITY BLACK_ADC_HIGH");return;}
  if(s.startsWith("KP:")||s.startsWith("KI:")||s.startsWith("KD:")){
    float v;
    if(!parseGain(s.substring(3),v)){reply("ERR: PID_VALUE_0_50");return;}
    float kp=pid_get_kp(),ki=pid_get_ki(),kd=pid_get_kd();
    if(s.startsWith("KP:"))kp=v;
    if(s.startsWith("KI:"))ki=v;
    if(s.startsWith("KD:"))kd=v;
    pid_set_tunings(kp,ki,kd);showPid();return;
  }
  if(s.startsWith("V:")||s.startsWith("BASE:")||s.startsWith("SPEED:")){
    int v;int sep=s.indexOf(':');
    if(sep<0||!parseSpeed(s.substring(sep+1),v)){reply("ERR: SPEED_0_255");return;}
    // A BASE above HS_PEAK makes the trapezoid undefined. Reject instead
    // of silently turning a supposedly lower peak into a higher target.
    if(v>pid_get_high_speed_peak()){reply("ERR: BASE_GT_HS_PEAK");return;}
    pid_set_base_speed(v);manualSpeed=(uint8_t)v;
    reply(String("SPEED SET: ")+v);return;
  }
  if(s=="F"||s=="B"||s=="L"||s=="R"){
    if(adcBenchActive()){reply("ERR: ADC_BENCH_ACTIVE");return;}
    if(estopLatched){reply("ERR: ESTOP_LATCHED");return;}
    if(!sensor_adc_ready()){reply("ERR: ADC_NOT_READY");return;}
    safeStop();
    if(s=="F")motor_forward(manualSpeed);
    if(s=="B")motor_backward(manualSpeed);
    if(s=="L")motor_set_speeds(-(int16_t)manualSpeed*2/3,manualSpeed);
    if(s=="R")motor_set_speeds(manualSpeed,-(int16_t)manualSpeed*2/3);
    manualActive=true;lastManual=millis();
    reply(String("MOTOR: ")+s+" (deadman 750ms)");return;
  }
  reply(String("ERR: UNKNOWN_COMMAND ")+s);
}
void connectionState(uint32_t now) {
  bool connected=sppStarted&&btConnected();
  if(connected!=btPrevious){
    if(connected){
      ledWrite(true);reply("[BT] CONNECTED: Windows SPP");
      reply("READY: LineRobot SPP v1 (72-byte frame)");showPid();
      command("HS_PEAK");
    }else{
      safeStop();adcBenchCancel();ledWrite(false);lastBlink=now;
      btDropPendingTelemetry(); // no replay after reconnect
    }
    btPrevious=connected;
  }
}
// Calibration indicator has priority over the Bluetooth LED:
// blink only while capturing; OFF after each completed calibration step.
void updateLedIndicator(uint32_t now) {
  const uint32_t blinkMs =
    button34Phase==BUTTON34_WHITE_CAPTURING ? CAL_WHITE_LED_BLINK_MS :
    button34Phase==BUTTON34_BLACK_CAPTURING ? CAL_BLACK_LED_BLINK_MS : 0U;
  if(blinkMs) {
    if((uint32_t)(now-lastButton34LedMs)>=blinkMs) {
      lastButton34LedMs=now;
      ledWrite(!ledOn);
    }
    return;
  }
  // Do not let a connected Bluetooth SPP session relight GPIO2 while
  // waiting for CAL_WHITE, CAL_BLACK or START.
  if(button34Phase!=BUTTON34_START_REUSE){
    if(ledOn) ledWrite(false);
    return;
  }
  if(sppStarted && btConnected()){
    if(!ledOn)ledWrite(true);
  }else if((uint32_t)(now-lastBlink)>=BLINK_MS){
    lastBlink=now;
    ledWrite(!ledOn);
  }
}
void sendFrame(){
  if(!sppStarted||!btConnected())return;
  Telemetry t{};
  // Timestamp belongs to the single post-PID snapshot, not the Bluetooth write.
  t.t_ms=millis();
  for(int i=0;i<N_SENSORS;++i)t.adc[i]=sensorRaw[i];
  int32_t l=0,r=0;
  encoder_get_counts(&l,&r);
  t.encL=l;t.encR=r;
  t.pwmL=motor_get_left_speed();t.pwmR=motor_get_right_speed();
  t.err=pid_is_auto_mode()?pid_get_last_error():0.0f;
  t.p=pid_get_term_p();t.i=pid_get_term_i();t.d=pid_get_term_d();
  t.out=pid_is_auto_mode()?pid_get_correction():0.0f;
  t.mode=pid_is_auto_mode()?1:0;
  if(btSendTelemetry(t))++txCount;
}
} // namespace

void setup() {
  // USB Serial disabled. Debug/commands exclusively via Bluetooth SPP.
  pinMode(PIN_DEBUG_LED,OUTPUT);ledWrite(false);
  sensor_init();motor_init();
  encoder_init(); // mainv2 signed quadrature x4 on A and B channels
  pid_init(1.6f,0.0f,0.8f,100); // default PID gains for MAP 1/2/3
  pid_select_map(1); // physical GPIO34 START works without a connected app
  sppStarted=btBegin("LineRobot");
  lastTxUs=lastDiagUs=micros();
  lastTxStats=btGetTxStats();
  lastBlink=millis();
}
void loop() {
  const uint32_t loopStartUs=micros();
  perfLoopBegin(loopStartUs);
  uint32_t now=millis();++loopCount;
  uint32_t stageStartUs=micros();
  connectionState(now);
  if(sppStarted)btPollCommands(command);
  perfRecord(PERF_COMM, micros()-stageStartUs);
  // GPIO34 physical 3-press workflow: WHITE -> BLACK -> START.
  // GPIO34 itself is debounced once per LOW press in sensor.cpp.
  if(sensor_start_button_pressed()){
    if(pid_is_auto_mode() || manualActive){
      reply("ERR: BUTTON34_REQUIRES_STOPPED");
    }else if(adcBenchActive()){
      reply("ERR: ADC_BENCH_ACTIVE");
    }else if(button34Phase==BUTTON34_WHITE_NEXT){
      reply("BUTTON34: CAL_WHITE_REQUEST");
      command("CAL_WHITE"); // changes phase only on accepted capture
    }else if(button34Phase==BUTTON34_WHITE_CAPTURING){
      reply("ERR: BUTTON34_WAIT_CAL_WHITE_DONE");
    }else if(button34Phase==BUTTON34_BLACK_NEXT){
      if(sensor_is_calibrating() ||
         sensor_calibration_state()!=CAL_WHITE_READY){
        reply("ERR: BUTTON34_WAIT_CAL_WHITE_DONE");
      }else{
        reply("BUTTON34: CAL_BLACK_REQUEST");
        command("CAL_BLACK");
      }
    }else if(button34Phase==BUTTON34_BLACK_CAPTURING){
      reply("ERR: BUTTON34_WAIT_CAL_BLACK_DONE");
    }else{
      // Preserve previous placement rule: any of central S5..S10 is enough,
      // but full 16/16 black indicates bad calibration / instant finish.
      if(calibrationReady && !sensor_is_calibrating() && sensor_adc_ready()){
        const uint16_t bits=sensor_get_line_bits();
        if(bits==0xFFFFU){
          reply("ERR: BUTTON34_ALL_BLACK_16_CHECK_CALIBRATION");
        }else if((bits & BUTTON34_START_CENTER_MASK)==0U){
          reply("ERR: BUTTON34_NO_CENTER_LINE_S5_TO_S10");
        }else{
          reply("BUTTON34: START_REQUEST");
          command("AUTO");
          if(pid_is_auto_mode()) setButton34Phase(BUTTON34_START_REUSE);
        }
      }else{
        // AUTO command keeps the existing ADC/CAL/ESTOP guards.
        command("AUTO");
      }
    }
  }
  const bool wasCalibrating=sensor_is_calibrating();
  stageStartUs=micros();
  if(adcBenchActive())adcBenchStep();
  else if(wasCalibrating)sensor_update_calibration_loop();
  else sensor_read_all();
  perfRecord(PERF_ADC, micros()-stageStartUs);
  if(!sensor_adc_ready() && (pid_is_auto_mode() || manualActive)){
    safeStop();
    reply("ERR: ADC_READ_FAULT_MOTORS_STOPPED");
  }
  switch(sensor_take_calibration_event()){
    case CAL_WHITE_STARTED: reply("CAL_WHITE_STARTED"); break;
    case CAL_WHITE_DONE:
      setButton34Phase(BUTTON34_BLACK_NEXT); // LED OFF, ready for BLACK press
      reply("CAL_WHITE_DONE");break;
    case CAL_BLACK_STARTED: reply("CAL_BLACK_STARTED"); break;
    case CAL_SAVED:
      setButton34Phase(BUTTON34_START_NEXT);
      reply("CAL_OK_SAVED");showCalib();break;
    case CAL_ERROR:
      // An invalid black profile cannot advance to START. Redo WHITE first.
      setButton34Phase(BUTTON34_WHITE_NEXT);
      reply("CAL_FAILED");showCalib();break;
    default: break;
  }
  updateLedIndicator(now);
  if(pid_is_auto_mode()&&!sensor_is_calibrating()){
    stageStartUs=micros();
    const LineCase_t oldCase=pid_get_current_case();
    const uint8_t oldHighSpeedStage=pid_get_high_speed_stage();
    pid_line_follow_step();++pidCount;
    perfRecord(PERF_PID, micros()-stageStartUs);
    const LineCase_t newCase=pid_get_current_case();
    const uint8_t newHighSpeedStage=pid_get_high_speed_stage();
    // A latched finish or PID stop must end AUTO, otherwise Python remains
    // in AUTO while both PWM outputs are zero and START stays locked.
    // Snapshot the stop reason BEFORE safeStop() clears PID state.
    if(newCase==LINE_CASE_STOPPED){
      const bool fullBlackFinish=pid_finish_line_latched();
      const uint8_t stopMap=pid_get_selected_map();
      const int32_t stopEnc=pid_get_run_encoder_progress();
      safeStop();
      // Drop queued pre-stop telemetry before announcing terminal state.
      btDropPendingTelemetry();
      char stopEvent[128];
      snprintf(stopEvent,sizeof(stopEvent),
        "%s map=%u runEnc=%ld t_ms=%lu",
        fullBlackFinish ? "AUTO_FINISHED: FULL_BLACK_3" :
                          "AUTO_STOPPED: PID_STOPPED",
        (unsigned)stopMap,(long)stopEnc,(unsigned long)millis());
      reply(stopEvent);
    }
    if(newHighSpeedStage!=oldHighSpeedStage && newHighSpeedStage!=0U){
      static const char* const labels[]={
        "OFF","RAMP","CRUISE","BRAKE","COMPLETE","ABORTED","HOLD"
      };
      if(newHighSpeedStage<7U){
        char b[160];
        snprintf(b,sizeof(b),
          "HS_STAGE %s map=%u runEnc=%ld base=%d peak=%d brake=%ld end=2300 hold=110 minEnd=3100 t_ms=%lu",
          labels[newHighSpeedStage],(unsigned)pid_get_selected_map(),
          (long)pid_get_run_encoder_progress(),pid_get_base_speed(),
          pid_get_high_speed_peak(),(long)pid_get_high_speed_brake_start(),
          (unsigned long)millis());
        reply(b);
      }
    }
    // Only log transitions, never print from the high-frequency PID controller.
    // The Python app already mirrors text replies into the MAP-specific TXT.
    if(oldCase!=newCase &&
       (oldCase==LINE_CASE_GAP_BRIDGE || newCase==LINE_CASE_GAP_BRIDGE)){
      const bool entered=newCase==LINE_CASE_GAP_BRIDGE;
      const char* event=entered ? "GAP_START" :
        (newCase==LINE_CASE_SHARP_LEFT || newCase==LINE_CASE_SHARP_RIGHT
            ? "GAP_TIMEOUT_SEARCH" : "GAP_RECOVERED");
      char b[150];
      snprintf(b,sizeof(b),
        "%s map=%u runEnc=%ld pwmL=%d pwmR=%d t_ms=%lu",
        event,(unsigned)pid_get_selected_map(),
        (long)pid_get_run_encoder_progress(),
        motor_get_left_speed(),motor_get_right_speed(),
        (unsigned long)millis());
      reply(b);
    }
  }
  if(manualActive&&now-lastManual>=MANUAL_TIMEOUT_MS){
    motor_stop();manualActive=false;reply("[SAFE] MANUAL DEADMAN: MOTOR STOP");
  }
  // No stale bursts; bounded nonblocking enqueue at the selected stream rate.
  const uint32_t txNow=micros();
  if(streamPeriodUs&&!adcBenchActive()){
    const uint32_t elapsed=txNow-lastTxUs;
    if(elapsed>=streamPeriodUs){
      const uint32_t slots=elapsed/streamPeriodUs;
      lastTxUs+=slots*streamPeriodUs;
      if(slots>1)skippedSlots+=slots-1;
      if(sppStarted&&btConnected()){
        stageStartUs=micros();
        sendFrame();
        perfRecord(PERF_TX,micros()-stageStartUs);
      }
    }
  }
  perfLoopEnd(micros()-loopStartUs);
  // Loop start-to-start time includes yield and background scheduler work.
  yield();
}
