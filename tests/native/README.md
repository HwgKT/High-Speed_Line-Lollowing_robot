# Native PID stability smoke test

Compile the **real** firmware PID controller using a host Arduino clock stub,
and mock only the hardware-facing ADC/motor I/O:

~~~bash
g++ -std=c++17 -Wall -Wextra -Itests/native -Iinclude \
  src/pid_controller.cpp tests/native/pid_stability_smoke.cpp \
  -o pid_stability_smoke
./pid_stability_smoke
~~~

Requires a desktop C++17 compiler (GCC/Clang); no board or PlatformIO required.

Test coverage:
- Raw calibration threshold TH_i=1400 is respected, CAL=350 is not a black line.
- No startup movement without 10 ms stable black detection.
- If an already acquired line is lost during the first five seconds of AUTO,
  drive straight at no more than PWM 65 until the line returns or the grace
  window expires; an initial start with no line remains stopped.
- Brief no-black interval coasts with damped PID correction, then a confirmed
  40-ms loss starts an in-place pivot search at PWM 85 per wheel (170 PWM
  differential), bounded by the existing two-second timeout.
- Pivot search hands control back after three consecutive reliable samples:
  either a centered sensor or an adjacent sensor pair anywhere on the array.
  An isolated far-edge sensor does not cancel the search. It safely stops after
  two seconds if no line is found.
- Sharp left/right line detections stay under PID control. The inner wheel may
  reverse. At outer errors above 20, the peak PID wheel limit ramps from the
  governed speed to `maxSpeed` by error 60, allowing stronger counter-steering
  while remaining within the configured motor limit.
- Detection by the outer four sensors per side reduces the PID forward target
  by up to 40% before the centroid governor reaches its maximum reduction;
  steering authority still scales with both outer-sensor position and error.
- After search reacquisition, derivative output is held at zero for four PID
  samples and the forward target ramps from PWM 65 over four samples to soften
  the handoff without delaying steering for an off-center line.
- A brief line gap keeps the previous forward arc for the 40-ms loss debounce;
  the arc uses its sharp-turn PWM profile without the governor cap, while
  persistent loss falls back to the existing bounded pivot search.
- Negative PWM is permitted during line-loss turns/search, damped dashed-line
  correction, and signed PID steering on a visible corner.
- PID remains active while a turn is visible; the turn latch releases after
  two centered samples.
- Normal PID target speed follows line error directly: higher error lowers
  the target. Near center, wheel commands stay within the governed speed;
  larger outer errors can use the stronger bounded steering envelope. The
  governor reaches its full reduction at error 30; the sharp-turn arc uses its
  own PWM profile. There is no timed post-reacquisition acceleration ramp.
- A wide black intersection (including 15/16 black sensors) drives both motors
  forward. The finish stop is latched only after all 16 sensors detect black
  for 3 consecutive PID samples; any intervening non-full sample resets the
  counter to zero. This is sample-based, not a 50-ms timer.
- Stopping AUTO stops both motors immediately.

This **does not validate actual ESP32 pin wiring, motor polarity, timing under
RTOS load, Bluetooth SPP delivery or the compiled full firmware**. Perform the
real hardware acceptance test with raised wheels and then a straight black line.
