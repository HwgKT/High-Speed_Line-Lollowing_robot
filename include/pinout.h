#ifndef PINOUT_H
#define PINOUT_H

// Line sensor CD74HC4067 and GPIO34 WHITE/BLACK/START button.
#define PIN_LINE_SIG       36
#define PIN_MUX_S0         17
#define PIN_MUX_S1         18
#define PIN_MUX_S2         19
#define PIN_MUX_S3         23
// GPIO34 is input-only, with NO internal pull-up/down: install an external
// pullup to 3.3V, button from GPIO34 to GND (active LOW).
#define PIN_START_BUTTON   34
#define PIN_UNUSED_BUTTON  35  // Not used by this firmware.

// Quadrature channels A/B per wheel, inherited from mainv2.
#define PIN_ENCODER_L1     21
#define PIN_ENCODER_L2     22
#define PIN_ENCODER_R1     32
#define PIN_ENCODER_R2     33
// Forward motion should increase both signed encoder counts.
// Flip either sign if wheel counts backward with wheels driven forward.
#define ENCODER_LEFT_SIGN  1
#define ENCODER_RIGHT_SIGN -1

// Motor driver inputs.
#define PIN_MOTOR_L1       25
#define PIN_MOTOR_L2       26
#define PIN_MOTOR_R1       27
#define PIN_MOTOR_R2       13

#define PIN_DEBUG_LED 2
#define DEBUG_LED_ACTIVE_HIGH 1

#endif // PINOUT_H
