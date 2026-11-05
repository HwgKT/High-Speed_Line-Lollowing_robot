#include "motor.h"
#include <Arduino.h>

static int16_t currentLeftSpeed = 0;
static int16_t currentRightSpeed = 0;

void motor_init(void) {
    // Cài đặt tần số và độ phân giải cho 4 kênh PWM
    ledcSetup(CH_L1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_L2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_R1, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(CH_R2, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);

    // Gán chân điều khiển vào các kênh PWM tương ứng
    ledcAttachPin(PIN_MOTOR_L1, CH_L1);
    ledcAttachPin(PIN_MOTOR_L2, CH_L2);
    ledcAttachPin(PIN_MOTOR_R1, CH_R1);
    ledcAttachPin(PIN_MOTOR_R2, CH_R2);

    // Mặc định dừng cả 2 động cơ khi khởi động
    motor_stop();
}

void motor_stop(void) {
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
    ledcWrite(CH_L1, 0);
    ledcWrite(CH_L2, 0);
    ledcWrite(CH_R1, 0);
    ledcWrite(CH_R2, 0);
}

void motor_set_left(int16_t speed) {
    // Giới hạn tốc độ trong khoảng [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    currentLeftSpeed = speed;

    if (speed > 0) {
        // Động cơ trái TIẾN
        ledcWrite(CH_L1, (uint32_t)speed);
        ledcWrite(CH_L2, 0);
    } else if (speed < 0) {
        // Động cơ trái LÙI
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, (uint32_t)(-speed));
    } else {
        // Động cơ trái DỪNG
        ledcWrite(CH_L1, 0);
        ledcWrite(CH_L2, 0);
    }
}

void motor_set_right(int16_t speed) {
    // Giới hạn tốc độ trong khoảng [-255, 255]
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;
    currentRightSpeed = speed;

    if (speed > 0) {
        // Động cơ phải TIẾN
        ledcWrite(CH_R1, (uint32_t)speed);
        ledcWrite(CH_R2, 0);
    } else if (speed < 0) {
        // Động cơ phải LÙI
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, (uint32_t)(-speed));
    } else {
        // Động cơ phải DỪNG
        ledcWrite(CH_R1, 0);
        ledcWrite(CH_R2, 0);
    }
}

void motor_set_speeds(int16_t speed_left, int16_t speed_right) {
    motor_set_left(speed_left);
    motor_set_right(speed_right);
}

int16_t motor_get_left_speed(void) {
    return currentLeftSpeed;
}

int16_t motor_get_right_speed(void) {
    return currentRightSpeed;
}

void motor_forward(uint8_t speed) {
    motor_set_speeds(speed, speed);
}

void motor_backward(uint8_t speed) {
    motor_set_speeds(-(int16_t)speed, -(int16_t)speed);
}
