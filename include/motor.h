#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "pinout.h"

#ifdef __cplusplus
extern "C" {
#endif

// =================== CẤU HÌNH PWM LEDC ===================
#define MOTOR_PWM_FREQ        10000    // Tần số PWM 10000 Hz chuẩn cho driver động cơ (êm, lực kéo khỏe, không trượt xung)
#define MOTOR_PWM_RESOLUTION  8       // Độ phân giải PWM 8-bit (0 - 255)

#define CH_L1                 0       // Kênh LEDC cho IN_L1
#define CH_L2                 1       // Kênh LEDC cho IN_L2
#define CH_R1                 2       // Kênh LEDC cho IN_R1
#define CH_R2                 3       // Kênh LEDC cho IN_R2

// =================== CÁC HÀM ĐIỀU KHIỂN ĐỘNG CƠ ===================

/**
 * @brief Khởi tạo các kênh PWM LEDC và gắn chân điều khiển động cơ
 */
void motor_init(void);

/**
 * @brief Dừng cả hai động cơ ngay lập tức
 */
void motor_stop(void);

/**
 * @brief Điều khiển động cơ trái
 * @param speed Tốc độ từ -255 đến 255 (Dương: tiến, Âm: lùi, 0: dừng)
 */
void motor_set_left(int16_t speed);

/**
 * @brief Điều khiển động cơ phải
 * @param speed Tốc độ từ -255 đến 255 (Dương: tiến, Âm: lùi, 0: dừng)
 */
void motor_set_right(int16_t speed);

/**
 * @brief Đặt tốc độ đồng thời cho cả hai động cơ (rất tiện cho thuật toán dò line / PID)
 * @param speed_left Tốc độ động cơ trái (-255 đến 255)
 * @param speed_right Tốc độ động cơ phải (-255 đến 255)
 */
void motor_set_speeds(int16_t speed_left, int16_t speed_right);

/**
 * @brief Lấy tốc độ PWM hiện tại của động cơ trái
 * @return Giá trị PWM (-255 đến 255)
 */
int16_t motor_get_left_speed(void);

/**
 * @brief Lấy tốc độ PWM hiện tại của động cơ phải
 * @return Giá trị PWM (-255 đến 255)
 */
int16_t motor_get_right_speed(void);

/**
 * @brief Cả hai động cơ cùng tiến
 * @param speed Tốc độ PWM (0 - 255)
 */
void motor_forward(uint8_t speed);

/**
 * @brief Cả hai động cơ cùng lùi
 * @param speed Tốc độ PWM (0 - 255)
 */
void motor_backward(uint8_t speed);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
