#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =================== CÁC TRƯỜNG HỢP DÒ LINE (CASE STATES) ===================
typedef enum

{ LINE_CASE_NORMAL_PID = 1, // Case 1: Dò line chuẩn theo thuật toán PID (đường
                            // thẳng & cua thoai thoải)
  LINE_CASE_SHARP_LEFT =
      2, // Case 2: Cua gắt trái (90 độ hoặc lệch trái cực lớn)
  LINE_CASE_SHARP_RIGHT =
      3, // Case 3: Cua gắt phải (90 độ hoặc lệch phải cực lớn)
  LINE_CASE_LOST_LINE = 4, // Case 4: Mất line (tất cả 16 mắt đều trắng)
  LINE_CASE_CROSS_LINE =
      5, // Case 5: Vạch ngang / ngã tư / vạch đích (toàn bộ hoặc đa số mắt đen)
  LINE_CASE_T_LEFT = 6,     // Case 6: Ngã 3 chữ T bên trái
  LINE_CASE_T_RIGHT = 7,    // Case 7: Ngã 3 chữ T bên phải
  LINE_CASE_STOPPED = 8,    // Case 8: Dừng xe
  LINE_CASE_DASHED_LINE = 9, // Case 9: Đoạn nét đứt (vượt qua khoảng trắng)
  LINE_CASE_STARTUP_COAST = 10, // Case 10: Khởi động mất line, chạy thẳng có giới hạn
  LINE_CASE_GAP_BRIDGE = 11 // Case 11: Qua line dut theo MAP, toi da 100ms
} LineCase_t;

// Cấu trúc chứa thông số PID
typedef struct {
  float Kp;
  float Ki;
  float Kd;
  int16_t baseSpeed; // Tốc độ chuẩn (ví dụ 160)
  int16_t maxSpeed;  // Tốc độ tối đa (255)
  int16_t minSpeed;  // Legacy field; normal PID may issue signed wheel commands
} PID_Config_t;

// =================== CÁC HÀM KHỞI TẠO & CÀI ĐẶT ===================

/**
 * @brief Khởi tạo thông số PID và cấu hình tốc độ
 */
void pid_init(float kp, float ki, float kd, int16_t baseSpeed);

/**
 * @brief Cập nhật hệ số Kp, Ki, Kd
 */
void pid_set_tunings(float kp, float ki, float kd);

/**
 * @brief Cập nhật tốc độ cơ bản (base speed)
 */
void pid_set_base_speed(int16_t speed);

/**
 * @brief Bật hoặc tắt chế độ tự động dò line
 */
void pid_set_auto_mode(bool enable);

/**
 * @brief Kiểm tra xem đang chạy tự động dò line hay lái tay
 */
bool pid_is_auto_mode(void);
// Set track profile before AUTO. Valid MAP values: 1, 2, 3.
bool pid_select_map(uint8_t mapId);
uint8_t pid_get_selected_map(void);
// High-speed stage: 0 disabled, 1 ramp, 2 cruise, 3 brake,
// 4 complete after first right corner recovery, 5 aborted, 6 corner HOLD.
uint8_t pid_get_high_speed_stage(void);
// Adjustable peak PWM before START (100-255), must not be below BASE.
bool pid_set_high_speed_peak(int16_t pwm);
int16_t pid_get_high_speed_peak(void);
// Dynamic brake start: 1800 for peak 100..229; 1600 for 230..239; 1400 for 240..255.
// Encoder ticks are relative to the current AUTO origin on all MAPs.
int32_t pid_get_high_speed_brake_start(void);
// Signed average encoder progress since AUTO, without resetting global counts.
int32_t pid_get_run_encoder_progress(void);
void pid_get_run_encoder_origins(int32_t* left, int32_t* right);
void pid_get_gap_window(int32_t* begin, int32_t* end);
bool pid_line_is_confirmed(void);
// True only after 16 black sensors for 3 consecutive PID cycles.
bool pid_finish_line_latched(void);
uint32_t pid_get_confirmed_line_loss_count(void);
uint32_t pid_get_line_reacquisition_count(void);

// =================== CÁC HÀM XỬ LÝ CHÍNH ===================

/**
 * @brief Tính vị trí tâm vạch line từ 16 mắt cảm biến (từ -75 đến +75, 0 là
 * chính giữa)
 * @param outPosition Con trỏ lưu vị trí tính được
 * @param outBlackCount Con trỏ lưu số lượng mắt đang thấy vạch đen
 * @return true nếu có mắt vượt ngưỡng RAW riêng TH_i; chỉ mắt đó
 *         góp phần vào trọng tâm Analog 0–1000
 */
bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount);

/**
 * @brief Nhận diện và phân loại trường hợp (Case) hiện tại
 */
LineCase_t pid_detect_case(float position, uint8_t blackCount);

/**
 * @brief Hàm chạy 1 chu kỳ dò line (xử lý từng switch case)
 */
void pid_line_follow_step(void);

// =================== HÀM TRUY VẤN DEBUG ===================
LineCase_t pid_get_current_case(void);
const char *pid_get_case_name(LineCase_t c);
float pid_get_last_error(void);
float pid_get_correction(void);
float pid_get_term_p(void);
float pid_get_term_i(void);
float pid_get_term_d(void);
float pid_get_kp(void);
float pid_get_ki(void);
float pid_get_kd(void);
int16_t pid_get_base_speed(void);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
