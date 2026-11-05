#ifndef BLE_CONTROL_H
#define BLE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =================== CÁC HÀM KHỞI TẠO & VÒNG LẶP ===================

/**
 * @brief Khởi tạo BLE Server với dịch vụ UART (Nordic UART Service)
 * @param deviceName Tên hiển thị khi quét BLE (ví dụ: "ESP32_LineRobot")
 */
void ble_control_init(const char* deviceName);

/**
 * @brief Cập nhật trạng thái kết nối và xử lý định kỳ trong hàm loop()
 */
void ble_control_update(void);

/**
 * @brief Kiểm tra xem điện thoại đã kết nối BLE hay chưa
 * @return true nếu đã kết nối
 */
bool ble_is_connected(void);

// =================== CÁC HÀM GỬI DỮ LIỆU QUA BLE ===================

/**
 * @brief Gửi chuỗi ký tự phản hồi về điện thoại qua BLE TX
 * @param message Nội dung chuỗi cần gửi
 */
void ble_send_string(const char* message);

/**
 * @brief Gửi trạng thái 16 mắt cảm biến (chuỗi 0 và 1) lên ứng dụng điện thoại
 */
void ble_send_sensor_states(void);

/**
 * @brief Gửi giá trị PWM 2 động cơ (trái/phải), sai số và trạng thái Case lên BLE
 * @param left PWM bánh trái (-255 đến 255)
 * @param right PWM bánh phải (-255 đến 255)
 * @param error Sai số lệch tâm hiện tại
 * @param caseName Tên trường hợp dò line hiện tại
 */
void ble_send_pwm_telemetry(int16_t left, int16_t right, float error, const char* caseName);

/**
 * Send compact binary debug snapshot over BLE: PWM L/R, encoder pulse counts,
 * 16 raw ADC readings and 16 calibrated readings. Fragment size stays below
 * the default MTU notification payload limit.
 */
void ble_send_debug_telemetry(int16_t left, int16_t right,
                              uint32_t encoder_left, uint32_t encoder_right);

// =================== CẤU HÌNH TỐC ĐỘ ĐỘNG CƠ ===================

/**
 * @brief Lấy giá trị tốc độ động cơ hiện tại
 * @return Tốc độ (0 - 255)
 */
uint8_t ble_get_current_speed(void);

/**
 * @brief Cài đặt tốc độ động cơ dùng khi nhận lệnh điều khiển
 * @param speed Tốc độ mới (0 - 255)
 */
void ble_set_current_speed(uint8_t speed);

#ifdef __cplusplus
}
#endif

#endif // BLE_CONTROL_H
